// SPDX-License-Identifier: GPL-3.0-only
//! In-memory shared-bus host exercise. No SocketCAN, kernel or device I/O.
//! Round-robin access models host fairness, not physical CAN arbitration.

use crate::{
    can::{Datagram, Demux, Node},
    clock_sync::ReleaseEstimate,
    flight::{ReceiveError, Retry},
    planner::{self, Decision, Planner, Priority, Snapshot, Wire},
    retry_timing::Timer,
    timed_flight::{self, TimedFlight},
};
use std::collections::BTreeMap;

const MAX_NODES: usize = 16;
const RETRY_LIMIT: u8 = 6;

#[derive(Debug)]
pub enum Error {
    Config,
    Time,
    UnknownNode,
    Unavailable,
    Planner(planner::Error),
    Transport(timed_flight::Error),
}

pub struct Peer {
    pub node: Node,
    pub sequence: u64,
    pub receive_window: usize,
    pub clock: Option<ReleaseEstimate>,
}

struct Link {
    peer: Peer,
    planner: Planner,
    flight: TimedFlight,
    retries: u8,
    failed: bool,
}

pub struct Bus {
    by_response: BTreeMap<u16, usize>,
    wire: Wire,
    peers: Vec<Link>,
    demux: Demux,
    turn: usize,
    time: f64,
    idle: f64,
}

#[derive(Debug)]
pub enum Action {
    Transmit {
        node: Node,
        datagrams: Vec<Datagram>,
        completes_at: f64,
        retry: Option<Retry>,
    },
    WaitUntil(f64),
    PeerTimedOut(Node),
    Idle,
}

#[derive(Debug, Default, PartialEq, Eq)]
pub struct Received {
    pub accepted_payloads: Vec<Vec<u8>>,
    pub out_of_window: usize,
    pub discarded_bytes: usize,
}

impl Bus {
    pub fn new(
        bitrate: u32,
        peers: Vec<Peer>,
        lanes: usize,
        capacity: usize,
    ) -> Result<Self, Error> {
        if bitrate == 0 || peers.is_empty() || peers.len() > MAX_NODES {
            return Err(Error::Config);
        }
        let mut result = Self {
            by_response: BTreeMap::new(),
            wire: Wire::ClassicCan { bitrate },
            peers: Vec::new(),
            demux: Demux::default(),
            turn: 0,
            time: 0.0,
            idle: 0.0,
        };
        for peer in peers {
            result
                .demux
                .register(peer.node)
                .map_err(|_| Error::Config)?;
            result
                .by_response
                .insert(peer.node.response_id(), result.peers.len());
            let planner = Planner::new(lanes, capacity).map_err(Error::Planner)?;
            let flight =
                TimedFlight::established(peer.sequence, result.wire).map_err(Error::Transport)?;
            result.peers.push(Link {
                peer,
                planner,
                flight,
                retries: 0,
                failed: false,
            });
        }
        Ok(result)
    }

    pub fn enqueue(
        &mut self,
        node: Node,
        lane: usize,
        release: u64,
        priority: Priority,
        payload: &[u8],
    ) -> Result<(), Error> {
        let idx = *self
            .by_response
            .get(&node.response_id())
            .ok_or(Error::UnknownNode)?;
        let link = &mut self.peers[idx];
        if link.failed {
            return Err(Error::Unavailable);
        }
        link.planner
            .push(lane, release, priority, payload)
            .map_err(Error::Planner)
    }

    pub fn receive(&mut self, datagram: Datagram, now: f64) -> Result<Option<Received>, Error> {
        self.advance(now)?;
        let Some(index) = self.by_response.get(&datagram.id()) else {
            return Ok(None);
        };
        let link = &mut self.peers[*index];
        if link.failed {
            return Ok(None);
        }
        let mut result = Received::default();
        let mut error = None;
        let stats = self
            .demux
            .feed(datagram, |frame| {
                if error.is_some() {
                    return;
                }
                match link.flight.observe(frame, now) {
                    Ok(progress) => {
                        if progress.retired != 0 {
                            link.retries = 0;
                        }
                        if !frame.payload.is_empty() {
                            result.accepted_payloads.push(frame.payload.to_vec());
                        }
                    }
                    Err(timed_flight::Error::Receive(ReceiveError::OutsideWindow)) => {
                        result.out_of_window += 1
                    }
                    Err(e) => error = Some(Error::Transport(e)),
                }
            })
            .ok_or(Error::Config)?;
        if let Some(error) = error {
            return Err(error);
        }
        result.discarded_bytes = stats.discarded;
        Ok(Some(result))
    }

    pub fn poll(&mut self, now: f64) -> Result<Action, Error> {
        self.advance(now)?;
        if now < self.idle {
            return Ok(Action::WaitUntil(self.idle));
        }
        let mut next = None::<f64>;
        let mut consider = |t: f64| {
            next = Some(next.map_or(t, |old| old.min(t)));
        };
        for offset in 0..self.peers.len() {
            let index = (self.turn + offset) % self.peers.len();
            let link = &mut self.peers[index];
            if link.failed {
                continue;
            }
            let node = link.peer.node;
            let timer = link.flight.timer();
            if timer == Timer::Fast || matches!(timer, Timer::At(t) if now >= t) {
                if link.retries == RETRY_LIMIT {
                    link.failed = true;
                    self.demux.remove(node);
                    return Ok(Action::PeerTimedOut(node));
                }
                let mut frames = Vec::new();
                let mut bytes = 0;
                let reason = link
                    .flight
                    .retry_if_due(now, |data| {
                        bytes = data.len();
                        frames.extend(node.fragments(data).expect("bounded flight"));
                        Ok(bytes)
                    })
                    .map_err(Error::Transport)?;
                link.retries += 1;
                return self.transmitted(index, now, bytes, frames, Some(reason));
            }
            if let Timer::At(t) = timer {
                consider(t);
            }
            let decision = link
                .planner
                .plan(
                    self.wire,
                    link.peer.clock,
                    Snapshot {
                        now,
                        link_idle: self.idle,
                        burst_bytes: 0,
                        in_flight: link.flight.pending_frames(),
                        outstanding_bytes: link.flight.outstanding_bytes(),
                        prior_ack_bytes: link.flight.prior_ack_bytes(),
                        receive_window: link.peer.receive_window,
                    },
                )
                .map_err(Error::Planner)?;
            match decision {
                Decision::Send(payload) => {
                    let mut frames = Vec::new();
                    let mut bytes = 0;
                    link.flight
                        .send(payload.as_slice(), link.peer.receive_window, now, |data| {
                            bytes = data.len();
                            frames.extend(node.fragments(data).expect("bounded Flight send"));
                            Ok(bytes)
                        })
                        .map_err(Error::Transport)?;
                    return self.transmitted(index, now, bytes, frames, None);
                }
                Decision::SleepUntil(t) | Decision::AwaitAck { check_at: t } => consider(t),
                Decision::Idle => {}
                Decision::DrainBurst => return Err(Error::Config),
            }
        }
        Ok(next.map_or(Action::Idle, |t| Action::WaitUntil(t.max(now.next_up()))))
    }

    fn transmitted(
        &mut self,
        index: usize,
        start: f64,
        bytes: usize,
        datagrams: Vec<Datagram>,
        retry: Option<Retry>,
    ) -> Result<Action, Error> {
        self.idle = start + self.wire.seconds(bytes).map_err(Error::Planner)?;
        self.turn = (index + 1) % self.peers.len();
        Ok(Action::Transmit {
            node: self.peers[index].peer.node,
            datagrams,
            completes_at: self.idle,
            retry,
        })
    }

    fn advance(&mut self, now: f64) -> Result<(), Error> {
        if !(0.0..1e9).contains(&now) || now == 0.0 || now < self.time {
            return Err(Error::Time);
        }
        self.time = now;
        Ok(())
    }
}
