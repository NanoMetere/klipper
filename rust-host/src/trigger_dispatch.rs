// SPDX-License-Identifier: GPL-3.0-only
//! In-memory trigger propagation and watchdog extension decisions.
//! Adapted from Klipper klippy/chelper/trdispatch.c.
//! Upstream copyright (C) 2021 Kevin O'Connor <kevin@koconnor.net>.
//! This module emits decisions, never device I/O.

use crate::{
    clock_sync::ReleaseEstimate,
    planner::clock_from_time,
    trigger_sync::{Reason, State, Timing},
};
const MAX_PEERS: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Configuration,
    Peer,
    Clock,
}

#[derive(Debug, Clone, Copy)]
pub struct Peer {
    pub oid: u8,
    pub last_clock: u64,
    pub timeout: u32,
    pub timing: Timing,
    pub estimate: ReleaseEstimate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Extend {
        peer: usize,
        oid: u8,
        clock: u64,
    },
    Trigger {
        peer: usize,
        oid: u8,
        reason: Reason,
    },
}

pub struct Dispatch {
    peers: Vec<Peer>,
    propagation_reason: Reason,
    terminal: Option<(usize, Reason)>,
}

impl Dispatch {
    pub fn new(peers: Vec<Peer>, cause: Reason) -> Result<Self, Error> {
        if !(1..=MAX_PEERS).contains(&peers.len())
            || peers.iter().any(|p| {
                p.timeout == 0
                    || p.timing.extend_threshold == 0
                    || p.timing.expires <= p.last_clock
                    || p.timing.expires - p.last_clock != u64::from(p.timeout)
                    || status_time(p).is_err()
            })
        {
            return Err(Error::Configuration);
        }
        Ok(Self {
            peers,
            propagation_reason: cause,
            terminal: None,
        })
    }

    pub fn terminal(&self) -> Option<(usize, Reason)> {
        self.terminal
    }

    /// Accept a report from its specific simulated link. The same OID may
    /// be present on different links. Once stopped, later reports are inert.
    pub fn observe(
        &mut self,
        peer: usize,
        status: State,
        host_time: f64,
        estimate: ReleaseEstimate,
    ) -> Result<Vec<Action>, Error> {
        let incoming = self.peers.get(peer).ok_or(Error::Peer)?;
        if status.oid != incoming.oid || self.terminal.is_some() {
            return Ok(Vec::new());
        }
        if !status.can_trigger {
            self.terminal = Some((peer, status.reason));
            return Ok(self
                .peers
                .iter()
                .enumerate()
                .map(|(peer, p)| Action::Trigger {
                    peer,
                    oid: p.oid,
                    reason: self.propagation_reason,
                })
                .collect());
        }
        let anchor = clock_from_time(estimate, host_time).map_err(|_| Error::Clock)?;
        let delta = i64::from(status.clock.wrapping_sub(anchor as u32) as i32);
        let last_clock = anchor.checked_add_signed(delta).ok_or(Error::Clock)?;
        let updated = Peer {
            estimate,
            last_clock,
            ..*incoming
        };
        let mut min = f64::INFINITY;
        let mut second = f64::INFINITY;
        let mut min_index = 0;
        for (index, item) in self.peers.iter().enumerate() {
            let time = status_time(if index == peer { &updated } else { item })?;
            if time < second {
                second = time;
                if time < min {
                    (min, second, min_index) = (time, min, index);
                }
            }
        }
        if second.is_infinite() {
            second = min;
        }
        let mut actions = Vec::new();
        for (index, item) in self.peers.iter().enumerate() {
            let item = if index == peer { &updated } else { item };
            let support = if index == min_index { second } else { min };
            let expire = clock_from_time(item.estimate, support)
                .map_err(|_| Error::Clock)?
                .checked_add(u64::from(item.timeout))
                .ok_or(Error::Clock)?;
            let threshold = item
                .timing
                .expires
                .checked_add(u64::from(item.timing.extend_threshold))
                .ok_or(Error::Clock)?;
            if expire >= threshold {
                actions.push(Action::Extend {
                    peer: index,
                    oid: item.oid,
                    clock: expire,
                });
            }
        }
        // No partial updates if any clock projection was invalid.
        self.peers[peer] = updated;
        for &action in &actions {
            if let Action::Extend { peer, clock, .. } = action {
                self.peers[peer].timing.expires = clock;
            }
        }
        Ok(actions)
    }
}

fn status_time(p: &Peer) -> Result<f64, Error> {
    if !(1.0..=1e12).contains(&p.estimate.hz) {
        return Err(Error::Clock);
    }
    let latest = u64::try_from(p.estimate.ticks).map_err(|_| Error::Clock)?;
    let distance =
        i64::try_from(i128::from(p.last_clock) - i128::from(latest)).map_err(|_| Error::Clock)?;
    let time = p.estimate.at + distance as f64 / p.estimate.hz;
    time.is_finite().then_some(time).ok_or(Error::Clock)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stalled_peer_caps_progress_and_stop_propagates_once() {
        let ce = ReleaseEstimate {
            at: 10.,
            hz: 1e6,
            ticks: 4_300_000_000,
        };
        let start = ce.ticks as u64;
        let peers = (0..3)
            .map(|idx| {
                let timing = Timing::for_peer(start, ce.hz, idx, 3).unwrap();
                Peer {
                    oid: 4,
                    last_clock: start,
                    timing,
                    timeout: 25_000,
                    estimate: ce,
                }
            })
            .collect();
        let mut d = Dispatch::new(peers, Reason::HostRequest).unwrap();
        let ok = State {
            oid: 4,
            can_trigger: true,
            reason: Reason::Other(0),
            clock: (start + 10_000) as u32,
        };
        assert!(d.observe(0, ok, 10.01, ce).unwrap().is_empty());
        assert_eq!(
            d.observe(1, ok, 10.01, ce).unwrap(),
            [Action::Extend {
                peer: 2,
                oid: 4,
                clock: start + 35_000
            }]
        );
        let stop = State {
            can_trigger: false,
            reason: Reason::CommsTimeout,
            ..ok
        };
        assert_eq!(d.observe(2, stop, 10.02, ce).unwrap().len(), 3);
        assert_eq!(d.terminal(), Some((2, Reason::CommsTimeout)));
        assert!(d.observe(0, ok, 10.03, ce).unwrap().is_empty());
    }
}
