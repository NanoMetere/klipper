// SPDX-License-Identifier: GPL-3.0-only
//! Offline transmit planning, adapted from Klippy's klippy/chelper/serialqueue.c.
//! Upstream copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>.
//! Does not write to a device, manage ACKs, or own the transmission clock.

use crate::flight::FRAME_WINDOW;
use crate::{clock_sync::ReleaseEstimate, MAX_FRAME, MAX_PAYLOAD, MIN_FRAME};
use std::collections::VecDeque;

const CLOCK_LIMIT: u64 = i64::MAX as u64;
const BACKGROUND_KEY: u64 = 0x7fff_ffff_0000_0000;
const RELEASE_RANGE: u64 = 3 << 29;
const LEAD: f64 = 0.100;
const BACKGROUND_EXTRA: f64 = 0.005;
const MAX_LANES: usize = 32;
const MAX_COMMANDS: usize = 4096;

#[derive(Clone, Copy, Debug)]
pub enum Wire {
    Pty,
    Uart8N1 { baud: u32 },
    ClassicCan { bitrate: u32 },
}

impl Wire {
    pub(crate) fn seconds(self, bytes: usize) -> Result<f64, Error> {
        match self {
            Self::Pty => Ok(0.0),
            Self::Uart8N1 { baud } if baud != 0 => Ok((10.0 / f64::from(baud)) * bytes as f64),
            Self::ClassicCan { bitrate } if bitrate != 0 => {
                // Stock minimum-bit estimate; not measured on a physical CAN bus.
                let bits = (bytes * 8 + bytes.div_ceil(8) * 47).saturating_sub(4);
                Ok((1.0 / f64::from(bitrate)) * bits as f64)
            }
            _ => Err(Error::InvalidLink),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Priority {
    Deadline(u64),
    Background,
}

impl Priority {
    fn key(self) -> u64 {
        match self {
            Self::Deadline(tick) => tick,
            Self::Background => BACKGROUND_KEY,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidLink,
    InvalidClock,
    InvalidLane,
    InvalidPayload,
    InvalidPriority,
    QueueFull,
}

#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub now: f64,
    pub link_idle: f64,
    /// Number of bytes already assembled in this driver's current burst.
    pub burst_bytes: usize,
    /// Frames sent and not yet acknowledged.
    pub in_flight: usize,
    pub outstanding_bytes: usize,
    /// Nonzero when the preceding transmitted block has not been fully ACKed.
    pub prior_ack_bytes: usize,
    /// Zero disables the receive byte window, as in stock.
    pub receive_window: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Payload {
    bytes: [u8; MAX_PAYLOAD],
    len: usize,
}

impl Payload {
    fn new(slice: &[u8]) -> Result<Self, Error> {
        if slice.is_empty() || slice.len() > MAX_PAYLOAD {
            return Err(Error::InvalidPayload);
        }
        let mut bytes = [0; MAX_PAYLOAD];
        bytes[..slice.len()].copy_from_slice(slice);
        Ok(Self {
            bytes,
            len: slice.len(),
        })
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Decision {
    Send(Payload),
    SleepUntil(f64),
    DrainBurst,
    AwaitAck { check_at: f64 },
    Idle,
}

struct Command {
    release: u64,
    priority: Priority,
    data: Payload,
}

#[derive(Default)]
struct Lane {
    pending: VecDeque<Command>,
    ready: VecDeque<Command>,
}

/// Queue bounds are explicit. One FIFO lane models one upstream command queue.
/// A successful Send consumes those commands. This offline API must not be
/// used as a driver without separately implementing transactional write/error
/// handling, ACK progression and retransmission.
pub struct Planner {
    lanes: Vec<Lane>,
    pending: VecDeque<usize>,
    ready: VecDeque<usize>,
    ready_bytes: usize,
    total: usize,
    capacity: usize,
}

impl Planner {
    pub fn new(lanes: usize, capacity: usize) -> Result<Self, Error> {
        if !(1..=MAX_LANES).contains(&lanes) {
            return Err(Error::InvalidLane);
        }
        if !(1..=MAX_COMMANDS).contains(&capacity) {
            return Err(Error::QueueFull);
        }
        Ok(Self {
            lanes: (0..lanes).map(|_| Lane::default()).collect(),
            pending: VecDeque::with_capacity(lanes),
            ready: VecDeque::with_capacity(lanes),
            ready_bytes: 0,
            total: 0,
            capacity,
        })
    }

    pub fn push(
        &mut self,
        lane: usize,
        release: u64,
        priority: Priority,
        bytes: &[u8],
    ) -> Result<(), Error> {
        let q = self.lanes.get_mut(lane).ok_or(Error::InvalidLane)?;
        if self.total == self.capacity {
            return Err(Error::QueueFull);
        }
        if release >= BACKGROUND_KEY {
            return Err(Error::InvalidClock);
        }
        let release = match priority {
            Priority::Deadline(deadline) if deadline < BACKGROUND_KEY => {
                release.max(deadline.saturating_sub(RELEASE_RANGE))
            }
            Priority::Deadline(_) => return Err(Error::InvalidPriority),
            Priority::Background => release,
        };
        let data = Payload::new(bytes)?;
        if q.pending.is_empty() {
            self.pending.push_back(lane);
        }
        q.pending.push_back(Command {
            release,
            priority,
            data,
        });
        self.total += 1;
        Ok(())
    }

    fn make_ready(&mut self, tick: u64) -> u64 {
        let mut stalled = CLOCK_LIMIT;
        for _ in 0..self.pending.len() {
            let lane = self.pending.pop_front().unwrap();
            let q = &mut self.lanes[lane];
            let was_empty = q.ready.is_empty();
            while let Some(command) = q.pending.front() {
                if command.release > tick {
                    stalled = stalled.min(command.release);
                    break;
                }
                let command = q.pending.pop_front().unwrap();
                self.ready_bytes += command.data.len;
                q.ready.push_back(command);
            }
            if !q.pending.is_empty() {
                self.pending.push_back(lane);
            }
            if was_empty && !q.ready.is_empty() {
                self.ready.push_back(lane);
            }
        }
        stalled
    }

    fn batch(&mut self) -> Decision {
        let mut output = Payload {
            bytes: [0; MAX_PAYLOAD],
            len: 0,
        };
        while let Some((index, &lane)) = self
            .ready
            .iter()
            .enumerate()
            .min_by_key(|(_, lane)| self.lanes[**lane].ready[0].priority.key())
        {
            let q = &mut self.lanes[lane].ready;
            let len = q[0].data.len;
            if output.len + len > MAX_PAYLOAD {
                // Preserve priority; do not fill the gap by skipping this command.
                break;
            }
            let command = q.pop_front().unwrap();
            output.bytes[output.len..output.len + len].copy_from_slice(command.data.as_slice());
            output.len += len;
            self.ready_bytes -= len;
            self.total -= 1;
            if q.is_empty() {
                self.ready.remove(index);
            }
        }
        debug_assert!(output.len > 0);
        Decision::Send(output)
    }

    pub fn plan(
        &mut self,
        wire: Wire,
        estimate: Option<ReleaseEstimate>,
        s: Snapshot,
    ) -> Result<Decision, Error> {
        if !s.now.is_finite()
            || s.now <= 0.0
            || !s.link_idle.is_finite()
            || s.link_idle < 0.0
            || s.burst_bytes > MAX_FRAME * FRAME_WINDOW
            || s.in_flight > FRAME_WINDOW
            || s.outstanding_bytes > MAX_FRAME * FRAME_WINDOW
            || s.prior_ack_bytes > MAX_FRAME
        {
            return Err(Error::InvalidLink);
        }
        let idle = s.now.max(s.link_idle) + wire.seconds(s.burst_bytes + MIN_FRAME)?;
        let ack = if let Some(clock) = estimate {
            clock_from_time(clock, idle)?
        } else {
            0
        };
        let stalled = self.make_ready(ack);
        let need_ack = s.outstanding_bytes + s.prior_ack_bytes + MAX_FRAME;
        if s.in_flight >= FRAME_WINDOW
            || (s.in_flight != 0 && s.receive_window != 0 && need_ack > s.receive_window)
        {
            return Ok(Decision::AwaitAck {
                check_at: s.now + 0.250,
            });
        }
        if self.ready_bytes >= MAX_PAYLOAD {
            return Ok(self.batch());
        }
        let Some(clock) = estimate else {
            return Ok(if self.ready_bytes != 0 {
                self.batch()
            } else {
                Decision::Idle
            });
        };
        let mut next = CLOCK_LIMIT;
        let background_time = if s.burst_bytes != 0 {
            idle
        } else {
            s.link_idle
        };
        for &lane in &self.ready {
            let command = &self.lanes[lane].ready[0];
            let goal = match command.priority {
                Priority::Deadline(tick) => tick,
                Priority::Background => {
                    clock_from_time(clock, background_time + (LEAD + BACKGROUND_EXTRA))?
                }
            };
            next = next.min(goal);
        }
        let lead_ticks = (LEAD * clock.hz) as u64;
        if self.ready_bytes != 0 && next <= ack + lead_ticks {
            return Ok(self.batch());
        }
        if s.burst_bytes != 0 {
            return Ok(Decision::DrainBurst);
        }
        if self.total == 0 {
            return Ok(Decision::Idle);
        }
        let want = next.saturating_sub(lead_ticks).min(stalled);
        Ok(Decision::SleepUntil(
            idle + want.saturating_sub(ack) as f64 / clock.hz,
        ))
    }
}

/// Upstream round-to-nearest formula for nonnegative projected MCU clocks.
/// Out-of-range upstream wraparound is rejected in this offline planner.
pub fn clock_from_time(estimate: ReleaseEstimate, host_time: f64) -> Result<u64, Error> {
    if !estimate.hz.is_finite()
        || !(1.0..=1e12).contains(&estimate.hz)
        || !estimate.at.is_finite()
        || !host_time.is_finite()
    {
        return Err(Error::InvalidClock);
    }
    let ticks = (host_time - estimate.at) * estimate.hz + 0.5;
    if !ticks.is_finite() || ticks <= i64::MIN as f64 || ticks >= -(i64::MIN as f64) {
        return Err(Error::InvalidClock);
    }
    let result = i128::from(estimate.ticks) + i128::from(ticks as i64);
    if !(0..i128::from(BACKGROUND_KEY)).contains(&result) {
        return Err(Error::InvalidClock);
    }
    Ok(result as u64)
}

#[cfg(test)]
mod tests {
    use super::*;
    const S: Snapshot = Snapshot {
        now: 10.0,
        link_idle: 10.0,
        burst_bytes: 0,
        in_flight: 0,
        outstanding_bytes: 0,
        prior_ack_bytes: 0,
        receive_window: 0,
    };
    const CE: Option<ReleaseEstimate> = Some(ReleaseEstimate {
        at: 10.0,
        ticks: 5_000_000,
        hz: 1e6,
    });

    #[test]
    fn releases_and_packs_multiple_lanes_in_priority_order() {
        let mut p = Planner::new(2, 8).unwrap();
        p.push(0, 5_000_001, Priority::Deadline(5_020_000), b"A")
            .unwrap();
        p.push(0, 4_000_000, Priority::Deadline(1), b"B").unwrap();
        p.push(1, 5_000_000, Priority::Deadline(5_090_000), b"C")
            .unwrap();
        let first = p.plan(Wire::Pty, CE, S).unwrap();
        assert!(matches!(first, Decision::Send(x) if x.as_slice() == b"C"));
        let wait = p.plan(Wire::Pty, CE, S).unwrap();
        assert!(matches!(wait, Decision::SleepUntil(t) if (t - 10.000001).abs() < 1e-12));
        let next = p
            .plan(Wire::Pty, CE, Snapshot { now: 10.001, ..S })
            .unwrap();
        assert!(matches!(next, Decision::Send(x) if x.as_slice() == b"AB"));
    }

    #[test]
    fn bounded_admission_and_ack_backpressure() {
        let mut p = Planner::new(2, 2).unwrap();
        p.push(0, 0, Priority::Deadline(0), &[1; 40]).unwrap();
        p.push(1, 0, Priority::Deadline(1), &[2; 30]).unwrap();
        assert_eq!(
            p.push(1, 0, Priority::Deadline(2), b"C"),
            Err(Error::QueueFull)
        );
        assert_eq!(
            p.plan(Wire::Pty, None, Snapshot { in_flight: 12, ..S }),
            Ok(Decision::AwaitAck { check_at: 10.25 })
        );
        let x = p.plan(Wire::Pty, None, S).unwrap();
        assert!(matches!(x, Decision::Send(x) if x.as_slice() == [1; 40]));
        let window = Snapshot {
            in_flight: 1,
            receive_window: 105,
            outstanding_bytes: 41,
            prior_ack_bytes: 1,
            ..S
        };
        assert_eq!(
            p.plan(Wire::Pty, None, window),
            Ok(Decision::AwaitAck { check_at: 10.25 })
        );
        let x = p.plan(Wire::Pty, None, S).unwrap();
        assert!(matches!(x, Decision::Send(x) if x.as_slice() == [2; 30]));
    }

    #[test]
    fn clock_horizon_rejects_wrapping_and_bad_inputs() {
        assert_eq!(clock_from_time(CE.unwrap(), 10.000001), Ok(5_000_001));
        assert_eq!(clock_from_time(CE.unwrap(), 0.0), Err(Error::InvalidClock));
        assert_eq!(
            clock_from_time(CE.unwrap(), f64::INFINITY),
            Err(Error::InvalidClock)
        );
    }
}
