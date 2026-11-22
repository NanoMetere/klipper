// SPDX-License-Identifier: GPL-3.0-only
//! Virtual interpretation of clocked, compressed stepper messages.
//! Step timing arithmetic is based on the Klipper src/stepper.c queue semantics.
//! Upstream copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>.
//! No physical step pins, motor control or realtime timing.

use crate::step_wire::{self, Message, StepMessages};
use std::collections::VecDeque;

const MAX_BLOCKS: usize = 32_768;
const MAX_DELTA: i64 = 3 << 28;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Wire(step_wire::Error),
    WrongOid,
    InvalidBlock,
    MissingDirection,
    Clock,
    Limit,
    Halted,
}

#[derive(Clone, Copy)]
struct Run {
    base: u64,
    end: u64,
    interval: i64,
    add: i64,
    count: u16,
    emitted: u16,
    positive: bool,
}

impl Run {
    fn after(self, count: u16) -> i64 {
        let count = i64::from(count);
        self.interval * count + self.add * count * (count - 1) / 2
    }

    fn due(self, time: u64) -> u16 {
        if time >= self.end {
            return self.count;
        }
        if time <= self.base {
            return 0;
        }
        // The validated interval is strictly positive on every step. A
        // binary search avoids replaying each microstep after a host pause.
        let elapsed = (time - self.base) as i64;
        let mut lo = self.emitted;
        let mut hi = u32::from(self.count) + 1;
        while u32::from(lo) + 1 < hi {
            let mid = (u32::from(lo) + hi) / 2;
            if self.after(mid as u16) <= elapsed {
                lo = mid as u16;
            } else {
                hi = mid;
            }
        }
        lo
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    pub observed_clock: u64,
    pub last_queued_tick: u64,
    pub position_steps: i64,
    pub pending_blocks: usize,
}

pub struct Stepper {
    oid: u8,
    observed: u64,
    last_queued: u64,
    actual_position: i64,
    last_target: i64,
    direction: Option<bool>,
    pending: VecDeque<Run>,
    halted: bool,
}

impl Stepper {
    pub fn new(oid: u8, initial_clock: u64, initial_position: i64) -> Self {
        Self {
            oid,
            observed: initial_clock,
            last_queued: initial_clock,
            actual_position: initial_position,
            last_target: initial_position,
            direction: None,
            pending: VecDeque::new(),
            halted: false,
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            observed_clock: self.observed,
            last_queued_tick: self.last_queued,
            position_steps: self.actual_position,
            pending_blocks: self.pending.len(),
        }
    }

    pub fn accept_payload(&mut self, codec: &StepMessages, bytes: &[u8]) -> Result<(), Error> {
        self.accept(codec.decode(bytes).map_err(Error::Wire)?)
    }

    pub fn accept(&mut self, message: Message) -> Result<(), Error> {
        if self.halted {
            return Err(Error::Halted);
        }
        match message {
            Message::NextDirection { oid, positive } => {
                if oid != self.oid {
                    return Err(Error::WrongOid);
                }
                self.direction = Some(positive);
            }
            Message::Block {
                oid,
                interval,
                count,
                add,
            } => {
                if oid != self.oid {
                    return Err(Error::WrongOid);
                }
                if self.pending.len() == MAX_BLOCKS {
                    return Err(Error::Limit);
                }
                if count == 0 {
                    return Err(Error::InvalidBlock);
                }
                let positive = self.direction.ok_or(Error::MissingDirection)?;
                let mut run = Run {
                    base: self.last_queued,
                    end: 0,
                    interval: i64::from(interval),
                    add: i64::from(add),
                    count,
                    emitted: 0,
                    positive,
                };
                let last_interval = run.interval + run.add * (i64::from(count) - 1);
                let delta = run.after(count);
                if !(1..MAX_DELTA).contains(&delta) || last_interval <= 0 || interval == 0 {
                    return Err(Error::InvalidBlock);
                }
                let first = run
                    .base
                    .checked_add(u64::from(interval))
                    .ok_or(Error::Clock)?;
                if first <= self.observed {
                    return Err(Error::Clock);
                }
                run.end = run.base.checked_add(delta as u64).ok_or(Error::Clock)?;
                let steps = if positive {
                    i64::from(count)
                } else {
                    -i64::from(count)
                };
                let target = self.last_target.checked_add(steps).ok_or(Error::Limit)?;
                self.pending.push_back(run);
                self.last_target = target;
                self.last_queued = run.end;
            }
        }
        Ok(())
    }

    /// Latch a virtual stop and discard queued future edges at the latest
    /// observed clock. An operator must create a new simulation to re-arm it.
    pub fn halt(&mut self) {
        self.pending.clear();
        self.last_queued = self.observed;
        self.last_target = self.actual_position;
        self.halted = true;
    }

    /// Steps at exactly this clock are included. No physical time passes.
    pub fn advance(&mut self, clock: u64) -> Result<Snapshot, Error> {
        if clock < self.observed {
            return Err(Error::Clock);
        }
        while let Some(run) = self.pending.front_mut() {
            let due = run.due(clock);
            let steps = i64::from(due - run.emitted);
            self.actual_position += if run.positive { steps } else { -steps };
            run.emitted = due;
            if due < run.count {
                break;
            }
            self.pending.pop_front();
        }
        self.observed = clock;
        Ok(self.snapshot())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_virtual_stop_latches_and_discards_future_edges() {
        let mut stepper = Stepper::new(2, 100, 0);
        let dir = Message::NextDirection {
            oid: 2,
            positive: true,
        };
        stepper.accept(dir).unwrap();
        stepper
            .accept(Message::Block {
                oid: 2,
                interval: 10,
                count: 5,
                add: 0,
            })
            .unwrap();
        assert_eq!(stepper.advance(125).unwrap().position_steps, 2);
        stepper.halt();
        assert_eq!(stepper.accept(dir), Err(Error::Halted));
        assert_eq!(stepper.advance(300).unwrap().position_steps, 2);
    }

    #[test]
    fn queued_direction_is_captured_and_boundaries_are_inclusive() {
        let mut stepper = Stepper::new(3, 100, 20);
        let dir = |positive| Message::NextDirection { oid: 3, positive };
        let run = |interval, count, add| Message::Block {
            oid: 3,
            interval,
            count,
            add,
        };
        assert_eq!(stepper.accept(run(10, 3, 1)), Err(Error::MissingDirection));
        stepper.accept(dir(true)).unwrap();
        stepper.accept(run(10, 3, 1)).unwrap(); // steps 110, 121, 133
        stepper.accept(dir(false)).unwrap();
        stepper.accept(run(8, 2, -2)).unwrap(); // steps 141, 147
        assert_eq!(stepper.accept(run(3, 6, -2)), Err(Error::InvalidBlock));
        for (clock, pos) in [(110, 21), (120, 21), (121, 22), (145, 22), (147, 21)] {
            assert_eq!(stepper.advance(clock).unwrap().position_steps, pos);
        }
        assert_eq!(stepper.advance(146), Err(Error::Clock));
        assert_eq!(stepper.accept(run(1, 1, 0)), Ok(()));
        stepper.advance(1000).unwrap();
        assert_eq!(stepper.accept(run(1, 1, 0)), Err(Error::Clock));
    }
}
