// SPDX-License-Identifier: GPL-3.0-only
//! Stock C step queue horizon for an inert offline clock trace.
//! Based on Klipper's chelper/stepcompress.c (GPL-3.0-only).
use super::*;
use crate::step_compress::compress_verified_prefix;

const HORIZON: u64 = 3 << 28;
const MAX_BLOCK: usize = 65_535;
const EXTEND_TRIM: usize = MAX_BLOCK + 2000;
const MAX_SLOTS: usize = 1 << 17;
pub(crate) const MAX_INPUT: usize = 1_000_000;
const MAX_COMMANDS: usize = 262_144;

pub(crate) struct LongEncoder {
    clock: Clock,
    max_error: u32,
    invert: bool,
    filter: Filter,
    output: Vec<Command>,
    requested: Vec<u64>,
    head: usize,
    logical_slots: usize,
    direction: Option<bool>,
    epoch: Option<f64>,
    count: usize,
}

impl LongEncoder {
    pub(crate) fn new(clock: Clock, max_error: u32, invert: bool) -> Result<Self, Error> {
        if max_error > 200_000 {
            return Err(Error::Capacity);
        }
        Ok(Self {
            clock,
            max_error,
            invert,
            filter: Filter::default(),
            output: Vec::new(),
            requested: Vec::new(),
            head: 0,
            logical_slots: 0,
            direction: None,
            epoch: None,
            count: 0,
        })
    }

    fn record(&mut self, command: Command) -> Result<(), Error> {
        if self.output.len() == MAX_COMMANDS {
            return Err(Error::Capacity);
        }
        self.output.push(command);
        Ok(())
    }

    fn block(&mut self, block: Block) -> Result<(), Error> {
        self.record(Command::Steps(block))?;
        self.clock.last_encoded = block.last_clock;
        self.epoch = None;
        Ok(())
    }

    fn flush_until(&mut self, cutoff: u64) -> Result<(), Error> {
        while self.head != self.requested.len() && self.clock.last_encoded < cutoff {
            let remaining = &self.requested[self.head..];
            let prefix = &remaining[..remaining.len().min(MAX_BLOCK)];
            let block = compress_verified_prefix(self.clock.last_encoded, prefix, self.max_error)
                .map_err(Error::Compress)?;
            self.block(block)?;
            self.head += usize::from(block.count);
        }
        if self.head == self.requested.len() {
            self.head = 0;
            self.requested.clear();
        }
        Ok(())
    }

    // Use stock's logical capacity, independent of any allocator over-allocation.
    fn append_extend(&mut self, clock: u64) -> Result<(), Error> {
        if self.requested.len() - self.head > EXTEND_TRIM {
            let cutoff = self.requested[self.requested.len() - MAX_BLOCK];
            self.flush_until(cutoff)?;
        }
        if self.requested.len() == self.logical_slots {
            if self.head != 0 {
                self.requested.copy_within(self.head.., 0);
                self.requested.truncate(self.requested.len() - self.head);
                self.head = 0;
            } else {
                let new_len = (self.logical_slots * 2).max(1024);
                if new_len > MAX_SLOTS {
                    return Err(Error::Capacity);
                }
                self.requested.reserve_exact(new_len - self.requested.len());
                self.logical_slots = new_len;
            }
        }
        self.requested.push(clock);
        Ok(())
    }

    fn commit(&mut self, step: Step) -> Result<(), Error> {
        if self.direction != Some(step.positive) {
            self.flush_until(u64::MAX)?;
            self.record(Command::SetDirection(step.positive ^ self.invert))?;
            self.direction = Some(step.positive);
        }
        let delta = step
            .clock
            .checked_sub(self.clock.last_encoded)
            .ok_or(Error::Nonmonotonic)?;
        if delta == 0 {
            return Err(Error::Nonmonotonic);
        }
        if delta >= HORIZON {
            self.flush_until(step.clock - HORIZON + 1)?;
            let current = step.clock - self.clock.last_encoded;
            if current >= HORIZON {
                // The offline schema cannot express an independent MCU req_clock
                // that disambiguates a delta larger than the interval field.
                let interval = u32::try_from(current).map_err(|_| Error::InvalidTime)?;
                return self.block(Block {
                    interval,
                    count: 1,
                    add: 0,
                    last_clock: step.clock,
                });
            }
            return self.append_extend(step.clock);
        }
        if self.requested.len() == self.logical_slots {
            return self.append_extend(step.clock);
        }
        self.requested.push(step.clock);
        Ok(())
    }

    pub(crate) fn push(&mut self, events: &[Event]) -> Result<(), Error> {
        if events.len() > MAX_INPUT - self.count {
            return Err(Error::Capacity);
        }
        self.count += events.len();
        for &event in events {
            let send = match event {
                Event::Commit => self.filter.commit(),
                Event::Crossing {
                    direction_positive,
                    segment_time,
                    offset,
                } => {
                    let epoch = if let Some(epoch) = self.epoch {
                        epoch
                    } else {
                        let epoch = self.clock.epoch()?;
                        self.epoch = Some(epoch);
                        epoch
                    };
                    self.filter.append(
                        Step {
                            clock: self.clock.tick_from_epoch(epoch, segment_time, offset)?,
                            positive: direction_positive,
                        },
                        self.clock.hz,
                    )?
                }
            };
            if let Some(step) = send {
                self.commit(step)?;
            }
        }
        Ok(())
    }

    pub(crate) fn finish(mut self) -> Result<Vec<Command>, Error> {
        if let Some(step) = self.filter.commit() {
            self.commit(step)?;
        }
        self.flush_until(u64::MAX)?;
        Ok(self.output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_run_count_limits_and_memory_bound() {
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: 20e6,
        };
        let mut e = LongEncoder::new(clock, 0, false).unwrap();
        for i in 1..=140_000_u64 {
            e.commit(Step {
                clock: i * 170,
                positive: true,
            })
            .unwrap();
            assert!(e.logical_slots <= MAX_SLOTS);
            assert!(e.requested.len() <= e.logical_slots);
        }
        let output = e.finish().unwrap();
        assert_eq!(
            output,
            [
                Command::SetDirection(true),
                Command::Steps(Block {
                    interval: 170,
                    count: 65_535,
                    add: 0,
                    last_clock: 11_140_950
                }),
                Command::Steps(Block {
                    interval: 170,
                    count: 65_535,
                    add: 0,
                    last_clock: 22_281_900
                }),
                Command::Steps(Block {
                    interval: 170,
                    count: 8_930,
                    add: 0,
                    last_clock: 23_800_000
                })
            ]
        );
        let mut capped = LongEncoder::new(clock, 0, false).unwrap();
        capped.count = MAX_INPUT - 1;
        assert_eq!(capped.push(&[Event::Commit; 2]), Err(Error::Capacity));
        assert!(capped.output.is_empty() && capped.requested.is_empty());
    }

    #[test]
    fn unrepresentable_far_intervals_fail_without_wrap() {
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: 20e6,
        };
        let mut e = LongEncoder::new(clock, 0, false).unwrap();
        assert_eq!(
            e.commit(Step {
                clock: u64::from(u32::MAX) + 1,
                positive: true
            }),
            Err(Error::InvalidTime)
        );
    }
}
