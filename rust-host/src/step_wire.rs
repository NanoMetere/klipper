// SPDX-License-Identifier: GPL-3.0-only
//! Offline encoding of checked step trace commands with a firmware dictionary.
//! Encoded payloads do not configure an MCU or drive a device.

use crate::{identify::Dictionary, step_clock::Command, vlq};

pub const SET_DIR: &str = "set_next_step_dir oid=%c dir=%c";
pub const QUEUE: &str = "queue_step oid=%c interval=%u count=%hu add=%hi";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    MissingFormat(&'static str),
    BadFormatId,
    InvalidTrace,
}

pub struct StepMessages {
    set_dir: i32,
    queue: i32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Payload {
    pub bytes: Vec<u8>,
    pub required_tick: u64,
    pub earliest_tick: u64,
}

impl StepMessages {
    pub fn from_dictionary(dictionary: &Dictionary) -> Result<Self, Error> {
        let get = |key| {
            dictionary
                .commands
                .get(key)
                .copied()
                .ok_or(Error::MissingFormat(key))
        };
        let (set_dir, queue) = (get(SET_DIR)?, get(QUEUE)?);
        if set_dir == queue {
            return Err(Error::BadFormatId);
        }
        Ok(Self { set_dir, queue })
    }

    /// Encode one prepared per-stepper trace. Commands are not framed or
    /// transmitted by this API. The caller must not reorder a motor's lane.
    pub fn payloads(
        &self,
        oid: u8,
        first_tick: u64,
        trace: &[Command],
    ) -> Result<Vec<Payload>, Error> {
        if trace.len() > 65_535 {
            return Err(Error::InvalidTrace);
        }
        let mut last = first_tick;
        let mut direction = None;
        let mut output = Vec::with_capacity(trace.len());
        for cmd in trace {
            let mut bytes = Vec::with_capacity(18);
            let mut earliest = 0;
            let required = last;
            match *cmd {
                Command::SetDirection(positive) => {
                    if direction == Some(positive) {
                        return Err(Error::InvalidTrace);
                    }
                    direction = Some(positive);
                    for v in [self.set_dir, i32::from(oid), i32::from(positive)] {
                        vlq::encode(&mut bytes, v);
                    }
                }
                Command::Steps(b) => {
                    if direction.is_none() || b.count == 0 || !(1..(3 << 28)).contains(&b.interval)
                    {
                        return Err(Error::InvalidTrace);
                    }
                    let n = i64::from(b.count);
                    let step = i64::from(b.interval);
                    let add = i64::from(b.add);
                    let last_spacing = step + add * (n - 1);
                    let delta = n * step + add * n * (n - 1) / 2;
                    if !(1..0x8000_0000).contains(&last_spacing)
                        || !(1..(3_i64 << 28)).contains(&delta)
                        || last.checked_add(delta as u64) != Some(b.last_clock)
                    {
                        return Err(Error::InvalidTrace);
                    }
                    earliest = last;
                    last = b.last_clock;
                    for v in [
                        self.queue,
                        i32::from(oid),
                        b.interval as i32,
                        i32::from(b.count),
                        i32::from(b.add),
                    ] {
                        vlq::encode(&mut bytes, v);
                    }
                }
            }
            output.push(Payload {
                bytes,
                earliest_tick: earliest,
                required_tick: required,
            });
        }
        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::step_compress::Block;

    #[test]
    fn rejects_no_direction_and_incorrect_end_tick() {
        let m = StepMessages {
            set_dir: 1,
            queue: 2,
        };
        let b = Command::Steps(Block {
            interval: 100,
            count: 3,
            add: -10,
            last_clock: 271,
        });
        assert_eq!(m.payloads(1, 0, &[b]), Err(Error::InvalidTrace));
        assert_eq!(
            m.payloads(1, 0, &[Command::SetDirection(true), b]),
            Err(Error::InvalidTrace)
        );
    }

    #[test]
    fn preserves_release_metadata_in_one_motor_lane() {
        let m = StepMessages {
            set_dir: 1,
            queue: 2,
        };
        let result = m
            .payloads(
                8,
                100,
                &[
                    Command::SetDirection(false),
                    Command::Steps(Block {
                        interval: 100,
                        count: 3,
                        add: -10,
                        last_clock: 370,
                    }),
                    Command::SetDirection(true),
                ],
            )
            .unwrap();
        assert_eq!(
            result[0],
            Payload {
                bytes: vec![1, 8, 0],
                earliest_tick: 0,
                required_tick: 100
            }
        );
        assert_eq!(
            result[1],
            Payload {
                bytes: vec![2, 8, 0x80, 100, 3, 118],
                earliest_tick: 100,
                required_tick: 100
            }
        );
        assert_eq!(
            result[2],
            Payload {
                bytes: vec![1, 8, 1],
                earliest_tick: 0,
                required_tick: 370
            }
        );
    }
}
