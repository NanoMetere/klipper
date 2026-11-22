// SPDX-License-Identifier: GPL-3.0-only
//! Offline encoding of checked step trace commands with a firmware dictionary.
//! Encoded payloads do not configure an MCU or drive a device.

use crate::{identify::Dictionary, step_clock::Command, vlq};
use std::{fmt, ops::Deref};

pub const SET_DIR: &str = "set_next_step_dir oid=%c dir=%c";
pub const QUEUE: &str = "queue_step oid=%c interval=%u count=%hu add=%hi";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    MissingFormat(&'static str),
    BadFormatId,
    InvalidTrace,
    InvalidPayload,
    UnknownMessage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Message {
    NextDirection {
        oid: u8,
        positive: bool,
    },
    Block {
        oid: u8,
        interval: u32,
        count: u16,
        add: i16,
    },
}

pub struct StepMessages {
    set_dir: i32,
    queue: i32,
}

// One signed message ID (5 bytes), u8 OID (2), interval (5),
// u16 count (3) and i16 add (3). Direction messages are smaller.
const PACKED_MAX: usize = 18;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct PayloadBytes {
    data: [u8; PACKED_MAX],
    len: u8,
}

impl PayloadBytes {
    const fn empty() -> Self {
        Self {
            data: [0; PACKED_MAX],
            len: 0,
        }
    }

    fn append(&mut self, value: i32) {
        vlq::emit(i64::from(value), |b| {
            self.data[usize::from(self.len)] = b;
            self.len += 1;
        });
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data[..usize::from(self.len)]
    }
}

impl Deref for PayloadBytes {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl AsRef<[u8]> for PayloadBytes {
    fn as_ref(&self) -> &[u8] {
        self.as_slice()
    }
}

impl fmt::Debug for PayloadBytes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.as_slice().fmt(f)
    }
}

impl IntoIterator for PayloadBytes {
    type Item = u8;
    type IntoIter = std::iter::Take<std::array::IntoIter<u8, PACKED_MAX>>;

    fn into_iter(self) -> Self::IntoIter {
        self.data.into_iter().take(usize::from(self.len))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Payload {
    pub bytes: PayloadBytes,
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

    /// Decode one complete, unframed stepper command. It has no physical effect.
    pub fn decode(&self, mut payload: &[u8]) -> Result<Message, Error> {
        let mut read = || vlq::read(&mut payload).ok_or(Error::InvalidPayload);
        let id = read()?;
        if id != i64::from(self.set_dir) && id != i64::from(self.queue) {
            return Err(Error::UnknownMessage);
        }
        let oid = u8::try_from(read()?).map_err(|_| Error::InvalidPayload)?;
        let message = if id == i64::from(self.set_dir) {
            let positive = match read()? {
                0 => false,
                1 => true,
                _ => return Err(Error::InvalidPayload),
            };
            Message::NextDirection { oid, positive }
        } else {
            let interval = read()?;
            if !(1..=u32::MAX as i64).contains(&interval) {
                return Err(Error::InvalidPayload);
            }
            let count = u16::try_from(read()?).map_err(|_| Error::InvalidPayload)?;
            let add = i16::try_from(read()?).map_err(|_| Error::InvalidPayload)?;
            Message::Block {
                oid,
                interval: interval as u32,
                count,
                add,
            }
        };
        if !payload.is_empty() {
            return Err(Error::InvalidPayload);
        }
        Ok(message)
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
            let mut bytes = PayloadBytes::empty();
            let mut earliest = 0;
            let required = last;
            match *cmd {
                Command::SetDirection(positive) => {
                    if direction == Some(positive) {
                        return Err(Error::InvalidTrace);
                    }
                    direction = Some(positive);
                    for v in [self.set_dir, i32::from(oid), i32::from(positive)] {
                        bytes.append(v);
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
                        bytes.append(v);
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
    fn inline_payload_roundtrips_signed_field_extremes() {
        let codec = StepMessages {
            set_dir: i32::MAX,
            queue: i32::MIN,
        };
        let width = 0x1000_0000;
        let records = codec
            .payloads(
                u8::MAX,
                0,
                &[
                    Command::SetDirection(true),
                    Command::Steps(Block {
                        interval: width,
                        count: 1,
                        add: i16::MIN,
                        last_clock: u64::from(width),
                    }),
                ],
            )
            .unwrap();
        assert_eq!(
            codec.decode(records[0].bytes.as_ref()),
            Ok(Message::NextDirection {
                oid: u8::MAX,
                positive: true
            })
        );
        assert_eq!(
            codec.decode(&records[1].bytes),
            Ok(Message::Block {
                oid: u8::MAX,
                interval: width,
                count: 1,
                add: i16::MIN
            })
        );
        let field = records[1].bytes;
        assert_eq!(
            field.iter().copied().collect::<Vec<_>>(),
            field.into_iter().collect::<Vec<_>>()
        );
        assert!(field.len() <= PACKED_MAX);
    }

    fn test_bytes(raw: &[u8]) -> PayloadBytes {
        let mut bytes = PayloadBytes::empty();
        bytes.data[..raw.len()].copy_from_slice(raw);
        bytes.len = raw.len() as u8;
        bytes
    }

    #[test]
    fn decodes_exactly_one_opaque_dictionary_command() {
        let m = StepMessages {
            set_dir: 1,
            queue: 2,
        };
        assert_eq!(
            m.decode(&[1, 5, 1]),
            Ok(Message::NextDirection {
                oid: 5,
                positive: true
            })
        );
        assert_eq!(m.decode(&[1, 5, 2]), Err(Error::InvalidPayload));
        assert_eq!(m.decode(&[1, 5, 1, 0]), Err(Error::InvalidPayload));
        assert_eq!(m.decode(&[1, 0x80]), Err(Error::InvalidPayload));
        assert_eq!(m.decode(&[22]), Err(Error::UnknownMessage));
        // A negative add encoded by the stock signed VLQ representation.
        assert_eq!(
            m.decode(&[2, 5, 10, 3, 0x7e]),
            Ok(Message::Block {
                oid: 5,
                interval: 10,
                count: 3,
                add: -2
            })
        );
    }

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
                bytes: test_bytes(&[1, 8, 0]),
                earliest_tick: 0,
                required_tick: 100
            }
        );
        assert_eq!(
            result[1],
            Payload {
                bytes: test_bytes(&[2, 8, 0x80, 100, 3, 118]),
                earliest_tick: 100,
                required_tick: 100
            }
        );
        assert_eq!(
            result[2],
            Payload {
                bytes: test_bytes(&[1, 8, 1]),
                earliest_tick: 0,
                required_tick: 370
            }
        );
    }
}
