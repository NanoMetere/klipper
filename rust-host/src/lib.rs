// SPDX-License-Identifier: GPL-3.0-only
//! Klipper MCU transport framing. This crate does not open a device.
//!
//! Framing and CRC adapted from Klipper's klippy/msgproto.py.
//! Upstream copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>.

#[cfg(feature = "identify")]
pub mod identify;
#[cfg(feature = "identify")]
pub mod clock;
#[cfg(feature = "identify")]
pub mod mcu_config;
#[cfg(feature = "identify")]
pub mod trigger_sync;
pub mod stream;
pub mod read_query;
pub mod clock_sync;
pub mod planner;
pub mod flight;
pub mod lookahead;
pub mod gcode;
pub mod gcode_macros;
pub mod trajectory;
pub mod step_times;
pub mod step_compress;
pub mod step_clock;
pub mod config;
pub mod machine_preview;
pub mod thermal;
pub mod extrusion;
pub mod pressure_advance;
pub mod input_shaper;
mod input_shaper_defs;
pub mod bed_mesh;
pub mod probe;
pub mod homing;
pub mod can;
pub mod retry_timing;
pub mod timed_flight;
pub mod virtual_can;
#[cfg(feature = "identify")]
pub mod step_wire;
#[cfg(feature = "identify")]
pub mod virtual_job;
#[cfg(feature = "webhooks")]
pub mod webhooks;
#[cfg(feature = "identify")]
mod vlq;

pub const MIN_FRAME: usize = 5;
pub const MAX_FRAME: usize = 64;
pub const MAX_PAYLOAD: usize = MAX_FRAME - MIN_FRAME;
const DEST: u8 = 0x10;
const SEQ_MASK: u8 = 0x0f;
const SYNC: u8 = 0x7e;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    pub sequence: u8,
    pub payload: &'a [u8],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    PayloadTooLong,
    Length,
    Destination,
    Sync,
    Checksum,
}

// Reflected CRC-16/CCITT polynomial used by Klipper (initial state 0xffff).
const fn crc_tables<const N: usize>() -> [[u16; 256]; N] {
    let mut tables = [[0u16; 256]; N];
    let mut index = 0;
    while index < 256 {
        let mut v = index as u16;
        let mut bit = 0;
        while bit < 8 {
            v = (v >> 1) ^ if v & 1 != 0 { 0x8408 } else { 0 };
            bit += 1;
        }
        tables[0][index] = v;
        index += 1;
    }
    let mut n = 1;
    while n < N {
        let mut index = 0;
        while index < 256 {
            let v = tables[n - 1][index];
            tables[n][index] = (v >> 8) ^ tables[0][v as u8 as usize];
            index += 1;
        }
        n += 1;
    }
    tables
}

static CRC_TABLES: [[u16; 256]; 4] = crc_tables();

fn crc16(bytes: &[u8]) -> [u8; 2] {
    let mut crc = 0xffffu16;
    let blocks = bytes.chunks_exact(4);
    let remaining = blocks.remainder();
    for block in blocks {
        let [a, b, c, d] = *block else { unreachable!("four-byte chunk") };
        crc = CRC_TABLES[3][(a ^ crc as u8) as usize]
            ^ CRC_TABLES[2][(b ^ (crc >> 8) as u8) as usize]
            ^ CRC_TABLES[1][c as usize]
            ^ CRC_TABLES[0][d as usize];
    }
    for &byte in remaining {
        crc = (crc >> 8) ^ CRC_TABLES[0][(byte ^ crc as u8) as usize];
    }
    crc.to_be_bytes()
}

/// Encode one frame. The sequence wraps modulo 16, as in Klippy.
pub fn encode(sequence: u8, payload: &[u8]) -> Result<Vec<u8>, FrameError> {
    if payload.len() > MAX_PAYLOAD {
        return Err(FrameError::PayloadTooLong);
    }
    let length = MIN_FRAME + payload.len();
    let mut bytes = Vec::with_capacity(length);
    bytes.extend_from_slice(&[length as u8, DEST | (sequence & SEQ_MASK)]);
    bytes.extend_from_slice(payload);
    let checksum = crc16(&bytes);
    bytes.extend_from_slice(&checksum);
    bytes.push(SYNC);
    Ok(bytes)
}

/// Decode one frame at the start of a buffer, leaving any remaining bytes untouched.
/// A partial frame returns `Ok(None)`; an invalid frame returns an explicit error.
pub fn decode_prefix(input: &[u8]) -> Result<Option<(Frame<'_>, usize)>, FrameError> {
    if input.len() < MIN_FRAME {
        return Ok(None);
    }
    let length = usize::from(input[0]);
    if !(MIN_FRAME..=MAX_FRAME).contains(&length) {
        return Err(FrameError::Length);
    }
    if input[1] & !SEQ_MASK != DEST {
        return Err(FrameError::Destination);
    }
    if input.len() < length {
        return Ok(None);
    }
    if input[length - 1] != SYNC {
        return Err(FrameError::Sync);
    }
    let crc_offset = length - 3;
    if crc16(&input[..crc_offset]).as_slice() != &input[crc_offset..length - 1] {
        return Err(FrameError::Checksum);
    }
    Ok(Some((Frame {
        sequence: input[1] & SEQ_MASK,
        payload: &input[2..crc_offset],
    }, length)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checks_upstream_wire_vectors() {
        for (seq, payload, expected) in VECTORS {
            assert_eq!(encode(*seq, payload).unwrap().as_slice(), *expected);
            assert_eq!(decode_prefix(expected), Ok(Some((Frame {
                sequence: *seq & SEQ_MASK,
                payload,
            }, expected.len()))));
        }
    }

    #[test]
    fn crc_matches_scalar_for_every_wire_length() {
        fn scalar(bytes: &[u8]) -> [u8; 2] {
            let mut crc = 0xffffu16;
            for &byte in bytes {
                let x = byte ^ crc as u8;
                let x = u16::from(x ^ ((x & 0x0f) << 4));
                crc = ((x << 8) | (crc >> 8)) ^ (x >> 4) ^ (x << 3);
            }
            crc.to_be_bytes()
        }

        let mut prng = 0x4de8_c7a9u32;
        for len in 0..=MAX_PAYLOAD {
            for seq in 0..=SEQ_MASK {
                for _ in 0..8 {
                    let mut payload = vec![0; len];
                    for byte in &mut payload {
                        prng ^= prng << 13;
                        prng ^= prng >> 17;
                        prng ^= prng << 5;
                        *byte = prng as u8;
                    }
                    let frame = encode(seq, &payload).unwrap();
                    let offset = frame.len() - 3;
                    assert_eq!(&frame[offset..offset + 2], &scalar(&frame[..offset]));
                    let (decoded, size) = decode_prefix(&frame).unwrap().unwrap();
                    assert_eq!((decoded.sequence, decoded.payload, size),
                               (seq, payload.as_slice(), frame.len()));
                }
            }
        }
    }

    #[test]
    fn accepts_ack_and_maximum_payload() {
        for len in [0, MAX_PAYLOAD] {
            let data = vec![SYNC; len];
            let bytes = encode(255, &data).unwrap();
            assert_eq!(bytes.len(), MIN_FRAME + len);
            let (frame, consumed) = decode_prefix(&bytes).unwrap().unwrap();
            assert_eq!(frame.sequence, 15);
            assert_eq!(frame.payload, data.as_slice());
            assert_eq!(consumed, bytes.len());
        }
        assert_eq!(encode(0, &[0; MAX_PAYLOAD + 1]), Err(FrameError::PayloadTooLong));
    }

    #[test]
    fn handles_partial_and_concatenated_frames() {
        let first = encode(0, &[1, 2, SYNC]).unwrap();
        let next = encode(1, &[]).unwrap();
        for i in 0..first.len() {
            assert_eq!(decode_prefix(&first[..i]), Ok(None));
        }
        let joined = [first.as_slice(), next.as_slice()].concat();
        let (_, consumed) = decode_prefix(&joined).unwrap().unwrap();
        assert_eq!(consumed, first.len());
        assert_eq!(decode_prefix(&joined[consumed..]), decode_prefix(&next));
    }

    #[test]
    fn rejects_corrupt_frames() {
        let bytes = encode(1, &[1, 2]).unwrap();
        for (index, value, expected) in [
            (0, 4, FrameError::Length),
            (0, 65, FrameError::Length),
            (1, 0x21, FrameError::Destination),
            (2, 0x33, FrameError::Checksum),
            (bytes.len() - 1, 0xff, FrameError::Sync),
        ] {
            let mut broken = bytes.clone();
            broken[index] = value;
            assert_eq!(decode_prefix(&broken), Err(expected));
        }
    }

    include!("wire_vectors.rs");
}

#[cfg(feature = "python-bridge")]
mod python_bridge;
