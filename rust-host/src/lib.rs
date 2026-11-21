// SPDX-License-Identifier: GPL-3.0-only
//! Klipper MCU transport framing. This crate does not open a device.
//!
//! Framing and CRC adapted from Klipper's klippy/msgproto.py.
//! Upstream copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>.

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

fn crc16(bytes: &[u8]) -> [u8; 2] {
    let mut crc = 0xffffu16;
    for &byte in bytes {
        let x = byte ^ crc as u8;
        let x = u16::from(x ^ ((x & 0x0f) << 4));
        crc = (x << 8 | crc >> 8) ^ (x >> 4) ^ (x << 3);
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
