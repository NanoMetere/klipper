// SPDX-License-Identifier: GPL-3.0-only
//! Internal integer encoding for Klipper message fields.

pub(crate) fn encode(out: &mut Vec<u8>, value: i32) {
    emit(i64::from(value), |b| out.push(b));
}

pub(crate) fn encode_unsigned(out: &mut Vec<u8>, value: u32) {
    emit(i64::from(value), |b| out.push(b));
}

pub(crate) fn emit(value: i64, mut byte: impl FnMut(u8)) {
    for (low, high, shift) in [
        (-0x0400_0000, 0x0c00_0000, 28),
        (-0x8_0000, 0x18_0000, 21),
        (-0x1000, 0x3000, 14),
        (-0x20, 0x60, 7),
    ] {
        if value < low || value >= high {
            byte(((value >> shift) as u8 & 0x7f) | 0x80);
        }
    }
    byte(value as u8 & 0x7f);
}

// An unsigned 32-bit field can be encoded either as a positive value by
// Klippy or as a signed 32-bit bit pattern by the MCU.
pub(crate) fn read(input: &mut &[u8]) -> Option<i64> {
    let (&first, rest) = input.split_first()?;
    *input = rest;
    let mut value = i64::from(first & 0x7f);
    if first & 0x60 == 0x60 {
        value -= 0x80;
    }
    let mut byte = first;
    for _ in 1..5 {
        if byte & 0x80 == 0 {
            return Some(value);
        }
        let (&next, rest) = input.split_first()?;
        *input = rest;
        value = (value << 7) | i64::from(next & 0x7f);
        byte = next;
    }
    (byte & 0x80 == 0).then_some(value)
}
