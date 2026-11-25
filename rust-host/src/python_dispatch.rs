// SPDX-License-Identifier: GPL-3.0-only
//! Optional scanner for the pinned Klippy debug-only adapter.

/// Scan UTF-8 input composed of newline-separated command rows.
/// A record is one code byte (0 = stock fallback, 1 = G0, 2 = G1)
/// followed by up to five (field ID, inclusive start, exclusive end)
/// triples. A zero field terminates an incomplete record. No G-code
/// values are interpreted; the selected stock handler owns validation.
///
/// # Safety
/// The caller supplies len readable bytes and out_len writable bytes.
/// Buffers must not overlap and may not be changed during this call.
#[no_mangle]
pub unsafe extern "C" fn klipper_rs_move_scan_v1(
    src: *const u8,
    len: usize,
    out: *mut u8,
    out_len: usize,
) -> i32 {
    let rows = out_len / 16;
    if len > 1_048_576
        || rows > 4096
        || src.is_null()
        || out.is_null()
        || rows == 0
        || !out_len.is_multiple_of(16)
    {
        return -1;
    }
    let src = unsafe { core::slice::from_raw_parts(src, len) };
    let out = unsafe { core::slice::from_raw_parts_mut(out, out_len) };
    let mut count = 0;
    for line in src.split(|&c| c == b'\n') {
        if count == rows {
            return -1;
        }
        let rec = &mut out[count * 16..count * 16 + 16];
        fast(line, rec);
        count += 1;
    }
    if count == rows {
        0
    } else {
        -1
    }
}

fn fast(line: &[u8], dst: &mut [u8]) {
    dst[0] = 0;
    if !(4..128).contains(&line.len()) || line[0] != b'G' || line[2] != b' ' {
        return;
    }
    let code = match line[1] {
        b'0' => 1,
        b'1' => 2,
        _ => return,
    };
    let mut p = 2;
    let mut n = 1;
    let mut seen = 0u8;
    while p < line.len() {
        if line[p] == b';' {
            break;
        }
        if line[p] != b' ' {
            return;
        }
        while p < line.len() && line[p] == b' ' {
            p += 1;
        }
        if p == line.len() || line[p] == b';' {
            break;
        }
        let axis = match line[p] {
            b'X' => 1,
            b'Y' => 2,
            b'Z' => 3,
            b'E' => 4,
            b'F' => 5,
            _ => return,
        };
        let bit = 1 << axis;
        if seen & bit != 0 {
            return;
        }
        seen |= bit;
        p += 1;
        let start = p;
        while p < line.len() && (line[p].is_ascii_digit() || matches!(line[p], b'+' | b'-' | b'.'))
        {
            p += 1;
        }
        if p == start {
            return;
        }
        dst[n..n + 3].copy_from_slice(&[axis, start as u8, p as u8]);
        n += 3;
    }
    if seen == 0 {
        return;
    }
    if n < dst.len() {
        dst[n] = 0;
    }
    dst[0] = code;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_scanner_preserves_order_and_rejects_uncertain_rows() {
        let line = b"G1 E-.25 X3 F1200";
        let mut row = [0; 16];
        fast(line, &mut row);
        assert_eq!(row[0], 2);
        for (index, axis, expected) in [
            (1, 4, &b"-.25"[..]),
            (4, 1, &b"3"[..]),
            (7, 5, &b"1200"[..]),
        ] {
            assert_eq!(row[index], axis);
            assert_eq!(
                &line[usize::from(row[index + 1])..usize::from(row[index + 2])],
                expected
            );
        }
        assert_eq!(row[10], 0);
        for line in [
            &b" G1 X2"[..],
            b"N1 G1 X2*3",
            b"G1 X2 X3",
            b"G1 X2E-4",
            b"g1 X2",
            b"G1 X2\tY3",
            b"G28",
        ] {
            fast(line, &mut row);
            assert_eq!(row[0], 0, "{line:?}");
        }
        fast("G1 X2 ; 주석".as_bytes(), &mut row);
        assert_eq!(&row[..5], &[2, 1, 4, 5, 0]);
        fast("G1 X2;주석".as_bytes(), &mut row);
        assert_eq!(&row[..5], &[2, 1, 4, 5, 0]);
        fast("G1 X2é ;".as_bytes(), &mut row);
        assert_eq!(row[0], 0);
    }

    #[test]
    fn batch_requires_matching_record_count_and_size() {
        let input = b"G1 X2\nM400";
        let mut rows = [0; 32];
        assert_eq!(
            unsafe {
                klipper_rs_move_scan_v1(input.as_ptr(), input.len(), rows.as_mut_ptr(), rows.len())
            },
            0
        );
        assert_eq!([rows[0], rows[16]], [2, 0]);
        for size in [0, 16, 31] {
            assert_eq!(
                unsafe {
                    klipper_rs_move_scan_v1(input.as_ptr(), input.len(), rows.as_mut_ptr(), size)
                },
                -1
            );
        }
    }
}
