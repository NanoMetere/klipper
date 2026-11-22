// SPDX-License-Identifier: GPL-3.0-only
//! Optional in-process Python bridge for the bounded look-ahead kernel.

use crate::lookahead::{plan_junctions, Junction, JunctionInput};
use std::{mem::size_of, panic::catch_unwind, slice};

/// Stable v1 kernel ABI. Each input row is five native-endian doubles
/// (max_start, max_cruise, delta, max_min_cruise, delta_min_cruise).
/// Each output row holds three doubles (start, cruise, end).
/// Returns the number of flushable rows, or -1 for fallback.
/// No caller array is modified if the bridge rejects the input.
///
/// # Safety
/// The caller must supply valid aligned pointers to `len * 5` readable and
/// `len * 3` writable doubles, respectively. The arrays must not overlap.
/// The caller must prevent concurrent mutation until this function returns.
#[no_mangle]
pub unsafe extern "C" fn klipper_rs_lookahead_plan_v1(
    input: *const f64,
    output: *mut f64,
    len: usize,
    lazy: u8,
) -> i32 {
    if len == 0 {
        return 0;
    }
    if len > 4096
        || lazy > 1
        || input.is_null()
        || output.is_null()
        || !input.is_aligned()
        || !output.is_aligned()
    {
        return -1;
    }
    let a = input as usize;
    let b = output as usize;
    let Some(a_end) = a.checked_add(len * size_of::<JunctionInput>()) else {
        return -1;
    };
    let Some(b_end) = b.checked_add(len * size_of::<Junction>()) else {
        return -1;
    };
    if a < b_end && b < a_end {
        return -1;
    }

    catch_unwind(|| {
        // Safety: the caller guarantees non-overlapping valid native arrays;
        // repr(C) structures here are exclusively contiguous f64 fields.
        let rows = unsafe { slice::from_raw_parts(input.cast::<JunctionInput>(), len) };
        for r in rows {
            if [
                r.max_start,
                r.max_cruise,
                r.dv2,
                r.max_min_cruise,
                r.dv2_min_cruise,
            ]
            .iter()
            .any(|x| !x.is_finite() || !(0.0..=1e30).contains(x))
                || r.max_cruise <= 0.0
            {
                return -1;
            }
        }
        let planned = plan_junctions(len, lazy != 0, |i| rows[i]);
        if planned
            .iter()
            .any(|j| !j.cruise.is_finite() || j.cruise <= 0.0)
        {
            return -1;
        }
        unsafe {
            std::ptr::copy_nonoverlapping(
                planned.as_ptr(),
                output.cast::<Junction>(),
                planned.len(),
            );
        }
        planned.len() as i32
    })
    .unwrap_or(-1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_nonfinite_and_aliasing_before_writing_output() {
        let bad = [0.0, f64::NAN, 10.0, 0.0, 2.0];
        let mut out = [1.0, 2.0, 3.0];
        assert_eq!(
            unsafe { klipper_rs_lookahead_plan_v1(bad.as_ptr(), out.as_mut_ptr(), 1, 0) },
            -1
        );
        assert_eq!(out, [1.0, 2.0, 3.0]);
        let mut shared = [0.0, 2.0, 10.0, 0.0, 2.0];
        let pointer = shared.as_mut_ptr();
        assert_eq!(
            unsafe { klipper_rs_lookahead_plan_v1(pointer, pointer, 1, 0) },
            -1
        );
    }
}
