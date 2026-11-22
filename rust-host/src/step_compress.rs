// SPDX-License-Identifier: GPL-3.0-only
//! Offline step clock compression adapted from klippy/chelper/stepcompress.c.
//! Upstream copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>.
//! Operates on one already-filtered same-direction run. No MCU I/O.

const MAX_COUNT: usize = 65_535;
const HORIZON: u64 = 3 << 28;
const MAX_ERROR: u32 = 200_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Empty,
    OutOfRange,
    Unrepresentable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Block {
    pub interval: u32,
    pub count: u16,
    pub add: i16,
    pub last_clock: u64,
}

fn ceil(n: i64, d: i64) -> i64 {
    if n >= 0 {
        (n + d - 1) / d
    } else {
        n / d
    }
}

fn floor(n: i64, d: i64) -> i64 {
    if n >= 0 {
        n / d
    } else {
        (n - d + 1) / d
    }
}

/// Compress a prefix of one direction's already-filtered request clocks. A
/// generated pulse is never late. Its early error is at most the configured
/// error and at most half the previous requested interval. Discontinuities,
/// motion history, clock era changes and wire command IDs belong to callers.
pub fn compress_prefix(base: u64, clocks: &[u64], max_error: u32) -> Result<Block, Error> {
    validate(base, clocks, max_error)?;
    compress_valid(base, clocks, max_error)
}

fn validate(base: u64, clocks: &[u64], max_error: u32) -> Result<(), Error> {
    if clocks.is_empty() {
        return Err(Error::Empty);
    }
    if clocks.len() > MAX_COUNT
        || max_error > MAX_ERROR
        || clocks[0] <= base
        || clocks.windows(2).any(|p| p[0] >= p[1])
        || clocks.last().unwrap() - base >= HORIZON
    {
        return Err(Error::OutOfRange);
    }
    Ok(())
}

/// Compress a bounded run, validating input once and checking every output.
pub fn compress_run(base: u64, clocks: &[u64], max_error: u32) -> Result<Vec<Block>, Error> {
    let mut blocks = Vec::new();
    compress_each(base, clocks, max_error, |block| blocks.push(block))?;
    Ok(blocks)
}

/// As `compress_run`, but hands each verified block to a consumer immediately
/// without building an intermediate Vec. If this returns an error, discard
/// the output of this entire run. Do not use it as a device write callback.
pub fn compress_each(
    mut base: u64,
    clocks: &[u64],
    max_error: u32,
    mut emit: impl FnMut(Block),
) -> Result<(), Error> {
    validate(base, clocks, max_error)?;
    let mut remaining = clocks;
    while !remaining.is_empty() {
        let block = compress_valid(base, remaining, max_error)?;
        remaining = &remaining[usize::from(block.count)..];
        base = block.last_clock;
        emit(block);
    }
    Ok(())
}

fn compress_valid(base: u64, clocks: &[u64], max_error: u32) -> Result<Block, Error> {
    let max_error = i64::from(max_error);
    let bounds = |i: usize| {
        let point = (clocks[i] - base) as i64;
        let prev = if i == 0 {
            0
        } else {
            (clocks[i - 1] - base) as i64
        };
        (point - ((point - prev) / 2).min(max_error), point)
    };
    let (mut outer_min, mut outer_max) = bounds(0);
    let (mut add, mut min_add, mut max_add) = (0, i64::from(i16::MIN), i64::from(i16::MAX));
    let (mut best_int, mut best_count, mut best_add, mut best_reach) = (0, 1, 1, i64::MIN);
    let (mut zero_int, mut zero_count) = (0, 0);
    let chosen = 'search: loop {
        let (mut lo, mut hi, mut interval) = (outer_min, outer_max, outer_max);
        let mut next_count = 2_i64;
        let (next_lo, next_hi, quad) = loop {
            if next_count as usize > clocks.len() {
                break 'search (interval, next_count - 1, add);
            }
            let (min_point, max_point) = bounds(next_count as usize - 1);
            let quad = next_count * (next_count - 1) / 2;
            let c = add * quad;
            // Most points already satisfy the current bounds. Avoid divisions
            // unless a candidate interval must actually be tightened.
            if lo * next_count < min_point - c {
                lo = ceil(min_point - c, next_count);
            }
            if hi * next_count > max_point - c {
                hi = floor(max_point - c, next_count);
            }
            if lo > hi {
                break (min_point, max_point, quad);
            }
            interval = hi;
            next_count += 1;
        };
        let count = next_count - 1;
        let reach = add * count * (count - 1) / 2 + interval * count;
        if reach > best_reach || (reach == best_reach && interval > best_int) {
            (best_int, best_count, best_add, best_reach) = (interval, count, add, reach);
            if add == 0 {
                (zero_int, zero_count) = (interval, count);
            }
            if count > 0x200 {
                break 'search preferred(best_int, best_count, best_add, zero_int, zero_count);
            }
        }
        if add * quad + interval * next_count < next_lo {
            min_add = add + 1;
            outer_max = hi;
        } else {
            max_add = add - 1;
            outer_min = lo;
        }
        if count > 1 {
            let delta = max_error * 11 / (count * count);
            min_add = min_add.max(add - delta);
            max_add = max_add.min(add + delta);
        }
        let required = next_lo - outer_max * next_count;
        if min_add * quad < required {
            min_add = ceil(required, quad);
        }
        let allowed = next_hi - outer_min * next_count;
        if max_add * quad > allowed {
            max_add = floor(allowed, quad);
        }
        if min_add > max_add {
            break 'search preferred(best_int, best_count, best_add, zero_int, zero_count);
        }
        add = max_add - (max_add - min_add) / 4;
    };
    let (interval, count, add) = chosen;
    if interval <= 0 || count <= 0 || add < i64::from(i16::MIN) || add > i64::from(i16::MAX) {
        return Err(Error::Unrepresentable);
    }
    let mut offset = 0;
    let mut spacing = interval;
    for i in 0..count as usize {
        if !(1..0x8000_0000).contains(&spacing) {
            return Err(Error::Unrepresentable);
        }
        offset += spacing;
        let (lo, hi) = bounds(i);
        if !(lo..=hi).contains(&offset) {
            return Err(Error::Unrepresentable);
        }
        spacing += add;
    }
    Ok(Block {
        interval: interval as u32,
        count: count as u16,
        add: add as i16,
        last_clock: base + offset as u64,
    })
}

fn preferred(
    best_int: i64,
    best_count: i64,
    best_add: i64,
    zero_int: i64,
    zero_count: i64,
) -> (i64, i64, i64) {
    if zero_count + zero_count / 16 >= best_count {
        (zero_int, zero_count, 0)
    } else {
        (best_int, best_count, best_add)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_exact_constant_and_ramped_runs_when_error_is_zero() {
        let base = (1 << 32) - 400;
        let constant: Vec<_> = (1..=1000).map(|i| base + i * 200).collect();
        assert_eq!(
            compress_prefix(base, &constant, 0).unwrap(),
            Block {
                interval: 200,
                count: 1000,
                add: 0,
                last_clock: base + 200_000
            }
        );
        let slope: Vec<_> = (1..=300).map(|i| base + i * 30 + i * (i - 1) / 2).collect();
        assert_eq!(
            compress_prefix(base, &slope, 0).unwrap(),
            Block {
                interval: 30,
                count: 300,
                add: 1,
                last_clock: *slope.last().unwrap()
            }
        );
    }

    #[test]
    fn rejects_nonmonotonic_clocks_and_horizon_crossings() {
        assert_eq!(compress_prefix(19, &[], 0), Err(Error::Empty));
        for clock in [&[20, 20][..], &[19], &[19 + HORIZON]] {
            assert_eq!(compress_prefix(19, clock, 0), Err(Error::OutOfRange));
        }
    }
}
