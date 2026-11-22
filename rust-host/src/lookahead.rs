// SPDX-License-Identifier: GPL-3.0-only
//! Offline look-ahead adapted from Klipper's klippy/toolhead.py (Move and LookAheadQueue).
//! Upstream copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>.
//! Produces motion profiles, not step pulses, heater commands, or kinematic checks.

const FLUSH_TIME: f64 = 0.150;
const MAX_QUEUED: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidLimits,
    InvalidMove,
    InvalidProfile,
    Full,
}

#[derive(Clone, Copy)]
pub struct Limits {
    velocity: f64,
    accel: f64,
    deviation: f64,
    pseudo_accel: f64,
}

impl Limits {
    pub fn new(
        velocity: f64,
        accel: f64,
        square_corner: f64,
        cruise_ratio: f64,
    ) -> Result<Self, Error> {
        let deviation = square_corner.powi(2) * (2.0_f64.sqrt() - 1.0) / accel;
        if !positive(velocity)
            || !velocity.powi(2).is_finite()
            || !positive(accel)
            || !square_corner.is_finite()
            || square_corner < 0.0
            || !deviation.is_finite()
            || !(0.0..1.0).contains(&cruise_ratio)
        {
            return Err(Error::InvalidLimits);
        }
        Ok(Self {
            velocity,
            accel,
            deviation,
            pseudo_accel: accel * (1.0 - cruise_ratio),
        })
    }
}

fn positive(v: f64) -> bool {
    v > 0.0 && v.is_finite()
}

// Python 3.12's floating sum preserves the low terms when computing
// sqrt(dx² + dy² + dz²). That rounding can change junctions near reversal.
fn squared_xyz(delta: [f64; 4]) -> f64 {
    let (mut high, mut low) = (0.0_f64, 0.0);
    for v in &delta[..3] {
        let x = v * v;
        let next = high + x;
        low += if high.abs() >= x.abs() {
            (high - next) + x
        } else {
            (x - next) + high
        };
        high = next;
    }
    if low.is_finite() {
        high + low
    } else {
        high
    }
}

pub struct Move {
    start: [f64; 4],
    end: [f64; 4],
    direction: [f64; 4],
    distance: f64,
    kinematic: bool,
    accel: f64,
    deviation: f64,
    min_time: f64,
    max_cruise: f64,
    dv2: f64,
    next_junction: f64,
    max_start: f64,
    max_min_cruise: f64,
    dv2_min_cruise: f64,
}

impl Move {
    /// XYZ and one optional extrusion axis (E). This does not authorize E;
    /// machine-specific position, temperature and extrusion limits are separate.
    pub fn new(
        limits: Limits,
        start: [f64; 4],
        mut end: [f64; 4],
        speed: f64,
    ) -> Result<Self, Error> {
        let mut delta: [f64; 4] = std::array::from_fn(|i| end[i] - start[i]);
        if !positive(speed) || delta.iter().any(|v| !v.is_finite()) {
            return Err(Error::InvalidMove);
        }
        let mut distance = squared_xyz(delta).sqrt();
        let kinematic = distance >= 1e-9;
        let mut accel = limits.accel;
        let mut velocity = speed.min(limits.velocity);
        if !kinematic {
            end[..3].copy_from_slice(&start[..3]);
            delta[..3].fill(0.0);
            distance = delta[3].abs();
            accel = 99_999_999.9;
            velocity = speed;
        }
        let direction = delta.map(|v| v * (1.0 / distance));
        let max_cruise = velocity.powi(2);
        let dv2 = 2.0 * distance * accel;
        let dv2_min_cruise = 2.0 * distance * limits.pseudo_accel;
        let min_time = distance / velocity;
        if ![distance, max_cruise, dv2, dv2_min_cruise, min_time]
            .into_iter()
            .all(positive)
            || direction.iter().any(|v| !v.is_finite())
            || dv2 + max_cruise == f64::INFINITY
        {
            return Err(Error::InvalidMove);
        }
        Ok(Self {
            start,
            end,
            direction,
            distance,
            kinematic,
            accel,
            deviation: limits.deviation,
            min_time,
            max_cruise,
            dv2,
            next_junction: 999_999_999.9,
            max_start: 0.0,
            max_min_cruise: 0.0,
            dv2_min_cruise,
        })
    }

    /// The position after stock normalization of sub-nanometer XYZ changes.
    pub fn end_position(&self) -> [f64; 4] {
        self.end
    }

    /// Manually apply a stricter limit (e.g. after checking Z axis or E travel).
    pub fn limit(&mut self, speed: f64, accel: f64) -> Result<(), Error> {
        if !positive(speed)
            || !positive(accel)
            || !positive(speed.powi(2))
            || !positive(self.distance / speed)
            || !positive(2.0 * self.distance * self.accel.min(accel))
        {
            return Err(Error::InvalidLimits);
        }
        if speed.powi(2) < self.max_cruise {
            self.max_cruise = speed.powi(2);
            self.min_time = self.distance / speed;
        }
        self.accel = self.accel.min(accel);
        self.dv2 = 2.0 * self.distance * self.accel;
        self.dv2_min_cruise = self.dv2_min_cruise.min(self.dv2);
        Ok(())
    }

    pub fn limit_next_junction(&mut self, speed: f64) -> Result<(), Error> {
        if !speed.is_finite() || speed < 0.0 || !speed.powi(2).is_finite() {
            return Err(Error::InvalidLimits);
        }
        self.next_junction = self.next_junction.min(speed.powi(2));
        Ok(())
    }

    fn junction(&mut self, prev: &Move, extruder_corner: Option<f64>) {
        if !self.kinematic || !prev.kinematic {
            return;
        }
        let mut max = self
            .max_cruise
            .min(prev.max_cruise)
            .min(prev.next_junction)
            .min(prev.max_start + prev.dv2);
        if let Some(velocity) = extruder_corner {
            let change = (self.direction[3] - prev.direction[3]).abs();
            if change != 0.0 {
                max = max.min((velocity / change).powi(2));
            }
        }
        let cos = -(self.direction[0] * prev.direction[0]
            + self.direction[1] * prev.direction[1]
            + self.direction[2] * prev.direction[2]);
        let sin_half = (0.5 * (1.0 - cos)).max(0.0).sqrt();
        let cos_half = (0.5 * (1.0 + cos)).max(0.0).sqrt();
        let one_minus = 1.0 - sin_half;
        if one_minus > 0.0 && cos_half > 0.0 {
            let radius = sin_half / one_minus;
            let quarter_tan = 0.25 * sin_half / cos_half;
            max = max
                .min(radius * self.deviation * self.accel)
                .min(radius * prev.deviation * prev.accel)
                .min(self.dv2 * quarter_tan)
                .min(prev.dv2 * quarter_tan);
        }
        self.max_start = max;
        self.max_min_cruise = max.min(prev.max_min_cruise + prev.dv2_min_cruise);
    }

    fn profile(&self, start: f64, cruise: f64, end: f64) -> Result<Profile, Error> {
        let half_inv = 0.5 / self.accel;
        let accel_d = (cruise - start) * half_inv;
        let decel_d = (cruise - end) * half_inv;
        let cruise_d = self.distance - accel_d - decel_d;
        let start = start.sqrt();
        let cruise = cruise.sqrt();
        let end = end.sqrt();
        let accel_t = accel_d / ((start + cruise) * 0.5);
        let cruise_t = cruise_d / cruise;
        let decel_t = decel_d / ((end + cruise) * 0.5);
        if ![start, cruise, end, accel_t, cruise_t, decel_t]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(Error::InvalidProfile);
        }
        Ok(Profile {
            start: self.start,
            end: self.end,
            direction: self.direction,
            kinematic: self.kinematic,
            accel: self.accel,
            start_v: start,
            cruise_v: cruise,
            end_v: end,
            accel_t,
            cruise_t,
            decel_t,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Profile {
    pub start: [f64; 4],
    pub end: [f64; 4],
    pub direction: [f64; 4],
    pub kinematic: bool,
    pub accel: f64,
    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
    pub accel_t: f64,
    pub cruise_t: f64,
    pub decel_t: f64,
}

/// A bounded offline queue. Lazy flushing keeps a tail when subsequent moves
/// may still affect its velocity; a full flush assumes a complete stop.
pub struct LookAhead {
    moves: Vec<Move>,
    capacity: usize,
    budget: f64,
    extruder_corner: Option<f64>,
}

impl LookAhead {
    pub fn new(capacity: usize, extruder_corner: Option<f64>) -> Result<Self, Error> {
        if !(1..=MAX_QUEUED).contains(&capacity)
            || extruder_corner.is_some_and(|v| v < 0.0 || !v.is_finite())
        {
            return Err(Error::InvalidLimits);
        }
        Ok(Self {
            moves: Vec::with_capacity(capacity),
            capacity,
            budget: FLUSH_TIME,
            extruder_corner,
        })
    }

    pub fn queued(&self) -> usize {
        self.moves.len()
    }

    /// Returns whether the stock look-ahead time budget requests a lazy flush.
    pub fn push(&mut self, mut next: Move) -> Result<bool, Error> {
        if self.moves.len() == self.capacity {
            return Err(Error::Full);
        }
        if let Some(prev) = self.moves.last() {
            if prev.end != next.start {
                return Err(Error::InvalidMove);
            }
            next.junction(prev, self.extruder_corner);
            self.budget -= next.min_time;
            self.moves.push(next);
            return Ok(self.budget <= 0.0);
        }
        self.moves.push(next);
        Ok(false)
    }

    pub fn flush(&mut self, lazy: bool) -> Result<Vec<Profile>, Error> {
        self.budget = FLUSH_TIME;
        let mut update_count = lazy;
        let mut count = self.moves.len();
        let mut info = vec![(0.0, None, 0.0); count];
        let (mut next_start, mut next_mcr_start, mut peak) = (0.0, 0.0, 0.0);
        let mut pending = 0;
        for (i, m) in self.moves.iter().enumerate().rev() {
            let reachable = next_start + m.dv2;
            let start = m.max_start.min(reachable);
            let reachable_mcr = next_mcr_start + m.dv2_min_cruise;
            let mcr = m.max_min_cruise.min(reachable_mcr);
            let mut cruise = None;
            pending += 1;
            if mcr < reachable_mcr {
                if mcr + m.dv2_min_cruise > next_mcr_start || pending > 1 {
                    if update_count && peak != 0.0 {
                        count = i + pending;
                        update_count = false;
                    }
                    peak = (mcr + reachable_mcr) * 0.5;
                }
                cruise = Some(((start + reachable) * 0.5).min(m.max_cruise).min(peak));
                pending = 0;
            }
            info[i] = (start, cruise, next_start);
            next_start = start;
            next_mcr_start = mcr;
        }
        if update_count || count == 0 {
            return Ok(Vec::new());
        }
        let mut output = Vec::with_capacity(count);
        let mut prev_cruise: f64 = 0.0;
        for (m, (start, cruise, next)) in self.moves.iter().zip(info).take(count) {
            let cruise = cruise.unwrap_or_else(|| prev_cruise.min(start));
            output.push(m.profile(start.min(cruise), cruise, next.min(cruise))?);
            prev_cruise = cruise;
        }
        self.moves.drain(..count);
        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fits_a_short_reversal_and_finishes_stopped() {
        let limits = Limits::new(200.0, 2500.0, 5.0, 0.5).unwrap();
        let a = [0.0; 4];
        let b = [1.0, 0.0, 0.0, 0.0];
        let mut look = LookAhead::new(8, None).unwrap();
        look.push(Move::new(limits, a, b, 100.0).unwrap()).unwrap();
        look.push(Move::new(limits, b, a, 100.0).unwrap()).unwrap();
        let profiles = look.flush(false).unwrap();
        assert_eq!(profiles.len(), 2);
        assert!(profiles
            .iter()
            .all(|m| m.start_v == 0.0 && m.cruise_v > 0.0 && m.end_v == 0.0));
        assert_eq!(look.queued(), 0);
    }

    #[test]
    fn rejects_zero_nonfinite_and_overflow_and_bounds_queue() {
        assert!(Limits::new(1.0, 1.0, 5.0, 1.0).is_err());
        let limits = Limits::new(200.0, 2500.0, 5.0, 0.5).unwrap();
        let mut look = LookAhead::new(1, None).unwrap();
        let begin = [0.0; 4];
        let end = [1.0; 4];
        for invalid in [begin, [f64::NAN; 4], [f64::MAX; 4]] {
            assert!(Move::new(limits, begin, invalid, 20.0).is_err());
        }
        look.push(Move::new(limits, begin, end, 20.0).unwrap())
            .unwrap();
        assert!(matches!(
            look.push(Move::new(limits, end, begin, 20.0).unwrap()),
            Err(Error::Full)
        ));
        assert!(look.flush(true).unwrap().is_empty());
        assert_eq!(look.queued(), 1);
    }
}
