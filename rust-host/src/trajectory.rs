// SPDX-License-Identifier: GPL-3.0-only
//! Offline XYZ trapezoid segments, adapted from klippy/chelper/trapq.c.
//! Upstream copyright (C) 2018-2021 Kevin O'Connor <kevin@koconnor.net>.
//! No step generation, position resets, history pruning or physical I/O.

use crate::lookahead::Profile;

const MAX_SEGMENTS: usize = 16_384;
const TIME_LIMIT: f64 = 1e12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Limit,
    Invalid,
    Overlap,
    Full,
}

#[derive(Debug, Clone, Copy)]
pub struct Segment {
    pub at: f64,
    pub duration: f64,
    pub position: [f64; 3],
    pub direction: [f64; 3],
    pub start_velocity: f64,
    pub half_acceleration: f64,
}

impl Segment {
    pub fn end(&self) -> f64 {
        self.at + self.duration
    }

    pub(crate) fn local(&self, offset: f64) -> [f64; 3] {
        let distance = (self.start_velocity + self.half_acceleration * offset) * offset;
        std::array::from_fn(|i| self.position[i] + self.direction[i] * distance)
    }

    pub fn sample(&self, time: f64) -> Result<[f64; 3], Error> {
        if !time.is_finite() || !self.duration.is_finite() || self.duration < 0.0 {
            return Err(Error::Invalid);
        }
        Ok(self.local((time - self.at).clamp(0.0, self.duration)))
    }
}

/// A bounded, monotonic timeline of XYZ segments including idle spans.
/// Callers must reserve long dwells and extrusion-only time separately.
pub struct Trajectory {
    segments: Vec<Segment>,
    capacity: usize,
}

#[derive(Clone, Copy)]
pub struct Trapezoid {
    pub at: f64,
    pub phase_sec: [f64; 3],
    pub start: [f64; 3],
    pub direction: [f64; 3],
    pub start_velocity: f64,
    pub cruise_velocity: f64,
    pub accel: f64,
}

impl Trajectory {
    pub fn new(capacity: usize) -> Result<Self, Error> {
        if !(1..=MAX_SEGMENTS).contains(&capacity) {
            return Err(Error::Limit);
        }
        Ok(Self {
            segments: Vec::new(),
            capacity,
        })
    }

    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    pub(crate) fn discard_ended_before(&mut self, cutoff: f64) -> usize {
        let first = self.segments.partition_point(|m| m.end() <= cutoff);
        self.segments.drain(..first);
        first
    }

    /// Append the kinematic portion of a verified look-ahead profile.
    /// Tiny negative cruise durations from zero-cruise rounding are normalized;
    /// genuinely negative or non-finite intervals are rejected.
    pub fn append_profile(&mut self, at: f64, p: &Profile) -> Result<(), Error> {
        if !p.kinematic {
            return Ok(());
        }
        let cruise = if (-1e-12..0.0).contains(&p.cruise_t) {
            0.0
        } else {
            p.cruise_t
        };
        self.append(Trapezoid {
            at,
            phase_sec: [p.accel_t, cruise, p.decel_t],
            start: p.start[..3].try_into().unwrap(),
            direction: p.direction[..3].try_into().unwrap(),
            start_velocity: p.start_v,
            cruise_velocity: p.cruise_v,
            accel: p.accel,
        })
    }

    /// Append up to three phases and an optional idle span atomically.
    pub fn append(&mut self, move_: Trapezoid) -> Result<(), Error> {
        self.append_phases(move_, false)
    }

    pub(crate) fn append_signed(&mut self, move_: Trapezoid) -> Result<(), Error> {
        self.append_phases(move_, true)
    }

    fn append_phases(&mut self, move_: Trapezoid, signed: bool) -> Result<(), Error> {
        let Trapezoid {
            mut at,
            phase_sec,
            start: mut position,
            direction,
            start_velocity,
            cruise_velocity,
            accel,
        } = move_;
        if !(0.0..TIME_LIMIT).contains(&at)
            || phase_sec.iter().any(|t| !t.is_finite() || *t < 0.0)
            || [start_velocity, cruise_velocity, accel]
                .iter()
                .any(|x| !x.is_finite() || (!signed && *x < 0.0))
            || position.iter().chain(&direction).any(|x| !x.is_finite())
        {
            return Err(Error::Invalid);
        }
        let mut next = [None; 4];
        let mut n = 0;
        if !phase_sec.iter().any(|d| *d > 0.0) {
            return Ok(());
        }
        let previous_end = self.segments.last().map_or(-1.0, Segment::end);
        if previous_end > at {
            return Err(Error::Overlap);
        }
        if previous_end < at {
            let previous_at = self.segments.last().map_or(-1.0, |s| s.at);
            let idle_start = if previous_at <= 0.0 && at > 1.0 {
                at - 1.0
            } else {
                previous_end
            };
            next[n] = Some(Segment {
                at: idle_start,
                duration: at - idle_start,
                position,
                direction: [0.0; 3],
                start_velocity: 0.0,
                half_acceleration: 0.0,
            });
            n += 1;
        }
        for (duration, start_velocity, half_acceleration) in [
            (phase_sec[0], start_velocity, 0.5 * accel),
            (phase_sec[1], cruise_velocity, 0.0),
            (phase_sec[2], cruise_velocity, -0.5 * accel),
        ] {
            if duration == 0.0 {
                continue;
            }
            let seg = Segment {
                at,
                duration,
                position,
                direction,
                start_velocity,
                half_acceleration,
            };
            let end = seg.end();
            if end < at || end > TIME_LIMIT {
                return Err(Error::Invalid);
            }
            position = seg.local(duration);
            if position.iter().any(|x| !x.is_finite()) {
                return Err(Error::Invalid);
            }
            // Like trapq, retain phases that advance locally but round to
            // the same absolute time. The step solver clips to this span.
            next[n] = Some(seg);
            n += 1;
            at = end;
        }
        if self.segments.len() + n > self.capacity {
            return Err(Error::Full);
        }
        self.segments.extend(next.into_iter().flatten());
        Ok(())
    }

    /// Samples the last segment starting no later than the query. Empty
    /// timelines return None; endpoints clamp to the nearest segment.
    pub fn position(&self, at: f64) -> Result<Option<[f64; 3]>, Error> {
        if !at.is_finite() {
            return Err(Error::Invalid);
        }
        let after = self.segments.partition_point(|s| s.at <= at);
        self.segments
            .get(after.saturating_sub(1))
            .map(|s| s.sample(at))
            .transpose()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(at: f64) -> Trapezoid {
        Trapezoid {
            at,
            phase_sec: [0.25, 0.5, 0.25],
            start: [0.; 3],
            direction: [1., 0., 0.],
            start_velocity: 0.,
            cruise_velocity: 2.,
            accel: 8.,
        }
    }

    #[test]
    fn samples_accel_cruise_decel_and_holds() {
        let mut q = Trajectory::new(4).unwrap();
        q.append(sample(11.0)).unwrap();
        assert_eq!(q.segments().len(), 4);
        assert_eq!(q.segments()[0].at, 10.0);
        assert_eq!(q.position(11.125).unwrap(), Some([0.0625, 0., 0.]));
        assert_eq!(q.position(11.5).unwrap(), Some([0.75, 0., 0.]));
        assert_eq!(q.position(30.0).unwrap(), Some([1.5, 0., 0.]));
        assert_eq!(q.append(sample(11.5)), Err(Error::Overlap));
        assert_eq!(q.append(sample(20.0)), Err(Error::Full));
        assert_eq!(q.segments().len(), 4);
    }

    #[test]
    fn builds_a_timeline_from_offline_gcode_profiles() {
        use crate::{gcode::Preview, lookahead::Limits};
        let mut preview =
            Preview::new(Limits::new(100.0, 1200.0, 5.0, 0.5).unwrap(), 64, None).unwrap();
        let mut q = Trajectory::new(32).unwrap();
        preview.line("G1 X10 F1200").unwrap();
        preview.line("G1 X10 Y10").unwrap();
        let mut at = 2.0;
        for m in preview.finish().unwrap() {
            q.append_profile(at, &m).unwrap();
            at = q.segments().last().unwrap().end();
        }
        let last = q.position(at + 1.0).unwrap().unwrap();
        assert!((last[0] - 10.0).abs() < 1e-12 && (last[1] - 10.0).abs() < 1e-12);
    }

    #[test]
    fn keeps_sub_ulp_phases_at_their_absolute_timestamp() {
        let mut q = Trajectory::new(4).unwrap();
        let mut s = sample(8.);
        s.phase_sec[1] = 2.960594732333751e-17;
        q.append(s).unwrap();
        let phases = q.segments();
        assert_eq!(phases[2].at, phases[2].end());
        assert_eq!(phases[2].at, phases[3].at);
        assert_eq!(q.position(8.5).unwrap(), Some([0.5, 0., 0.]));
    }

    #[test]
    fn rejects_invalid_inputs_without_consuming_capacity() {
        let mut q = Trajectory::new(4).unwrap();
        assert_eq!(q.append(sample(f64::NAN)), Err(Error::Invalid));
        let mut s = sample(11.);
        s.phase_sec[1] = -1e-30;
        assert_eq!(q.append(s), Err(Error::Invalid));
        assert!(q.segments().is_empty());
        assert_eq!(q.position(1.0), Ok(None));
        q.append(sample(4.0)).unwrap();
    }
}
