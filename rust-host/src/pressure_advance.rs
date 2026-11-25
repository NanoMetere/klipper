// SPDX-License-Identifier: GPL-3.0-only
//! Offline extruder pressure-advance sampling adapted from klippy/chelper/kin_extruder.c.
//! Upstream copyright (C) 2018-2019 Kevin O'Connor <kevin@koconnor.net>.
//! The result is a geometric sample only, not an actuator command.

use crate::{
    step_times::{self, Axis, Event, Solver},
    trajectory::{Segment, Trajectory},
};
use std::collections::VecDeque;

const MAX_CHANGES: usize = 128;
const MAX_PHASES_PER_SAMPLE: usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Settings,
    Time,
    TooManyPhases,
}

pub struct Advance {
    half: f64,
    inverse_half_squared: f64,
    changes: Vec<(f64, f64)>,
}

/// Offline counterpart to a stock look-ahead timing callback. Sequence
/// points count all accepted motions, including extrusion-only moves.
pub(crate) struct GainCallbacks {
    smooth_time: f64,
    pending: VecDeque<(usize, f64)>,
    changes: Vec<(f64, f64)>,
}

impl GainCallbacks {
    pub(crate) fn new(base: &Advance, at: f64) -> Result<Self, Error> {
        let &[(start, gain)] = base.changes.as_slice() else {
            return Err(Error::Settings);
        };
        if base.half <= 0.0 || gain <= 0.0 || start > at {
            return Err(Error::Settings);
        }
        Ok(Self {
            smooth_time: base.half * 2.0,
            pending: VecDeque::new(),
            changes: vec![(start, gain)],
        })
    }

    pub(crate) fn defer(&mut self, target: usize, gain: f64) -> Result<(), Error> {
        if let Some((last, older)) = self.pending.back_mut() {
            if *last == target {
                *older = gain;
                return Ok(());
            }
        }
        if self.pending.len() + self.changes.len() >= MAX_CHANGES {
            return Err(Error::Settings);
        }
        self.pending.push_back((target, gain));
        Ok(())
    }

    pub(crate) fn resolve(&mut self, passed: usize, at: f64) -> Result<(), Error> {
        if let Some(&(target, _)) = self.pending.front() {
            if target < passed {
                return Err(Error::Time);
            }
            if target == passed {
                let (_, gain) = self.pending.pop_front().unwrap();
                let previous = self.changes.last_mut().unwrap();
                if at == previous.0 {
                    previous.1 = gain;
                } else if gain != previous.1 {
                    self.changes.push((at, gain));
                }
            }
        }
        Ok(())
    }

    pub(crate) fn has_pending(&self) -> bool {
        !self.pending.is_empty()
    }

    pub(crate) fn finish(self) -> Result<Advance, Error> {
        if !self.pending.is_empty() {
            return Err(Error::Time);
        }
        Advance::new(self.smooth_time, &self.changes)
    }
}

#[derive(Clone, Copy)]
struct Integrals {
    base_shift: f64,
    half_velocity: f64,
    third_velocity: f64,
    third_accel: f64,
    quarter_accel: f64,
}

impl Integrals {
    fn new(m: &Segment, gain: f64) -> Self {
        let vel = m.start_velocity + gain * 2.0 * m.half_acceleration;
        Self {
            base_shift: gain * m.start_velocity,
            half_velocity: 0.5 * vel,
            third_velocity: (1.0 / 3.0) * vel,
            third_accel: (1.0 / 3.0) * m.half_acceleration,
            quarter_accel: 0.25 * m.half_acceleration,
        }
    }

    fn integrate(self, m: &Segment, mut base: f64, start: f64, end: f64, offset: f64) -> f64 {
        let (from, to) = (start.max(0.0), end.min(m.duration));
        base += self.base_shift;
        let i = |t: f64| t * (base + t * (self.half_velocity + t * self.third_accel));
        let w = |t: f64| t * t * (0.5 * base + t * (self.third_velocity + t * self.quarter_accel));
        (w(to) - w(from)) - offset * (i(to) - i(from))
    }
}

impl Advance {
    /// A time-ordered gain change list, in the same clock domain as the
    /// extrusion trajectory. A move uses the gain active at its phase start.
    pub fn new(smooth_time: f64, changes: &[(f64, f64)]) -> Result<Self, Error> {
        let half = 0.5 * smooth_time;
        let inverse_half_squared = if half == 0.0 {
            0.0
        } else {
            1.0 / (half * half)
        };
        if !(0.0..=0.2).contains(&smooth_time)
            || !inverse_half_squared.is_finite()
            || changes.len() > MAX_CHANGES
            || changes
                .iter()
                .any(|(time, gain)| !time.is_finite() || !gain.is_finite() || *gain < 0.0)
            || changes.windows(2).any(|c| c[0].0 >= c[1].0)
        {
            return Err(Error::Settings);
        }
        Ok(Self {
            half,
            inverse_half_squared,
            changes: changes.to_vec(),
        })
    }

    fn gain_for(&self, m: &Segment) -> f64 {
        if m.direction[1] == 0.0 {
            return 0.0;
        }
        let i = self.changes.partition_point(|(at, _)| *at <= m.at);
        i.checked_sub(1).map_or(0.0, |i| self.changes[i].1)
    }

    fn prepare_phases<T>(
        &self,
        segs: &[Segment],
        mut prepare: impl FnMut(&Segment, f64) -> T,
    ) -> Vec<T> {
        if self.half == 0.0 {
            return Vec::new();
        }
        // The iterative crossing solver revisits the same phases. Resolve
        // gains once by traversing both sorted timelines in lockstep.
        let mut next_change = self.changes.iter().peekable();
        let mut gain = 0.0;
        segs.iter()
            .map(|m| {
                while let Some(&(_, active)) = next_change.next_if(|&&(at, _)| at <= m.at) {
                    gain = active;
                }
                let effective = if m.direction[1] == 0.0 { 0.0 } else { gain };
                prepare(m, effective)
            })
            .collect()
    }

    /// Interpolate a prepared E trajectory. X is filament position and Y's
    /// direction flag controls advance per phase. Query outside the range is
    /// evaluated against a stationary sentinel at the nearest endpoint.
    pub fn sample(&self, timeline: &Trajectory, time: f64) -> Result<Option<f64>, Error> {
        if !time.is_finite() {
            return Err(Error::Time);
        }
        let segs = timeline.segments();
        let Some(first) = segs.first() else {
            return Ok(None);
        };
        let last = segs.last().unwrap();
        // Outside the only relevant window, an idle reading is exact.
        let end_pos = last.local(last.duration)[0];
        if time <= first.at - self.half {
            return Ok(Some(first.position[0]));
        }
        if time >= last.end() + self.half {
            return Ok(Some(end_pos));
        }

        let head = idle(first.at - 1.0, 1.0, first.position[0]);
        let tail = idle(last.end(), 1.0, end_pos);
        let insertion = segs.partition_point(|m| m.at <= time);
        let k = if time >= tail.at {
            segs.len() as isize
        } else {
            insertion as isize - 1
        };
        let m = pick(segs, &head, &tail, k);
        let lookup = |i| {
            let phase = pick(segs, &head, &tail, i);
            (phase, Integrals::new(phase, self.gain_for(phase)))
        };
        Ok(Some(self.sample_relative(lookup, k, time - m.at)?))
    }

    fn sample_relative<'a>(
        &self,
        lookup: impl Fn(isize) -> (&'a Segment, Integrals),
        k: isize,
        local: f64,
    ) -> Result<f64, Error> {
        let (m, coeff) = lookup(k);
        let base = m.position[0];
        if self.half == 0.0 {
            return Ok(base + (m.start_velocity + m.half_acceleration * local) * local);
        }
        let (mut left, mut right) = (local - self.half, local + self.half);
        let mut area = coeff.integrate(m, 0.0, left, local, left);
        area -= coeff.integrate(m, 0.0, local, right, right);
        let mut budget = MAX_PHASES_PER_SAMPLE;
        let mut idx = k;
        while left < 0.0 {
            budget = budget.checked_sub(1).ok_or(Error::TooManyPhases)?;
            idx -= 1;
            let (prev, coeff) = lookup(idx);
            left += prev.duration;
            area += coeff.integrate(prev, prev.position[0] - base, left, prev.duration, left);
        }
        idx = k;
        let mut m = m;
        while right > m.duration {
            budget = budget.checked_sub(1).ok_or(Error::TooManyPhases)?;
            right -= m.duration;
            idx += 1;
            let (next, coeff) = lookup(idx);
            m = next;
            area -= coeff.integrate(m, m.position[0] - base, 0.0, right, right);
        }
        let sample = base + area * self.inverse_half_squared;
        if !sample.is_finite() {
            return Err(Error::Time);
        }
        Ok(sample)
    }

    /// Offline raw crossings for an E timeline. Run the same lead/trailing
    /// spans around active nominal phases that stock uses for smoothing.
    pub fn crossings(
        &self,
        timeline: &Trajectory,
        distance: f64,
    ) -> Result<Vec<Event>, step_times::Error> {
        let initial = timeline.segments().first().map_or(0., |s| s.position[0]);
        self.crossings_from(timeline, distance, initial)
    }

    /// Match phase-local nominal geometry without resetting the previously
    /// quantized motor position to a fractional nominal E coordinate.
    pub(crate) fn crossings_from(
        &self,
        timeline: &Trajectory,
        distance: f64,
        initial: f64,
    ) -> Result<Vec<Event>, step_times::Error> {
        let segs = timeline.segments();
        let Some((first, last)) = segs.first().zip(segs.last()) else {
            return Ok(Vec::new());
        };
        let mut solver = Solver::new(Axis::X, distance, [initial, 0., 0.])?;
        let head = idle(first.at - 1.0, 1.0, first.position[0]);
        let tail = idle(last.end(), 1.0, last.local(last.duration)[0]);
        let coefficients = self.prepare_phases(segs, Integrals::new);
        let inactive = Integrals::new(&head, 0.0);
        let lookup = |i: isize| {
            (
                pick(segs, &head, &tail, i),
                coefficients.get(i as usize).copied().unwrap_or(inactive),
            )
        };
        let mut out = Vec::new();
        let mut solve = |idx: usize, start: f64, end: f64| -> Result<(), step_times::Error> {
            let room = 65_535_usize
                .checked_sub(out.len())
                .filter(|n| *n > 0)
                .ok_or(step_times::Error::EventLimit)?;
            let seg = pick(segs, &head, &tail, idx as isize);
            solver.solve_range_into(seg, start.max(0.0)..end, room, &mut out, |t| {
                self.sample_relative(lookup, idx as isize, t)
                    .map_err(|_| step_times::Error::Invalid)
            })
        };
        let mut force_until = self.half;
        let mut skipped = None;
        for (i, m) in (0..=segs.len()).map(|i| (i, pick(segs, &head, &tail, i as isize))) {
            if m.direction[0] != 0.0 {
                if let Some(inactive) = skipped.take() {
                    let lead = (m.at - self.half).max(force_until);
                    if lead < m.at {
                        solve(inactive, lead, m.at)?;
                    }
                }
                solve(i, m.at, m.end())?;
                force_until = m.end() + self.half;
            } else {
                if m.at < force_until && m.end() > 0.0 {
                    solve(i, m.at, m.end().min(force_until).max(0.0))?;
                }
                skipped = Some(i);
            }
        }
        Ok(out)
    }
}

fn pick<'a>(segs: &'a [Segment], head: &'a Segment, tail: &'a Segment, i: isize) -> &'a Segment {
    if i < 0 {
        head
    } else {
        segs.get(i as usize).unwrap_or(tail)
    }
}

fn idle(at: f64, duration: f64, position: f64) -> Segment {
    Segment {
        at,
        duration,
        position: [position, 0.0, 0.0],
        direction: [0.0; 3],
        start_velocity: 0.0,
        half_acceleration: 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{extrusion::append_nominal, lookahead::Profile};

    fn extrusion(direction: [f64; 4]) -> Trajectory {
        let mut q = Trajectory::new(8).unwrap();
        append_nominal(
            &mut q,
            2.0,
            &Profile {
                start: [0.; 4],
                end: [0.; 4],
                direction,
                kinematic: direction[0] != 0.0,
                accel: 100.,
                start_v: 10.,
                cruise_v: 10.,
                end_v: 10.,
                accel_t: 0.,
                cruise_t: 1.,
                decel_t: 0.,
            },
        )
        .unwrap();
        q
    }

    #[test]
    fn constant_velocity_inside_window_and_no_advance_for_retraction() {
        let a = Advance::new(0.08, &[(0., 0.1)]).unwrap();
        let forward = extrusion([1., 0., 0., 1.]);
        assert!((a.sample(&forward, 2.5).unwrap().unwrap() - 6.0).abs() < 1e-12);
        let reverse = extrusion([1., 0., 0., -1.]);
        assert!((a.sample(&reverse, 2.5).unwrap().unwrap() + 5.0).abs() < 1e-12);
        assert_eq!(a.sample(&forward, 20.), Ok(Some(10.)));
        assert!(Advance::new(0.201, &[]).is_err());
    }

    #[test]
    fn prepared_integrals_keep_stock_operation_order() {
        for sv in [-23.7, -0.0, 0.0007, 38.6] {
            for accel in [-1715.5, -0.0, 0.31, 166.53] {
                let mut m = idle(0.0, 0.25, 0.0);
                m.start_velocity = sv;
                m.half_acceleration = accel;
                for gain in [0.0, 0.021, 0.0773] {
                    let prepared = Integrals::new(&m, gain);
                    for (mut base, start, end, offset) in [
                        (0.0, -0.012, 0.017, -0.012),
                        (-0.0, 0.0, 0.0, 0.013),
                        (-0.25, 0.201, 0.332, 0.29),
                    ] {
                        let actual = prepared.integrate(&m, base, start, end, offset);
                        base += gain * m.start_velocity;
                        let vel = m.start_velocity + gain * 2.0 * m.half_acceleration;
                        let i = |t: f64| t * (base + t * (0.5 * vel + t * ((1.0 / 3.0) * accel)));
                        let w = |t: f64| {
                            t * t * (0.5 * base + t * ((1.0 / 3.0) * vel + t * (0.25 * accel)))
                        };
                        let (from, to) = (start.max(0.0), end.min(m.duration));
                        let expected = (w(to) - w(from)) - offset * (i(to) - i(from));
                        assert_eq!(actual.to_bits(), expected.to_bits());
                    }
                }
            }
        }
    }

    #[test]
    fn prepared_gains_match_queries_at_exact_phase_boundaries() {
        let a = Advance::new(0.08, &[(1.0, 0.1), (2.0, 0.2), (2.5, 0.0), (4.0, 0.4)]).unwrap();
        let phases: Vec<_> = [
            (0.5, true),
            (1.0, false),
            (1.2, true),
            (2.0, true),
            (2.5, true),
            (4.0, false),
            (5.0, true),
        ]
        .into_iter()
        .map(|(at, eligible)| {
            let mut phase = idle(at, 0.1, 0.0);
            phase.direction[1] = if eligible { 1.0 } else { 0.0 };
            phase
        })
        .collect();
        let prepared = a.prepare_phases(&phases, |_, gain| gain);
        assert_eq!(prepared, [0.0, 0.0, 0.1, 0.2, 0.0, 0.0, 0.4]);
        assert_eq!(
            prepared,
            phases.iter().map(|m| a.gain_for(m)).collect::<Vec<_>>()
        );
        assert!(Advance::new(0.0, &[(0.0, 0.1)])
            .unwrap()
            .prepare_phases(&phases, |_, gain| gain)
            .is_empty());
    }
}
