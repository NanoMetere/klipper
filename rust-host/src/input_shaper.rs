// SPDX-License-Identifier: GPL-3.0-only
//! Offline input-shaper pulse convolution on a bounded motion timeline.
//! Adapted from Klipper klippy/chelper/kin_shaper.c and itersolve.c.
//! Upstream copyright (C) 2018-2025 Kevin O'Connor and Dmitry Butyugin.
//! No accelerometer access, calibration or device actions.

use crate::{
    step_times::{self, Axis, Event, Solver},
    trajectory::{Segment, Trajectory},
};

const MAX_PULSES: usize = 10;
const MAX_WINDOW: f64 = 0.5;
const MAX_PHASES: usize = 16_384;
const MAX_EVENTS: usize = 65_535;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Parameters,
    Time,
}

#[derive(Clone, Copy, Default)]
struct Pulse {
    time: f64,
    amplitude: f64,
}

#[derive(Clone, Default)]
pub struct Pulses {
    values: [Pulse; MAX_PULSES],
    len: usize,
    split: usize,
}

impl Pulses {
    /// Input is the usual unnormalized (time, amplitude) sequence.
    /// Reverse and shift it exactly as Klippy does before convolution.
    pub fn new(input: &[(f64, f64)]) -> Result<Self, Error> {
        if input.len() > MAX_PULSES
            || input.iter().any(|&(t, a)| {
                !(0.0..=MAX_WINDOW).contains(&t) || !a.is_finite() || a.abs() > 1000.0
            })
            || input.windows(2).any(|w| w[0].0 >= w[1].0)
        {
            return Err(Error::Parameters);
        }
        let mut out = Self::default();
        if input.is_empty() {
            return Ok(out);
        }
        let sum: f64 = input.iter().map(|&(_, a)| a).sum();
        if sum < 1e-8 || !sum.is_finite() {
            return Err(Error::Parameters);
        }
        let inverse = 1.0 / sum;
        out.len = input.len();
        let pulses = &mut out.values[..out.len];
        for (pulse, &(time, amplitude)) in pulses.iter_mut().zip(input.iter().rev()) {
            *pulse = Pulse {
                time: -time,
                amplitude: amplitude * inverse,
            };
        }
        let shift: f64 = pulses.iter().map(|p| p.amplitude * p.time).sum();
        for pulse in pulses.iter_mut() {
            pulse.time -= shift;
            if pulse.time.abs() > MAX_WINDOW || pulse.amplitude.abs() > 16.0 {
                return Err(Error::Parameters);
            }
        }
        out.split = pulses.partition_point(|p| p.time <= 0.0);
        Ok(out)
    }

    fn window(&self) -> (f64, f64) {
        if self.len == 0 {
            (0.0, 0.0)
        } else {
            (
                self.values[self.len - 1].time.max(0.0),
                (-self.values[0].time).max(0.0),
            )
        }
    }

    fn convolve(
        &self,
        phases: &Phases<'_>,
        index: isize,
        time: f64,
        axis: usize,
    ) -> Result<f64, Error> {
        let nominal = |segment: &Segment, t| {
            let distance = (segment.start_velocity + segment.half_acceleration * t) * t;
            segment.position[axis] + segment.direction[axis] * distance
        };
        let current = phases.pick(index);
        if self.len == 0 {
            return Ok(nominal(current, time));
        }
        let mut value = 0.0;
        let mut i = index;
        let mut t = time;
        let mut segment = current;
        let mut budget = MAX_PHASES;
        for pulse in self.values[..self.split].iter().rev() {
            t += pulse.time;
            while t < 0.0 {
                if i <= -1 {
                    return Err(Error::Time);
                }
                budget = budget.checked_sub(1).ok_or(Error::Time)?;
                i -= 1;
                segment = phases.pick(i);
                t += segment.duration;
            }
            value += pulse.amplitude * nominal(segment, t);
            t -= pulse.time;
        }
        i = index;
        t = time;
        segment = current;
        for pulse in &self.values[self.split..self.len] {
            t += pulse.time;
            while t > segment.duration {
                if i >= phases.segments.len() as isize {
                    return Err(Error::Time);
                }
                budget = budget.checked_sub(1).ok_or(Error::Time)?;
                t -= segment.duration;
                i += 1;
                segment = phases.pick(i);
            }
            value += pulse.amplitude * nominal(segment, t);
            t -= pulse.time;
        }
        value.is_finite().then_some(value).ok_or(Error::Time)
    }
}

struct Phases<'a> {
    segments: &'a [Segment],
    head: Segment,
    tail: Segment,
}

impl<'a> Phases<'a> {
    fn new(timeline: &'a Trajectory) -> Option<Self> {
        let segments = timeline.segments();
        let (first, last) = segments.first().zip(segments.last())?;
        let idle = |at, duration, position| Segment {
            at,
            duration,
            position,
            direction: [0.0; 3],
            start_velocity: 0.0,
            half_acceleration: 0.0,
        };
        Some(Self {
            segments,
            head: idle(first.at - 1.0, 1.0, first.position),
            tail: idle(last.end(), 1.0, last.local(last.duration)),
        })
    }

    fn pick(&self, i: isize) -> &Segment {
        if i < 0 {
            &self.head
        } else {
            self.segments.get(i as usize).unwrap_or(&self.tail)
        }
    }
}

#[derive(Clone, Default)]
pub struct Shaper {
    axes: [Pulses; 3],
}

impl Shaper {
    pub fn new(axes: [Pulses; 3]) -> Self {
        Self { axes }
    }

    pub fn is_enabled_for(&self, lane: Axis) -> bool {
        selected(lane).iter().any(|&axis| self.axes[axis].len != 0)
    }

    /// Look-ahead and history windows for a motor lane.
    pub fn window(&self, lane: Axis) -> (f64, f64) {
        let mut out = (0.0_f64, 0.0_f64);
        for i in selected(lane) {
            let (before, after) = self.axes[*i].window();
            out = (out.0.max(before), out.1.max(after));
        }
        out
    }

    pub fn sample(
        &self,
        timeline: &Trajectory,
        lane: Axis,
        time: f64,
    ) -> Result<Option<f64>, Error> {
        if !time.is_finite() {
            return Err(Error::Time);
        }
        let Some(phases) = Phases::new(timeline) else {
            return Ok(None);
        };
        let i = phases.segments.partition_point(|s| s.at <= time) as isize - 1;
        let i = if time >= phases.tail.at {
            phases.segments.len() as isize
        } else {
            i
        };
        let m = phases.pick(i);
        let local = (time - m.at).clamp(
            if i < 0 { MAX_WINDOW } else { 0.0 },
            if i >= phases.segments.len() as isize {
                1.0 - MAX_WINDOW
            } else {
                m.duration
            },
        );
        self.sample_relative(&phases, lane, i, local).map(Some)
    }

    fn sample_relative(
        &self,
        phases: &Phases<'_>,
        lane: Axis,
        index: isize,
        local: f64,
    ) -> Result<f64, Error> {
        let mut coord = [0.0; 3];
        for &i in selected(lane) {
            coord[i] = self.axes[i].convolve(phases, index, local, i)?;
        }
        let value = lane.position(coord);
        value.is_finite().then_some(value).ok_or(Error::Time)
    }

    /// Produce bounded inert crossing/commit events. The extra intervals
    /// across neighboring inactive phases mirror stock's activity windows.
    pub fn crossings(
        &self,
        timeline: &Trajectory,
        lane: Axis,
        distance: f64,
        start: [f64; 3],
    ) -> Result<Vec<Event>, step_times::Error> {
        self.crossing_batches(
            timeline,
            lane,
            distance,
            start,
            #[cfg(feature = "long-step-preview")]
            None,
            |_| Ok(()),
        )
    }

    // The visitor may leave the collected events in place or consume and
    // clear them. The cumulative work limit is independent of either choice.
    pub(crate) fn crossing_batches<E: From<step_times::Error>>(
        &self,
        timeline: &Trajectory,
        lane: Axis,
        distance: f64,
        start: [f64; 3],
        #[cfg(feature = "long-step-preview")] limit: Option<usize>,
        mut visit: impl FnMut(&mut Vec<Event>) -> Result<(), E>,
    ) -> Result<Vec<Event>, E> {
        let Some(phases) = Phases::new(timeline) else {
            return Ok(Vec::new());
        };
        let (pre, post) = self.window(lane);
        let mut solver = Solver::new(lane, distance, start)?;
        let mut events = Vec::new();
        #[cfg(not(feature = "long-step-preview"))]
        let mut remaining = MAX_EVENTS;
        #[cfg(feature = "long-step-preview")]
        let mut remaining = limit.unwrap_or(MAX_EVENTS);
        let mut solve = |i: isize, lo: f64, hi: f64| -> Result<(), E> {
            if remaining == 0 {
                return Err(step_times::Error::EventLimit.into());
            }
            let segment = phases.pick(i);
            let before = events.len();
            #[cfg(feature = "long-step-preview")]
            let room = if limit.is_some() {
                remaining.min(MAX_EVENTS)
            } else {
                remaining
            };
            #[cfg(not(feature = "long-step-preview"))]
            let room = remaining;
            solver.solve_range_into(segment, lo..hi, room, &mut events, |local| {
                self.sample_relative(&phases, lane, i, local)
                    .map_err(|_| step_times::Error::Invalid)
            })?;
            remaining -= events.len() - before;
            visit(&mut events)
        };
        let mut force_until = post;
        let mut skipped = 0;
        for i in 0..=phases.segments.len() as isize {
            let m = phases.pick(i);
            if m.end() <= 0.0 {
                continue;
            }
            if selected(lane).iter().any(|&axis| m.direction[axis] != 0.0) {
                if skipped != 0 && pre > 0.0 {
                    let lead = (m.at - pre).max(force_until).max(0.0);
                    let mut previous = i - 1;
                    while previous > i - skipped && phases.pick(previous).at > lead {
                        previous -= 1;
                    }
                    for j in previous..i {
                        let s = phases.pick(j);
                        if s.end() > lead {
                            solve(j, lead.max(s.at), s.end())?;
                        }
                    }
                }
                solve(i, m.at.max(0.0), m.end())?;
                force_until = m.end() + post;
                skipped = 0;
            } else if m.at < force_until {
                solve(i, m.at.max(0.0), force_until.min(m.end()))?;
                skipped = 1;
            } else {
                skipped += 1;
            }
        }
        Ok(events)
    }
}

fn selected(lane: Axis) -> &'static [usize] {
    match lane {
        Axis::X => &[0],
        Axis::Y => &[1],
        Axis::Z => &[2],
        Axis::CoreXYPlus | Axis::CoreXYMinus => &[0, 1],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trajectory::Trapezoid;

    #[test]
    fn weighted_time_center_preserves_linear_time_and_boundaries_hold() {
        let pulse = Pulses::new(&[(0.0, 1.0), (0.02, 0.8)]).unwrap();
        let shaper = Shaper::new([pulse, Pulses::default(), Pulses::default()]);
        let mut q = Trajectory::new(10).unwrap();
        q.append(Trapezoid {
            at: 2.0,
            phase_sec: [0.0, 0.5, 0.0],
            start: [10., 20., 30.],
            direction: [1.0, 0., 0.],
            start_velocity: 32.,
            cruise_velocity: 32.,
            accel: 0.,
        })
        .unwrap();
        for (time, expected) in [(0., 10.), (2.1, 13.2), (2.25, 18.), (9., 26.)] {
            assert!((shaper.sample(&q, Axis::X, time).unwrap().unwrap() - expected).abs() < 1e-12);
        }
        assert!(!shaper
            .crossings(&q, Axis::X, 0.01, [10., 20., 30.])
            .unwrap()
            .is_empty());
        assert!(Pulses::new(&[(0.02, 1.0), (0.02, 1.0)]).is_err());
        assert!(Pulses::new(&[(0., 1.0), (0.02, -1.0)]).is_err());
    }
}
