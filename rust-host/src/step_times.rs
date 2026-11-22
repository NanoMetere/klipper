// SPDX-License-Identifier: GPL-3.0-only
//! Offline raw step-crossing solver adapted from Klipper's chelper/itersolve.c.
//! Upstream copyright (C) 2018-2020 Kevin O'Connor <kevin@koconnor.net>.
//! This is before clock conversion, compression, and rapid-reversal rollback.

use crate::trajectory::Segment;
use std::ops::Range;

const SEEK: f64 = 0.000100;
const MAX_EVENTS: usize = 65_536;

#[derive(Debug, Clone, Copy)]
pub enum Axis {
    X,
    Y,
    Z,
    CoreXYPlus,
    CoreXYMinus,
}

impl Axis {
    pub fn position(self, [x, y, z]: [f64; 3]) -> f64 {
        match self {
            Self::X => x,
            Self::Y => y,
            Self::Z => z,
            Self::CoreXYPlus => x + y,
            Self::CoreXYMinus => x - y,
        }
    }

    fn active(self, [x, y, z]: [f64; 3]) -> bool {
        match self {
            Self::X => x != 0.0,
            Self::Y => y != 0.0,
            Self::Z => z != 0.0,
            Self::CoreXYPlus | Self::CoreXYMinus => x != 0.0 || y != 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    /// Commit any pending step prior to a possible reversal (see stepcompress).
    Commit,
    Crossing {
        direction_positive: bool,
        segment_time: f64,
        offset: f64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Invalid,
    EventLimit,
    IterationLimit,
}

pub struct Solver {
    axis: Axis,
    dist: f64,
    pos: f64,
    positive: bool,
}

impl Solver {
    pub fn new(axis: Axis, step_distance: f64, initial_position: [f64; 3]) -> Result<Self, Error> {
        let pos = axis.position(initial_position);
        if !step_distance.is_finite()
            || step_distance <= 1e-9
            || !pos.is_finite()
            || pos + step_distance == pos
            || initial_position.iter().any(|x| !x.is_finite())
        {
            return Err(Error::Invalid);
        }
        Ok(Self {
            axis,
            dist: step_distance,
            pos,
            positive: false,
        })
    }

    pub fn commanded_position(&self) -> f64 {
        self.pos
    }

    /// Run an ordered, non-overlapping portion of a validated trajectory segment.
    /// On failure the solver state is unchanged and no partial events escape.
    /// The caller owns time-range order and whether to skip inactive segments.
    pub fn crossing_events(
        &mut self,
        m: &Segment,
        abs_start: f64,
        abs_end: f64,
        cap: usize,
    ) -> Result<Vec<Event>, Error> {
        let mut out = Vec::new();
        self.crossing_events_into(m, abs_start..abs_end, cap, &mut out)?;
        Ok(out)
    }

    /// Append to the caller's buffer. On error its prior events and solver
    /// position are preserved, even if the range produced partial crossings.
    pub(crate) fn crossing_events_into(
        &mut self,
        m: &Segment,
        times: Range<f64>,
        cap: usize,
        out: &mut Vec<Event>,
    ) -> Result<(), Error> {
        let axis = self.axis;
        if !axis.active(m.direction) {
            return Ok(());
        }
        self.solve_range_into(m, times, cap, out, |next| Ok(axis.position(m.local(next))))
    }

    pub(crate) fn solve_range_into(
        &mut self,
        m: &Segment,
        times: Range<f64>,
        cap: usize,
        out: &mut Vec<Event>,
        position: impl FnMut(f64) -> Result<f64, Error>,
    ) -> Result<(), Error> {
        let initial = out.len();
        let result = self.solve_range(m, times, cap, out, position);
        if result.is_err() {
            out.truncate(initial);
        }
        result
    }

    fn solve_range(
        &mut self,
        m: &Segment,
        times: Range<f64>,
        cap: usize,
        out: &mut Vec<Event>,
        mut position: impl FnMut(f64) -> Result<f64, Error>,
    ) -> Result<(), Error> {
        let Range {
            start: abs_start,
            end: abs_end,
        } = times;
        if !(1..=MAX_EVENTS).contains(&cap)
            || !abs_start.is_finite()
            || !abs_end.is_finite()
            || abs_start > abs_end
            || !m.at.is_finite()
            || !m.duration.is_finite()
            || m.duration <= 0.0
            || m.position
                .iter()
                .chain(&m.direction)
                .any(|x| !x.is_finite())
            || !m.start_velocity.is_finite()
            || !m.half_acceleration.is_finite()
        {
            return Err(Error::Invalid);
        }
        let start = (abs_start - m.at).max(0.0);
        let end = (abs_end - m.at).min(m.duration);
        if start >= end {
            return Ok(());
        }
        let initial = out.len();
        let mut push = |event| {
            if out.len() - initial == cap {
                return Err(Error::EventLimit);
            }
            out.push(event);
            Ok(())
        };
        let half = 0.5 * self.dist;
        let mut positive = self.positive;
        let mut target = self.pos + if positive { half } else { -half };
        let (mut guess, mut old) = ((start, self.pos), (start, self.pos));
        let (mut changed, mut bracket, mut oscillate) = (false, false, false);
        let (mut last, mut low, mut high) = (start, start, (start + SEEK).min(end));
        for _ in 0..cap * 64 + 128 {
            let guess_dist = guess.1 - target;
            let old_dist = old.1 - target;
            let mut next = (old.0 * guess_dist - guess.0 * old_dist) / (guess_dist - old_dist);
            if !(next > low && next < high) {
                if bracket {
                    next = (low + high) * 0.5;
                    oscillate = false;
                } else if guess.0 >= end {
                    self.pos = target - if positive { half } else { -half };
                    self.positive = positive;
                    return Ok(());
                } else {
                    next = high;
                    high = (2.0 * high - last).min(end);
                }
            }
            old = guess;
            // Evaluate at relative time to retain stock sub-tick precision.
            guess = (next, position(next)?);
            if !guess.1.is_finite() || !target.is_finite() {
                return Err(Error::Invalid);
            }
            let distance = guess.1 - target;
            if distance.abs() > 1e-9 {
                let relative = if positive { distance } else { -distance };
                if relative > 0.0 {
                    if bracket && old.0 <= low {
                        if oscillate {
                            old = guess;
                        }
                        oscillate = true;
                    }
                    high = guess.0;
                    bracket = true;
                } else if relative < -(half + half + 1e-8) {
                    positive = !positive;
                    target = if positive {
                        target + half + half
                    } else {
                        target - half - half
                    };
                    low = last;
                    high = guess.0;
                    changed = true;
                    bracket = true;
                    oscillate = false;
                } else {
                    low = guess.0;
                }
                if !bracket || high - low > 1e-9 {
                    if !changed && relative >= -half {
                        push(Event::Commit)?;
                    }
                    continue;
                }
            }
            push(Event::Crossing {
                direction_positive: positive,
                segment_time: m.at,
                offset: guess.0,
            })?;
            target = if positive {
                target + half + half
            } else {
                target - half - half
            };
            let mut delta = (1.5 * (guess.0 - last)).max(1e-9);
            if changed {
                delta = delta.min(SEEK);
            }
            last = guess.0;
            low = guess.0;
            high = (guess.0 + delta).min(end);
            changed = false;
            bracket = false;
            oscillate = false;
        }
        Err(Error::IterationLimit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retains_motor_state_after_bounded_failure_and_tracks_reversal() {
        let mut solve = Solver::new(Axis::CoreXYPlus, 0.25, [0.0; 3]).unwrap();
        let fwd = Segment {
            at: 1.0,
            duration: 1.0,
            position: [0.0; 3],
            direction: [0.5, 0.5, 0.0],
            start_velocity: 1.0,
            half_acceleration: 0.0,
        };
        assert_eq!(
            solve.crossing_events(&fwd, 1.0, 2.0, 1),
            Err(Error::EventLimit)
        );
        assert_eq!(solve.commanded_position(), 0.0);
        let mut prior = vec![Event::Commit];
        assert_eq!(
            solve.crossing_events_into(&fwd, 1.0..2.0, 1, &mut prior),
            Err(Error::EventLimit),
        );
        assert_eq!(prior, [Event::Commit]);
        assert_eq!(solve.commanded_position(), 0.0);
        let events = solve.crossing_events(&fwd, 1.0, 2.0, 2048).unwrap();
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(
                    e,
                    Event::Crossing {
                        direction_positive: true,
                        ..
                    }
                ))
                .count(),
            4
        );
        assert_eq!(solve.commanded_position(), 1.0);

        let rev = Segment {
            at: 2.0,
            position: [0.5, 0.5, 0.0],
            direction: [-0.5, -0.5, 0.0],
            ..fwd
        };
        let events = solve.crossing_events(&rev, 2.0, 3.0, 2048).unwrap();
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(
                    e,
                    Event::Crossing {
                        direction_positive: false,
                        ..
                    }
                ))
                .count(),
            4
        );
        assert_eq!(solve.commanded_position(), 0.0);
    }
}
