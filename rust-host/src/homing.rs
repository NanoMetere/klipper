// SPDX-License-Identifier: GPL-3.0-only
//! Offline single-rail Cartesian/CoreXY homing and probe arithmetic.
//! Adapted from Klipper extras/homing.py, kinematics/cartesian.py and corexy.py.
//! Upstream copyright (C) 2016-2026 Kevin O'Connor <kevin@koconnor.net>.
//! Counter snapshots are inputs; no motor, MCU, endstop or probe is driven.

use crate::step_times::Axis;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kinematics {
    Cartesian,
    CoreXy,
}

impl Kinematics {
    fn axes(self) -> [Axis; 3] {
        match self {
            Self::Cartesian => [Axis::X, Axis::Y, Axis::Z],
            Self::CoreXy => [Axis::CoreXYPlus, Axis::CoreXYMinus, Axis::Z],
        }
    }

    fn xyz(self, [x, y, z]: [f64; 3]) -> [f64; 3] {
        match self {
            Self::Cartesian => [x, y, z],
            Self::CoreXy => [0.5 * (x + y), 0.5 * (x - y), z],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Invalid,
    NoTrigger,
    StillTriggered { motor: usize },
    Pending,
    Stale,
    Unhomed,
}

#[derive(Debug, Clone, Copy)]
pub struct Counter {
    pub start: i32,
    pub trigger: i32,
    pub halt: i32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RecordedPositions {
    pub trigger: [f64; 3],
    pub halt: [f64; 3],
}

fn distance(start: [f64; 3], end: [f64; 3]) -> (f64, [f64; 3]) {
    let d: [f64; 3] = std::array::from_fn(|i| end[i] - start[i]);
    (d.iter().map(|x| x * x).sum::<f64>().sqrt(), d)
}

/// The per-endstop maximum motor step rate, expressed as seconds per step.
/// Motor indices are those linked to this endstop; for a CoreXY XY endstop
/// both XY motors are normally linked.
pub fn sample_interval(
    kinematics: Kinematics,
    start: [f64; 3],
    end: [f64; 3],
    speed: f64,
    steps: [f64; 3],
    motor_mask: u8,
) -> Result<f64, Error> {
    if !speed.is_finite()
        || speed <= 0.0
        || !(1..=7).contains(&motor_mask)
        || steps.iter().any(|d| !d.is_finite() || *d <= 0.0)
    {
        return Err(Error::Invalid);
    }
    let (travel, _) = distance(start, end);
    let mut fastest: f64 = 0.0;
    for (i, axis) in kinematics.axes().into_iter().enumerate() {
        if motor_mask & (1 << i) != 0 {
            let n = (axis.position(start) - axis.position(end)).abs() / steps[i];
            if !n.is_finite() {
                return Err(Error::Invalid);
            }
            fastest = fastest.max(n);
        }
    }
    let time = if fastest <= 0.0 {
        0.001
    } else {
        (travel / speed) / fastest
    };
    (travel.is_finite() && time.is_finite() && time > 0.0)
        .then_some(time)
        .ok_or(Error::Invalid)
}

/// Stock retract target and the virtual start used for the second approach.
pub fn retract(
    start: [f64; 3],
    end: [f64; 3],
    retract_distance: f64,
) -> Result<([f64; 3], [f64; 3]), Error> {
    let (travel, delta) = distance(start, end);
    if !travel.is_finite()
        || travel <= 0.0
        || !retract_distance.is_finite()
        || retract_distance <= 0.0
    {
        return Err(Error::Invalid);
    }
    let ratio = (retract_distance / travel).min(1.0);
    let update = |position: [f64; 3]| std::array::from_fn(|i| position[i] - delta[i] * ratio);
    let target = update(end);
    let second = update(target);
    if target.iter().chain(&second).any(|v| !v.is_finite()) {
        return Err(Error::Invalid);
    }
    Ok((target, second))
}

/// No snapshots are trusted if an armed endstop did not report a trigger.
/// An absent entry is an unlinked motor, not a successful endstop response.
pub fn check_trigger(
    triggered: bool,
    readings: [Option<Counter>; 3],
    require_movement: bool,
) -> Result<(), Error> {
    if !triggered {
        return Err(Error::NoTrigger);
    }
    if readings.iter().all(Option::is_none) {
        return Err(Error::Invalid);
    }
    if require_movement {
        for (motor, value) in readings.into_iter().enumerate() {
            if value.is_some_and(|c| c.start == c.trigger) {
                return Err(Error::StillTriggered { motor });
            }
        }
    }
    Ok(())
}

/// For a probe, reconstruct trigger and final halt positions based on steps
/// relative to the start. For a mechanical endstop, the supplied origin is
/// the nominal home coordinate and motor travel after trigger is overshoot.
pub fn recorded_positions(
    kinematics: Kinematics,
    origin: [f64; 3],
    steps: [f64; 3],
    counts: [Option<Counter>; 3],
    probe: bool,
) -> Result<RecordedPositions, Error> {
    if steps.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err(Error::Invalid);
    }
    let motor = kinematics.axes().map(|axis| axis.position(origin));
    let project = |trigger: bool| {
        kinematics.xyz(std::array::from_fn(|i| {
            let offset = counts[i].map_or(0_i64, |c| {
                let (reference, observed) = if probe {
                    (c.start, if trigger { c.trigger } else { c.halt })
                } else {
                    (c.trigger, c.halt)
                };
                i64::from(observed) - i64::from(reference)
            });
            motor[i] + offset as f64 * steps[i]
        }))
    };
    let trigger = if probe { project(true) } else { origin };
    let halt = if probe || counts.iter().flatten().any(|c| c.trigger != c.halt) {
        project(false)
    } else {
        origin
    };
    if trigger.iter().chain(&halt).any(|v| !v.is_finite()) {
        return Err(Error::Invalid);
    }
    Ok(RecordedPositions { trigger, halt })
}

/// Simulated homing gate. No physical operation is authorized by this type.
/// A mechanical-home result is accepted only for the currently armed move
/// and expected motor set. A failed matching attempt clears known homing.
pub struct VirtualHoming {
    kinematics: Kinematics,
    position: [f64; 3],
    steps: [f64; 3],
    completed: u8,
    generation: u64,
    pending: Option<(u8, f64, bool)>,
}

impl VirtualHoming {
    pub fn new(kinematics: Kinematics, position: [f64; 3], steps: [f64; 3]) -> Result<Self, Error> {
        if position.iter().any(|x| !x.is_finite())
            || steps.iter().any(|x| !x.is_finite() || *x <= 0.0)
        {
            return Err(Error::Invalid);
        }
        Ok(Self {
            kinematics,
            position,
            steps,
            completed: 0,
            generation: 0,
            pending: None,
        })
    }

    pub fn homed_axes(&self) -> u8 {
        self.completed
    }

    /// Returns the pose only once all three axes are verified and no home is in progress.
    pub fn ready_position(&self) -> Result<[f64; 3], Error> {
        if self.pending.is_some() {
            return Err(Error::Pending);
        }
        if self.completed != 7 {
            return Err(Error::Unhomed);
        }
        Ok(self.position)
    }

    pub(crate) fn ready_position_for(
        &self,
        kinematics: Kinematics,
        step_distances: [f64; 3],
    ) -> Result<[f64; 3], Error> {
        if kinematics != self.kinematics || step_distances != self.steps {
            return Err(Error::Invalid);
        }
        self.ready_position()
    }

    /// Start an offline mechanical-home attempt. Axes are numbered 0 (X),
    /// 1 (Y), and 2 (Z). The token must be returned with the counter snapshots.
    pub fn arm(&mut self, axis: u8, endstop: f64, after_retract: bool) -> Result<u64, Error> {
        if axis > 2 || !endstop.is_finite() {
            return Err(Error::Invalid);
        }
        if self.pending.is_some() {
            return Err(Error::Pending);
        }
        self.generation = self.generation.checked_add(1).ok_or(Error::Invalid)?;
        self.completed &= !(1 << axis);
        self.pending = Some((axis, endstop, after_retract));
        Ok(self.generation)
    }

    pub fn complete(
        &mut self,
        token: u64,
        readings: [Option<Counter>; 3],
        triggered: bool,
    ) -> Result<RecordedPositions, Error> {
        if self.generation != token {
            return Err(Error::Stale);
        }
        let (axis, endstop, after_retract) = self.pending.take().ok_or(Error::Stale)?;
        let expected = match (self.kinematics, axis) {
            (Kinematics::CoreXy, 0 | 1) => 3,
            _ => 1 << axis,
        };
        let actual = readings.iter().enumerate().fold(0, |mask, (i, reading)| {
            mask | (u8::from(reading.is_some()) << i)
        });
        let mut origin = self.position;
        origin[axis as usize] = endstop;
        let result = check_trigger(triggered, readings, after_retract).and_then(|()| {
            if expected != actual {
                return Err(Error::Invalid);
            }
            recorded_positions(self.kinematics, origin, self.steps, readings, false)
        });
        match result {
            Ok(p) => {
                self.completed |= 1 << axis;
                self.position = p.halt;
            }
            Err(_) => self.completed = 0,
        }
        result
    }

    /// Explicitly invalidate snapshots after a restart, motor release, or
    /// simulated interruption. Old completion tokens cannot restore state.
    pub fn invalidate(&mut self) {
        self.pending = None;
        self.completed = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn homing_failures_never_count_as_success_and_unlinked_motors_are_ignored() {
        let c = Some(Counter {
            start: 20,
            trigger: 20,
            halt: 20,
        });
        assert_eq!(
            check_trigger(false, [None, c, None], false),
            Err(Error::NoTrigger)
        );
        assert_eq!(
            check_trigger(true, [None, c, None], true),
            Err(Error::StillTriggered { motor: 1 })
        );
        assert_eq!(check_trigger(true, [None, c, None], false), Ok(()));
        assert_eq!(check_trigger(true, [None; 3], false), Err(Error::Invalid));
    }

    #[test]
    fn virtual_gate_rejects_stale_incomplete_and_failed_homing() {
        let mut state = VirtualHoming::new(Kinematics::CoreXy, [1., 2., 3.], [0.01; 3]).unwrap();
        let reading = Some(Counter {
            start: 10,
            trigger: 8,
            halt: 7,
        });
        assert_eq!(state.ready_position(), Err(Error::Unhomed));
        let token = state.arm(0, 0., false).unwrap();
        assert_eq!(state.ready_position(), Err(Error::Pending));
        assert_eq!(
            state.complete(token - 1, [reading; 3], true),
            Err(Error::Stale)
        );
        assert_eq!(
            state.complete(token, [reading, None, None], true),
            Err(Error::Invalid)
        );
        assert_eq!(state.homed_axes(), 0);
        for axis in 0..3 {
            let token = state.arm(axis, f64::from(axis), true).unwrap();
            let readings = if axis != 2 {
                [reading, reading, None]
            } else {
                [None, None, reading]
            };
            state.complete(token, readings, true).unwrap();
        }
        assert_eq!(state.homed_axes(), 0b111);
        let [x, y, z] = state.ready_position().unwrap();
        assert!((x + 0.02).abs() < 1e-12 && (y - 1.).abs() < 1e-12 && (z - 1.99).abs() < 1e-12);
        let aborted = state.arm(2, 0., false).unwrap();
        state.invalidate();
        assert_eq!(
            state.complete(aborted, [None, None, reading], true),
            Err(Error::Stale)
        );
        assert_eq!(state.ready_position(), Err(Error::Unhomed));
    }

    #[test]
    fn distinct_trigger_halt_and_full_width_step_counter_difference() {
        let readings = [
            Some(Counter {
                start: i32::MIN,
                trigger: 0,
                halt: i32::MAX,
            }),
            None,
            None,
        ];
        let positions =
            recorded_positions(Kinematics::CoreXy, [0.; 3], [0.25; 3], readings, true).unwrap();
        assert_eq!(positions.trigger, [268_435_456., 268_435_456., 0.]);
        assert_eq!(positions.halt, [536_870_911.875, 536_870_911.875, 0.]);
    }
}
