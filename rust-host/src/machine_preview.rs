// SPDX-License-Identifier: GPL-3.0-only
//! Connects a strict machine config subset to the in-memory XYZ step trace.
//! No homing, peripherals, tool/pin setup, protocol writes or hardware I/O.

use crate::{
    config::Config,
    gcode::{self, Output, Preview},
    lookahead::{Limits, Profile},
    step_clock::{self, Clock, Command},
    step_times::{self, Axis, Event, Solver},
    trajectory::{self, Trajectory},
};

const SECTIONS: [&str; 3] = ["stepper_x", "stepper_y", "stepper_z"];
const MAX_EVENTS: usize = 65_535;
const MAX_LINES: usize = 32768;

#[derive(Debug)]
pub enum Error {
    Config {
        section: &'static str,
        option: &'static str,
    },
    Unsupported,
    InitialPosition,
    MaximumLines,
    Gcode {
        line: usize,
        error: gcode::Error,
    },
    Trajectory(trajectory::Error),
    Solver(step_times::Error),
    Clock(step_clock::Error),
}

pub struct Machine {
    motor_axis: [Axis; 3],
    motor_step: [f64; 3],
    travel: [(f64, f64); 3],
    limits: Limits,
    z_velocity: f64,
    z_accel: f64,
}

fn config_error(section: &'static str, option: &'static str) -> Error {
    Error::Config { section, option }
}

fn number(
    cfg: &Config,
    section: &'static str,
    key: &'static str,
    default: Option<f64>,
) -> Result<f64, Error> {
    cfg.get(section, key)
        .map_or(default, |s| s.parse().ok())
        .filter(|v| v.is_finite())
        .ok_or_else(|| config_error(section, key))
}

fn gear_product(cfg: &Config, section: &'static str) -> Result<f64, Error> {
    let Some(raw) = cfg.get(section, "gear_ratio") else {
        return Ok(1.0);
    };
    fn valid(v: &str) -> Option<f64> {
        v.trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite() && *v > 0.0)
    }
    raw.split(',')
        .try_fold(1.0_f64, |product, part| {
            let (a, b) = part.split_once(':')?;
            let ratio = product * (valid(a)? / valid(b)?);
            (ratio.is_finite() && ratio > 0.0).then_some(ratio)
        })
        .ok_or_else(|| config_error(section, "gear_ratio"))
}

impl Machine {
    /// Supports one motor per X/A, Y/B and Z rail, no transform/pluggable
    /// geometry. Peripheral sections are not initialized or interpreted.
    pub fn from_config(config: &Config) -> Result<Self, Error> {
        let motor_axis = match config.get("printer", "kinematics") {
            Some("cartesian") => [Axis::X, Axis::Y, Axis::Z],
            Some("corexy") => [Axis::CoreXYPlus, Axis::CoreXYMinus, Axis::Z],
            _ => return Err(config_error("printer", "kinematics")),
        };
        if config
            .sections()
            .any(|s| s == "dual_carriage" || SECTIONS.iter().any(|p| s.starts_with(p) && s != *p))
        {
            return Err(Error::Unsupported);
        }
        let max_v = number(config, "printer", "max_velocity", None)?;
        let max_a = number(config, "printer", "max_accel", None)?;
        let scv = number(config, "printer", "square_corner_velocity", Some(5.0))?;
        let cruise_ratio = number(config, "printer", "minimum_cruise_ratio", Some(0.5))?;
        let limits = Limits::new(max_v, max_a, scv, cruise_ratio)
            .map_err(|_| config_error("printer", "motion limits"))?;
        let z_velocity = number(config, "printer", "max_z_velocity", Some(max_v))?;
        let z_accel = number(config, "printer", "max_z_accel", Some(max_a))?;
        if !(0.0 < z_velocity && z_velocity <= max_v && 0.0 < z_accel && z_accel <= max_a) {
            return Err(config_error("printer", "Z limits"));
        }
        let mut travel = [(0.0, 0.0); 3];
        let mut motor_step = [0.0; 3];
        for (i, section) in SECTIONS.into_iter().enumerate() {
            let minimum = number(config, section, "position_min", Some(0.0))?;
            let maximum = number(config, section, "position_max", None)?;
            if minimum >= maximum {
                return Err(config_error(section, "position_max"));
            }
            travel[i] = (minimum, maximum);
            let rotation = number(config, section, "rotation_distance", None)?;
            let integer = |key, fallback| -> Result<u32, Error> {
                config
                    .get(section, key)
                    .map_or(fallback, |s| s.parse().ok())
                    .filter(|v| *v > 0)
                    .ok_or_else(|| config_error(section, key))
            };
            let micro = integer("microsteps", None)?;
            let full = integer("full_steps_per_rotation", Some(200))?;
            if full % 4 != 0 {
                return Err(config_error(section, "full_steps_per_rotation"));
            }
            let steps = f64::from(micro) * f64::from(full) * gear_product(config, section)?;
            let step = rotation / steps;
            if !(1e-9..=1000.0).contains(&step) {
                return Err(config_error(section, "rotation_distance"));
            }
            motor_step[i] = step;
        }
        Ok(Self {
            motor_axis,
            motor_step,
            travel,
            limits,
            z_velocity,
            z_accel,
        })
    }

    pub fn step_distances(&self) -> [f64; 3] {
        self.motor_step
    }

    fn in_range(&self, pos: &[f64]) -> bool {
        self.travel
            .iter()
            .zip(pos)
            .all(|(&(lo, hi), x)| (lo..=hi).contains(x))
    }

    /// Previews a bounded script from an explicit virtual position. Inversion
    /// flags are caller-supplied (no parsing device pin configuration).
    pub fn trace(
        &self,
        script: &str,
        start: [f64; 3],
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<Trace, Error> {
        if !self.in_range(&start) || !at.is_finite() || at <= 0.0 {
            return Err(Error::InitialPosition);
        }
        if script.len() > 8 * 1024 * 1024 {
            return Err(Error::MaximumLines);
        }
        let mut preview =
            Preview::new_at(self.limits, 4096, None, [start[0], start[1], start[2], 0.0])
                .map_err(|error| Error::Gcode { line: 0, error })?;
        let mut queue = Trajectory::new(16_384).map_err(Error::Trajectory)?;
        let mut time = at;
        let mut n = 0;
        let mut append = |o: Output| -> Result<(), Error> {
            for m in o.moves {
                append_move(&mut queue, &mut time, &m).map_err(Error::Trajectory)?;
            }
            time += o.dwell_seconds.unwrap_or(0.0);
            if !time.is_finite() || time >= 1e12 {
                return Err(Error::Trajectory(trajectory::Error::Invalid));
            }
            Ok(())
        };
        for line in script.lines() {
            n += 1;
            if n > MAX_LINES {
                return Err(Error::MaximumLines);
            }
            let output = preview
                .line_with(line, |previous, next| {
                    let end = next.end_position();
                    if !self.in_range(&end[..3]) || end[3] != previous[3] {
                        return Err(gcode::Error::MoveOutsidePreview);
                    }
                    if let Some(z) = next.z_ratio() {
                        next.limit(z * self.z_velocity, z * self.z_accel)?;
                    }
                    Ok(())
                })
                .map_err(|error| Error::Gcode { line: n, error })?;
            append(output)?;
        }
        let moves = preview
            .finish()
            .map_err(|error| Error::Gcode { line: n + 1, error })?;
        append(Output {
            moves,
            dwell_seconds: None,
        })?;
        let mut commands = std::array::from_fn(|_| Vec::new());
        for (motor, output) in commands.iter_mut().enumerate() {
            let mut solver = Solver::new(self.motor_axis[motor], self.motor_step[motor], start)
                .map_err(Error::Solver)?;
            let mut events: Vec<Event> = Vec::new();
            for seg in queue.segments().iter().filter(|s| s.direction != [0.0; 3]) {
                let room = MAX_EVENTS
                    .checked_sub(events.len())
                    .filter(|n| *n > 0)
                    .ok_or(Error::Solver(step_times::Error::EventLimit))?;
                let more = solver
                    .crossing_events(seg, seg.at, seg.end(), room)
                    .map_err(Error::Solver)?;
                events.extend(more);
            }
            *output = step_clock::encode_trace(clock, &events, max_error, invert[motor])
                .map_err(Error::Clock)?;
        }
        Ok(Trace {
            motors: commands,
            duration: time - at,
        })
    }
}

fn append_move(queue: &mut Trajectory, at: &mut f64, m: &Profile) -> Result<(), trajectory::Error> {
    if m.kinematic {
        queue.append_profile(*at, m)?;
        *at = queue.segments().last().unwrap().end();
    } else {
        *at += m.accel_t + m.cruise_t + m.decel_t;
    }
    Ok(())
}

pub struct Trace {
    pub motors: [Vec<Command>; 3],
    pub duration: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    const CONFIG: &str = "[printer]\nkinematics: corexy\nmax_velocity: 160\nmax_accel: 2500\nmax_z_velocity: 10\n\
        [stepper_x]\nposition_max: 200\nrotation_distance: 40\nmicrosteps: 16\n\
        [stepper_y]\nposition_max: 200\nrotation_distance: 40\nmicrosteps: 16\n\
        [stepper_z]\nposition_max: 200\nrotation_distance: 8\nmicrosteps: 16\ngear_ratio: 80:16, 2:1";

    #[test]
    fn previews_corexy_and_rejects_physical_parameters_outside_supported_scope() {
        let cfg = crate::config::parse(
            &BTreeMap::from([("printer.cfg".into(), CONFIG.into())]),
            "printer.cfg",
        )
        .unwrap();
        let machine = Machine::from_config(&cfg).unwrap();
        assert_eq!(machine.step_distances(), [0.0125, 0.0125, 0.00025]);
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: 20e6,
        };
        let trace = machine
            .trace(
                "G1 X1 Y1 Z0.02 F60\nG1 X2 Y2",
                [0.; 3],
                2.0,
                clock,
                0,
                [false; 3],
            )
            .unwrap();
        assert!(
            !trace.motors[0].is_empty()
                && trace.motors[1].is_empty()
                && !trace.motors[2].is_empty()
        );
        assert!(trace.duration > 2.8);
        for text in ["G1 X-0.1", "G1 E0.1", "G28", "G1 Z200.1"] {
            assert!(matches!(
                machine.trace(text, [0.; 3], 2.0, clock, 0, [false; 3]),
                Err(Error::Gcode { line: 1, .. })
            ));
        }
    }
}
