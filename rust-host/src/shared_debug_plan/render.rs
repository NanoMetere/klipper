// SPDX-License-Identifier: GPL-3.0-only
//! Offline typed composition of the bounded planner and step encoders.
//! Output is a trace only, with no connection to an MCU or printer.

use super::{plan, Error, Event, Options};
use crate::{
    extrusion::{append_nominal, prune_prepared, Extruder},
    pressure_advance::{Advance, WindowStepper},
    step_clock::{Clock, Command, TraceEncoder},
    step_times::{Axis, Solver},
    trajectory::Trajectory,
};

const MAX_EVENTS: usize = 65_535;

#[derive(Debug, Clone, Copy)]
pub struct OutputOptions {
    /// CoreXY A, CoreXY B, Z, shared E, in that order.
    pub step_distances: [f64; 4],
    pub invert: [bool; 4],
    pub clock_hz: f64,
    pub max_error_ticks: u32,
}

#[derive(Debug, PartialEq, Eq)]
pub struct MotionTrace {
    /// Ordered CoreXY A, CoreXY B, Z, shared E records.
    pub lanes: [Vec<Command>; 4],
}

#[derive(Debug)]
pub enum TraceError {
    Plan(Error),
    Trajectory(crate::trajectory::Error),
    Pressure(crate::pressure_advance::Error),
    Steps(crate::step_times::Error),
    Clock(crate::step_clock::Error),
    Options,
}

macro_rules! convert {
    ($($variant:ident: $source:path),*) => {$(
        impl From<$source> for TraceError {
            fn from(error: $source) -> Self { Self::$variant(error) }
        }
    )*};
}
convert!(
    Plan: Error,
    Trajectory: crate::trajectory::Error,
    Pressure: crate::pressure_advance::Error,
    Steps: crate::step_times::Error,
    Clock: crate::step_clock::Error
);

/// Plan and render one bounded no-pacing CoreXY job from G-code. Only a
/// fully rendered trace is returned. The initial XYZ is virtual, the two
/// temperatures are simulated, and no physical timing or I/O is performed.
pub fn trace(
    script: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
) -> Result<MotionTrace, TraceError> {
    if !(1.0..=1e9).contains(&wire.clock_hz) || wire.max_error_ticks > 200_000 {
        return Err(TraceError::Options);
    }
    let events = plan(script, start, options, geometries)?;
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.0,
        hz: wire.clock_hz,
    };
    let [a, b, z, e] = wire.step_distances;
    let mut solvers = [
        Solver::new(Axis::CoreXYPlus, a, start)?,
        Solver::new(Axis::CoreXYMinus, b, start)?,
        Solver::new(Axis::Z, z, start)?,
    ];
    let mut encoders = wire
        .invert
        .map(|invert| TraceEncoder::new(clock, wire.max_error_ticks, invert));
    let mut motor = WindowStepper::new(e, options.smooth)?;
    let mut xyz = Trajectory::new(1024)?;
    let mut queues = [Trajectory::new(1024)?, Trajectory::new(1024)?];
    let mut gains = Vec::new();
    let mut bound = None;
    let mut scratch = Vec::new();
    let mut from = 0.0_f64;
    for event in events {
        match event {
            Event::Profile {
                at,
                queue,
                movement,
            } => {
                xyz.append_profile(at, &movement)?;
                append_nominal(&mut queues[usize::from(queue)], at, &movement)?;
            }
            Event::Gain { at, advance } => {
                // Requests made at the same look-ahead boundary are applied
                // in source order; only the last affects the next phase.
                if let Some(last) = gains.last_mut().filter(|(t, _)| *t == at) {
                    last.1 = advance;
                } else {
                    gains.push((at, advance));
                }
            }
            Event::Bind { queue, anchor } => {
                bound = queue.map(usize::from);
                if bound.is_some() {
                    motor.rebase_after_flush(anchor)?;
                }
            }
            Event::Generation { at, retirement } => {
                let advance = Advance::new(options.smooth, &gains)?;
                let steps = motor.flush(bound.map(|i| &queues[i]), &advance, at, MAX_EVENTS)?;
                encoders[3].push(&steps)?;
                for phase in xyz.segments() {
                    let lo = from.max(phase.at);
                    let hi = at.min(phase.end());
                    if lo >= hi {
                        continue;
                    }
                    for (solver, encoder) in solvers.iter_mut().zip(&mut encoders[..3]) {
                        scratch.clear();
                        solver.crossing_events_into(phase, lo..hi, MAX_EVENTS, &mut scratch)?;
                        encoder.push(&scratch)?;
                    }
                }
                from = at;
                prune_prepared(&mut xyz, retirement)?;
                for queue in &mut queues {
                    prune_prepared(queue, retirement)?;
                }
            }
        }
    }
    let [a, b, z, e] = encoders;
    Ok(MotionTrace {
        lanes: [a.finish()?, b.finish()?, z.finish()?, e.finish()?],
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{extrusion::Settings, lookahead::Limits};

    #[test]
    fn rejects_bad_options_and_keeps_one_shared_e_stream() {
        let geometry = Extruder::new(Settings {
            nozzle: 0.4,
            filament: 1.75,
            max_area: 0.64,
            retraction_distance: 50.,
            retraction_velocity: 30.,
            retraction_accel: 300.,
            corner_velocity: 1.,
            min_simulated_temperature: 170.,
        })
        .unwrap();
        let options = Options {
            limits: Limits::new(250., 4000., 5., 0.5).unwrap(),
            z_velocity: 15.,
            z_accel: 200.,
            xyz_bounds: [[0., 300.]; 3],
            simulation_temp: [210.; 2],
            smooth: 0.04,
            initial_advance: 0.04,
        };
        let output = OutputOptions {
            step_distances: [40. / 3200., 40. / 3200., 8. / 3200., 22.17 / 3200.],
            invert: [false; 4],
            clock_hz: 50_000_000.,
            max_error_ticks: 0,
        };
        let run = |script, output| trace(script, [10., 10., 1.], options, [&geometry; 2], output);
        assert!(matches!(
            run(
                "M400",
                OutputOptions {
                    clock_hz: f64::NAN,
                    ..output
                }
            ),
            Err(TraceError::Options)
        ));
        assert!(run("G28\nM400", output).is_err());
        let script = "M83\nG1 X12 E.08\nSYNC_EXTRUDER_MOTION EXTRUDER=shared_extruder MOTION_QUEUE=\nG1 X14 E.07\nM400";
        let active_only = run("M83\nG1 X12 E.08\nM400", output).unwrap();
        let detached = run(script, output).unwrap();
        assert_eq!(active_only.lanes[3], detached.lanes[3]);
        assert!(!detached.lanes[3].is_empty());
        assert_ne!(active_only.lanes[0], detached.lanes[0]);
    }
}
