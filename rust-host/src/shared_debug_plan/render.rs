// SPDX-License-Identifier: GPL-3.0-only
//! Offline typed composition of the bounded planner and step encoders.
//! Output is a trace only, with no connection to an MCU or printer.

#[cfg(feature = "file-debug-preview")]
use super::plan_file_debug;
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
    /// A file-debug worker could not be started or joined.
    #[cfg(feature = "file-debug-preview")]
    Worker,
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
    render(events, start, options.smooth, wire)
}

/// Independently predict the bounded pinned stock-file debug output.
/// The file includes its checked virtual-start header and original comment
/// bytes. This is an offline simulation, not virtual SD or live timing.
#[cfg(feature = "file-debug-preview")]
pub fn trace_file_debug(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
) -> Result<MotionTrace, TraceError> {
    if !(1.0..=1e9).contains(&wire.clock_hz) || wire.max_error_ticks > 200_000 {
        return Err(TraceError::Options);
    }
    let events = plan_file_debug(file, start, options, geometries)?;
    render(events, start, options.smooth, wire)
}

/// File-debug preview using four independent lanes. Use only when multiple
/// CPU cores are available; the ordinary preview remains serial.
#[cfg(feature = "file-debug-preview")]
pub fn trace_file_debug_parallel(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
) -> Result<MotionTrace, TraceError> {
    if !(1.0..=1e9).contains(&wire.clock_hz) || wire.max_error_ticks > 200_000 {
        return Err(TraceError::Options);
    }
    let events = plan_file_debug(file, start, options, geometries)?;
    let events = &events;
    std::thread::scope(|scope| {
        let mut handles = Vec::with_capacity(3);
        for (lane, axis) in [Axis::CoreXYPlus, Axis::CoreXYMinus, Axis::Z]
            .into_iter()
            .enumerate()
        {
            match std::thread::Builder::new()
                .spawn_scoped(scope, move || solve_xyz(events, start, wire, lane, axis))
            {
                Ok(handle) => handles.push(handle),
                Err(_) => break,
            }
        }
        let extruder = if handles.len() == 3 {
            solve_e(events, options.smooth, wire)
        } else {
            Err(TraceError::Worker)
        };
        // Always join all spawned threads, even if another lane failed.
        let mut results = std::array::from_fn(|_| Err(TraceError::Worker));
        for (result, worker) in results[..3].iter_mut().zip(handles) {
            *result = worker.join().unwrap_or(Err(TraceError::Worker));
        }
        results[3] = extruder;
        let [a, b, z, e] = results;
        Ok(MotionTrace {
            lanes: [a?, b?, z?, e?],
        })
    })
}

/// Predict bounded file-debug output with fixed X/Y input shaping.
#[cfg(feature = "shaped-file-debug-preview")]
pub fn trace_file_debug_shaped(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
    shaper: &crate::input_shaper::Shaper,
) -> Result<MotionTrace, TraceError> {
    use crate::input_shaper::captured;
    let (events, captures, lanes) = shaped_plan(
        file,
        start,
        options,
        geometries,
        wire,
        shaper,
        #[cfg(feature = "runtime-shaper-preview")]
        None,
    )?;
    let [a, b, z] = captured::replay(
        shaper,
        &captures,
        lanes,
        wire.clock_hz,
        wire.max_error_ticks,
        #[cfg(feature = "runtime-shaper-preview")]
        None,
    )
    .map_err(replay_error)?;
    let e = solve_e(&events, options.smooth, wire)?;
    Ok(MotionTrace {
        lanes: [a, b, z, e],
    })
}

#[cfg(feature = "shaped-file-debug-preview")]
type ShapedPlan = (
    Vec<Event>,
    Vec<crate::input_shaper::captured::Capture>,
    [(Axis, f64, bool); 3],
);

#[cfg(feature = "runtime-shaper-preview")]
pub(super) struct ShapeCaptureSchedule<'a> {
    #[cfg(feature = "runtime-mesh-preview")]
    pub(super) mesh: Option<(&'a str, &'a crate::bed_mesh::Transform)>,
    pub(super) delays: &'a [(usize, f64)],
    pub(super) positions: &'a mut Vec<(usize, usize)>,
}

#[cfg(feature = "shaped-file-debug-preview")]
#[inline(always)]
pub(super) fn shaped_plan(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
    shaper: &crate::input_shaper::Shaper,
    #[cfg(feature = "runtime-shaper-preview")] updates: Option<ShapeCaptureSchedule<'_>>,
) -> Result<ShapedPlan, TraceError> {
    use crate::{input_shaper::captured::Capture, trajectory::Trapezoid};
    if !(1.0..=1e9).contains(&wire.clock_hz) || wire.max_error_ticks > 200_000 {
        return Err(TraceError::Options);
    }
    if shaper.is_enabled_for(Axis::Z) || !shaper.is_enabled_for(Axis::CoreXYPlus) {
        return Err(TraceError::Options);
    }
    let axes = [Axis::CoreXYPlus, Axis::CoreXYMinus, Axis::Z];
    let delay = axes.iter().fold(0.0_f64, |delay, &lane| {
        let (pre, post) = shaper.window(lane);
        delay.max(pre).max(post)
    });
    #[cfg(feature = "runtime-shaper-preview")]
    let mut positions = Vec::new();
    #[cfg(feature = "runtime-shaper-preview")]
    let (events, mut output_positions) = if let Some(ShapeCaptureSchedule {
        #[cfg(feature = "runtime-mesh-preview")]
        mesh,
        delays,
        positions: output_positions,
    }) = updates
    {
        let events = super::plan_file_debug_with_updates(
            file,
            start,
            options,
            geometries,
            delay,
            Some(super::RuntimePlan {
                #[cfg(feature = "runtime-mesh-preview")]
                mesh,
                updates: delays,
                positions: &mut positions,
            }),
        )?;
        (events, Some(output_positions))
    } else {
        (
            super::plan_file_debug_with_delay(file, start, options, geometries, delay)?,
            None,
        )
    };
    #[cfg(not(feature = "runtime-shaper-preview"))]
    let events = super::plan_file_debug_with_delay(file, start, options, geometries, delay)?;
    let mut captures = vec![Capture::Bind(start)];
    #[cfg(feature = "runtime-shaper-preview")]
    let mut positions = positions.into_iter().peekable();
    #[cfg_attr(
        not(feature = "runtime-shaper-preview"),
        allow(clippy::unused_enumerate_index)
    )]
    for (_index, event) in events.iter().enumerate() {
        #[cfg(feature = "runtime-shaper-preview")]
        if let Some(output) = output_positions.as_deref_mut() {
            while positions.peek().is_some_and(|p| p.0 == _index) {
                let (_, which) = positions.next().ok_or(TraceError::Options)?;
                output.push((captures.len(), which));
            }
        }
        match event {
            Event::Profile {
                at, movement: p, ..
            } if p.kinematic => {
                let cruise = if (-1e-12..0.0).contains(&p.cruise_t) {
                    0.
                } else {
                    p.cruise_t
                };
                captures.push(Capture::Profile(Trapezoid {
                    at: *at,
                    phase_sec: [p.accel_t, cruise, p.decel_t],
                    start: p.start[..3].try_into().unwrap(),
                    direction: p.direction[..3].try_into().unwrap(),
                    start_velocity: p.start_v,
                    cruise_velocity: p.cruise_v,
                    accel: p.accel,
                }));
            }
            Event::DebugTransmit(at) => captures.push(Capture::Transmit(*at)),
            Event::Generation { at, retirement } => {
                captures.push(Capture::Generate(*at));
                captures.push(Capture::Retire(*retirement));
            }
            _ => {}
        }
    }
    let lanes: [_; 3] = std::array::from_fn(|i| (axes[i], wire.step_distances[i], wire.invert[i]));
    Ok((events, captures, lanes))
}

#[cfg(feature = "shaped-file-debug-preview")]
pub(super) fn replay_error(e: crate::input_shaper::captured::ReplayError) -> TraceError {
    use crate::input_shaper::captured::ReplayError;
    match e {
        ReplayError::Options => TraceError::Options,
        ReplayError::Trajectory(e) => TraceError::Trajectory(e),
        ReplayError::Solver(e) => TraceError::Steps(e),
        ReplayError::Clock(e) => TraceError::Clock(e),
    }
}

#[cfg(feature = "file-debug-preview")]
fn solve_xyz(
    events: &[Event],
    start: [f64; 3],
    wire: OutputOptions,
    lane: usize,
    axis: Axis,
) -> Result<Vec<Command>, TraceError> {
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.0,
        hz: wire.clock_hz,
    };
    let mut xyz = Trajectory::new(1024)?;
    let mut solver = Solver::new(axis, wire.step_distances[lane], start)?;
    let mut encoder = TraceEncoder::new(clock, wire.max_error_ticks, wire.invert[lane]);
    let mut from = 0.0_f64;
    let mut scratch = Vec::new();
    for event in events {
        match *event {
            Event::Profile { at, movement, .. } => xyz.append_profile(at, &movement)?,
            Event::DebugTransmit(at) => {
                encoder.flush_before(crate::step_clock::captured_cutoff(at, clock.hz)?)?
            }
            Event::Generation { at, retirement } => {
                for phase in xyz.segments() {
                    let lo = from.max(phase.at);
                    let hi = at.min(phase.end());
                    if lo < hi {
                        scratch.clear();
                        solver.crossing_events_into(phase, lo..hi, MAX_EVENTS, &mut scratch)?;
                        encoder.push(&scratch)?;
                    }
                }
                from = at;
                prune_prepared(&mut xyz, retirement)?;
            }
            _ => {}
        }
    }
    Ok(encoder.finish()?)
}

#[cfg(feature = "file-debug-preview")]
pub(super) fn solve_e(
    events: &[Event],
    smooth: f64,
    wire: OutputOptions,
) -> Result<Vec<Command>, TraceError> {
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.0,
        hz: wire.clock_hz,
    };
    let mut e = WindowStepper::new(wire.step_distances[3], smooth)?;
    let mut encoder = TraceEncoder::new(clock, wire.max_error_ticks, wire.invert[3]);
    let mut queues = [Trajectory::new(1024)?, Trajectory::new(1024)?];
    let mut gains = Vec::new();
    let mut bound = None;
    for event in events {
        match *event {
            Event::Profile {
                at,
                movement,
                queue,
            } => append_nominal(&mut queues[usize::from(queue)], at, &movement)?,
            Event::Bind { queue, anchor } => {
                bound = queue.map(usize::from);
                if bound.is_some() {
                    e.rebase_after_flush(anchor)?;
                }
            }
            Event::Gain { at, advance } => {
                if let Some(last) = gains.last_mut().filter(|(t, _)| *t == at) {
                    last.1 = advance;
                } else {
                    gains.push((at, advance));
                }
            }
            Event::DebugTransmit(at) => {
                encoder.flush_before(crate::step_clock::captured_cutoff(at, clock.hz)?)?
            }
            Event::Generation { at, retirement } => {
                let advance = Advance::new(smooth, &gains)?;
                let steps = e.flush(bound.map(|i| &queues[i]), &advance, at, MAX_EVENTS)?;
                encoder.push(&steps)?;
                for queue in &mut queues {
                    prune_prepared(queue, retirement)?;
                }
            }
        }
    }
    Ok(encoder.finish()?)
}

#[cfg_attr(not(feature = "file-debug-preview"), inline(always))]
fn render(
    events: Vec<Event>,
    start: [f64; 3],
    smooth: f64,
    wire: OutputOptions,
) -> Result<MotionTrace, TraceError> {
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
    let mut motor = WindowStepper::new(e, smooth)?;
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
            #[cfg(feature = "file-debug-preview")]
            Event::DebugTransmit(at) => {
                let cutoff = crate::step_clock::captured_cutoff(at, clock.hz)?;
                for encoder in &mut encoders {
                    encoder.flush_before(cutoff)?;
                }
            }
            Event::Generation { at, retirement } => {
                let advance = Advance::new(smooth, &gains)?;
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

        #[cfg(feature = "file-debug-preview")]
        {
            let debug = |script: &str| {
                trace_file_debug(script, [10., 10., 1.], options, [&geometry; 2], output)
            };
            let parallel = |script: &str| {
                trace_file_debug_parallel(script, [10., 10., 1.], options, [&geometry; 2], output)
            };
            let header = "SET_KINEMATIC_POSITION X=10 Y=10 Z=1";
            let valid = format!("{header}\nM83\nG1 X12 Y11 Z1.1 E.08 F600\nM400\n");
            assert_eq!(parallel(&valid).unwrap(), debug(&valid).unwrap());
            assert!(debug(&format!("{header} ; virtual origin\nM400\n")).is_ok());
            for invalid in [
                "M400\n".into(),
                "SET_KINEMATIC_POSITION X=10 Y=10 Z=2\nM400\n".into(),
                format!("{header} HOMING_AXES=XYZ\nM400\n"),
                format!("{header}\n{header}\nM400\n"),
                format!("{header}\nM400"),
                format!("{header}\r\nM400\r\n"),
                format!("{header}\n; 보호\nM400\n"),
                format!("{header}\n;{}\nM400\n", " ".repeat(4096)),
                format!("{header}\n{}M400\n", ";\n".repeat(511)),
                format!("{header}\n{}M400\n", ";0123456789abcdef\n".repeat(4096)),
            ] {
                assert!(matches!(
                    debug(&invalid),
                    Err(TraceError::Plan(Error::Inputs))
                ));
                assert!(matches!(
                    parallel(&invalid),
                    Err(TraceError::Plan(Error::Inputs))
                ));
            }
            // Enabling the separate API must not relax the original preview.
            assert!(run(&format!("{}M400\n", ";\n".repeat(128)), output).is_err());
        }
    }
}
