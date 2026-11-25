// SPDX-License-Identifier: GPL-3.0-only
use super::{
    render::{replay_error, shaped_plan, solve_e},
    MotionTrace, Options, OutputOptions, TraceError,
};
use crate::{
    extrusion::Extruder,
    input_shaper::{captured, Shaper},
};

/// Predict the same ordered shaped trace with four independent lane workers.
/// Use only when multiple CPUs are available. The default shaped trace is serial.
pub fn trace_file_debug_shaped_parallel(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
    shaper: &Shaper,
) -> Result<MotionTrace, TraceError> {
    let (events, captures, lanes) = shaped_plan(file, start, options, geometries, wire, shaper)?;
    std::thread::scope(|scope| {
        let events = &events;
        let captures = &captures;
        let mut threads = Vec::with_capacity(3);
        for lane in lanes {
            match std::thread::Builder::new().spawn_scoped(scope, move || {
                captured::replay(
                    shaper,
                    captures,
                    [lane],
                    wire.clock_hz,
                    wire.max_error_ticks,
                )
                .map(|[lane]| lane)
                .map_err(replay_error)
            }) {
                Ok(handle) => threads.push(handle),
                Err(_) => break,
            }
        }
        let e = if threads.len() == 3 {
            solve_e(events, options.smooth, wire)
        } else {
            Err(TraceError::Worker)
        };
        // Collect every started worker before propagating any lane error.
        let mut xyz = std::array::from_fn(|_| Err(TraceError::Worker));
        for (lane, worker) in xyz.iter_mut().zip(threads) {
            *lane = worker.join().unwrap_or(Err(TraceError::Worker));
        }
        let [a, b, z] = xyz;
        Ok(MotionTrace {
            lanes: [a?, b?, z?, e?],
        })
    })
}
