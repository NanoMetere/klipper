// SPDX-License-Identifier: GPL-3.0-only
use super::{
    render::{replay_error, solve_e},
    runtime_shaper::prepare_runtime,
    MotionTrace, Options, OutputOptions, TraceError,
};
use crate::{config, extrusion::Extruder, input_shaper::captured};

/// Render the same runtime-shaped trace with four independent lane workers.
/// Use only when multiple CPUs are available; the default API is serial.
pub fn trace_file_debug_runtime_shaper_parallel(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
    initial: &config::Config,
) -> Result<MotionTrace, TraceError> {
    let work = prepare_runtime(file, start, options, geometries, wire, initial)?;
    std::thread::scope(|scope| {
        let mut threads = Vec::with_capacity(3);
        let work = &work;
        for lane in work.lanes {
            let result = std::thread::Builder::new().spawn_scoped(scope, move || {
                let updates = captured::ReplayUpdates {
                    #[cfg(feature = "runtime-pickup-extended-preview")]
                    extended_pickup: false,
                    positions: &work.positions,
                    shapes: &work.updates,
                };
                captured::replay(
                    &work.shaper,
                    &work.captures,
                    [lane],
                    wire.clock_hz,
                    wire.max_error_ticks,
                    Some(updates),
                )
                .map(|[lane]| lane)
                .map_err(replay_error)
            });
            match result {
                Ok(handle) => threads.push(handle),
                Err(_) => break,
            }
        }
        let e = if threads.len() == 3 {
            solve_e(&work.events, options.smooth, wire)
        } else {
            Err(TraceError::Worker)
        };
        // Collect every started thread and preserve the serial A/B/Z/E order.
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
