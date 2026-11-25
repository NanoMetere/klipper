// SPDX-License-Identifier: GPL-3.0-only
// Keep runtime-shaper parsing separate from the fixed and unshaped tools.
use super::*;
use klipper_host_protocol::{config, shared_debug_plan::trace_file_debug_runtime_shaper};
use std::collections::BTreeMap;

const HELP: &str = "Usage: klipper-runtime-shaper-preview MODEL.json GCODE SHAPER.cfg\nOffline X/Y input-shaper updates. SHAPER.cfg is an isolated [input_shaper] section. No device access.";

fn run() -> Result<(), String> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() == 1 && args[0] == "--help" {
        println!("{HELP}");
        return Ok(());
    }
    let [model, gcode, settings] = &args[..] else {
        return Err(HELP.into());
    };
    let raw = read(Path::new(model), MAX_MODEL)?;
    let m: Model = serde_json::from_str(&raw).map_err(|e| format!("Model: {e}"))?;
    let script = read(Path::new(gcode), MAX_GCODE)?;
    let raw = read(Path::new(settings), MAX_MODEL)?;
    let fixed = config::parse(&BTreeMap::from([("shape.cfg".into(), raw)]), "shape.cfg")
        .map_err(|e| format!("Shaper config: {e:?}"))?;
    let [g0, g1] = m.extruders;
    let a = Extruder::new(g0.into()).map_err(|e| format!("extruder: {e:?}"))?;
    let b = Extruder::new(g1.into()).map_err(|e| format!("extruder1: {e:?}"))?;
    let limits = Limits::new(
        m.max_velocity_mm_s,
        m.max_accel_mm_s2,
        m.square_corner_velocity_mm_s,
        m.min_cruise_ratio,
    )
    .map_err(|e| format!("Motion limits: {e:?}"))?;
    let t = trace_file_debug_runtime_shaper(
        &script,
        m.virtual_start_mm,
        Options {
            limits,
            z_velocity: m.max_z_velocity_mm_s,
            z_accel: m.max_z_accel_mm_s2,
            xyz_bounds: m.xyz_bounds_mm,
            simulation_temp: m.simulation_temperatures_c,
            smooth: m.smooth_time_s,
            initial_advance: m.initial_pressure_advance,
        },
        [&a, &b],
        OutputOptions {
            step_distances: m.step_distances_mm,
            invert: m.invert_directions,
            clock_hz: m.clock_hz,
            max_error_ticks: m.max_error_ticks,
        },
        &fixed,
    )
    .map_err(|e| format!("Offline preview: {e:?}"))?;
    let p = Preview {
        schema: "klipper.shared-corexy.runtime-shaper.v1",
        clock_hz: m.clock_hz,
        lane_order: ["corexy_a", "corexy_b", "z", "shared_e"],
        lanes: t
            .lanes
            .map(|lane| lane.into_iter().map(Record::from).collect()),
    };
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    serde_json::to_writer(&mut stdout, &p).map_err(|e| e.to_string())?;
    stdout
        .write_all(b"\n")
        .and_then(|()| stdout.flush())
        .map_err(|e| e.to_string())
}

pub(crate) fn entry() {
    if let Err(err) = run() {
        eprintln!("{err}");
        std::process::exit(2);
    }
}
