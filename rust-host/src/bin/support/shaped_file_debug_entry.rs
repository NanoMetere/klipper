// SPDX-License-Identifier: GPL-3.0-only
// The separate entry leaves the regular preview parser and code generation intact.
use super::*;
use klipper_host_protocol::{
    config,
    input_shaper::Shaper,
    shared_debug_plan::{trace_file_debug_shaped, trace_file_debug_shaped_parallel},
};
use std::collections::BTreeMap;

const HELP: &str = "Usage: klipper-shaped-file-debug-preview [--lane-workers 4] MODEL.json GCODE SHAPER.cfg\nFixed X/Y offline file-debug preview. SHAPER.cfg is an isolated [input_shaper] section. Default is serial; use four workers only with multiple CPUs. No device access.";

fn run() -> Result<(), String> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() == 1 && args[0] == "--help" {
        println!("{HELP}");
        return Ok(());
    }
    let (model, gcode, settings, parallel) = match &args[..] {
        [model, gcode, settings] => (model, gcode, settings, false),
        [option, count, model, gcode, settings] if option == "--lane-workers" && count == "4" => {
            (model, gcode, settings, true)
        }
        _ => return Err(HELP.into()),
    };
    let raw = read(Path::new(model), MAX_MODEL)?;
    let m: Model = serde_json::from_str(&raw).map_err(|e| format!("Model: {e}"))?;
    let script = read(Path::new(gcode), MAX_GCODE)?;
    let raw = read(Path::new(settings), MAX_MODEL)?;
    let fixed = config::parse(&BTreeMap::from([("shape.cfg".into(), raw)]), "shape.cfg")
        .map_err(|e| format!("Shaper config: {e:?}"))?;
    let shaper =
        Shaper::from_file_debug_config(&fixed).map_err(|e| format!("Shaper config: {e}"))?;
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
    let predict = if parallel {
        trace_file_debug_shaped_parallel
    } else {
        trace_file_debug_shaped
    };
    let t = predict(
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
        &shaper,
    )
    .map_err(|e| format!("Offline preview: {e:?}"))?;
    let p = Preview {
        schema: "klipper.shared-corexy.shaped-file-debug.v1",
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
