// SPDX-License-Identifier: GPL-3.0-only
use super::*;
#[cfg(feature = "runtime-pickup-pressure-preview")]
use klipper_host_protocol::shared_debug_plan::trace_file_debug_runtime_pickup_pressure;
#[cfg(feature = "runtime-pickup-extended-preview")]
use klipper_host_protocol::shared_debug_plan::{
    trace_file_debug_runtime_pickup_extended, trace_file_debug_runtime_pickup_extended_parallel,
};
use klipper_host_protocol::{
    bed_mesh::profile_bank::Bank, config,
    shared_debug_plan::trace_file_debug_runtime_pickup_mesh_bank,
};
use std::collections::BTreeMap;

const HELP: &str = "Usage: klipper-runtime-pickup-mesh-bank-preview MODEL.json GCODE SHAPER.cfg MESH.cfg PROFILE_A PROFILE_B [PROFILE ...]\nOffline shared pickup with 2–8 saved bed meshes, pressure advance, and X/Y input shaping. No probing, configuration writes, or device access.";

fn run() -> Result<(), String> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() == 1 && args[0] == "--help" {
        println!("{HELP}");
        #[cfg(feature = "runtime-pickup-extended-preview")]
        {
            println!("Use --extended-pickup for the extended offline motion preview (includes dynamic pressure).");
            println!("Use --extended-pickup-parallel for the same preview with optional parallel lanes on long jobs.");
        }
        #[cfg(feature = "runtime-pickup-pressure-preview")]
        println!(
            "Optional --dynamic-pressure before MODEL.json enables bounded SMOOTH_TIME updates."
        );
        return Ok(());
    }
    let args = &args[..];
    #[cfg(feature = "runtime-pickup-extended-preview")]
    let (extended_pickup, parallel, args) = match args.first() {
        Some(opt) if opt == "--extended-pickup" => (true, false, &args[1..]),
        Some(opt) if opt == "--extended-pickup-parallel" => (true, true, &args[1..]),
        _ => (false, false, args),
    };
    #[cfg(feature = "runtime-pickup-pressure-preview")]
    let (dynamic_pressure, args) = if args.first().is_some_and(|x| x == "--dynamic-pressure") {
        (true, &args[1..])
    } else {
        (false, args)
    };
    let [model, gcode, settings, mesh_path, names @ ..] = args else {
        return Err(HELP.into());
    };
    if !(2..=8).contains(&names.len()) {
        return Err(HELP.into());
    }
    let names = names
        .iter()
        .map(|s| s.to_str().ok_or("Invalid mesh profile name"))
        .collect::<Result<Vec<_>, _>>()?;
    let mesh = read(Path::new(mesh_path), MAX_GCODE)?;
    let bank = Bank::load(&mesh, &names).map_err(|e| format!("Mesh profiles: {e:?}"))?;
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
    let trace = trace_file_debug_runtime_pickup_mesh_bank;
    #[cfg(feature = "runtime-pickup-pressure-preview")]
    let trace = if dynamic_pressure {
        trace_file_debug_runtime_pickup_pressure
    } else {
        trace
    };
    #[cfg(feature = "runtime-pickup-extended-preview")]
    let trace = if parallel {
        trace_file_debug_runtime_pickup_extended_parallel
    } else if extended_pickup {
        trace_file_debug_runtime_pickup_extended
    } else {
        trace
    };
    let t = trace(
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
        &bank,
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
