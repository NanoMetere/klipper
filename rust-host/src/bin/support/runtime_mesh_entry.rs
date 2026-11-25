// SPDX-License-Identifier: GPL-3.0-only
// Use a separate entry for the opt-in offline saved-mesh reader.
use super::*;
use klipper_host_protocol::{config, shared_debug_plan::trace_file_debug_runtime_mesh};
use std::collections::BTreeMap;

const HELP: &str = "Usage: klipper-runtime-mesh-preview MODEL.json GCODE SHAPER.cfg MESH.cfg PROFILE\nOffline saved bed mesh and X/Y input-shaper updates. No probing or configuration writes. No device access.";

fn run() -> Result<(), String> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() == 1 && args[0] == "--help" {
        println!("{HELP}");
        return Ok(());
    }
    let [model, gcode, settings, mesh_path, mesh_name] = &args[..] else {
        return Err(HELP.into());
    };
    let mesh_name = mesh_name.to_str().ok_or("Invalid mesh profile name")?;
    let mesh = {
        let text = read(Path::new(mesh_path), MAX_GCODE)?;
        let loaded = config::parse(
            &BTreeMap::from([("mesh.cfg".into(), text.clone())]),
            "mesh.cfg",
        )
        .map_err(|e| format!("Mesh file: {e:?}"))?;
        let section = format!("bed_mesh {mesh_name}");
        if loaded.values("DEFAULT").is_some()
            || loaded.sections().any(|s| s != "bed_mesh" && s != section)
        {
            return Err("MESH.cfg must hold only one matching saved profile and [bed_mesh]".into());
        }
        klipper_host_protocol::bed_mesh::profile::load(&loaded, &text, mesh_name)
            .map_err(|e| format!("Mesh profile: {e:?}"))?
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
    let t = trace_file_debug_runtime_mesh(
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
        (mesh_name, &mesh),
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
