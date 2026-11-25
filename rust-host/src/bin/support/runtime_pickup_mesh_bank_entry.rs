// SPDX-License-Identifier: GPL-3.0-only
use super::*;
#[cfg(feature = "runtime-pickup-pressure-preview")]
use klipper_host_protocol::shared_debug_plan::trace_file_debug_runtime_pickup_pressure;
#[cfg(feature = "runtime-pickup-arcs-preview")]
use klipper_host_protocol::shared_debug_plan::{trace_file_debug_runtime_pickup_arcs, PickupArcs};
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
        #[cfg(feature = "runtime-pickup-arcs-preview")]
        println!("Add --arc-resolution=MM after an extended pickup flag to opt in to G2/G3 arcs.");
        #[cfg(feature = "runtime-pickup-fast-json")]
        println!(
            "Add --fast-json after an extended option to select equivalent buffered JSON output."
        );
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
    #[cfg(feature = "runtime-pickup-arcs-preview")]
    let (arc_resolution, args) = match args
        .first()
        .and_then(|s| s.to_str())
        .and_then(|s| s.strip_prefix("--arc-resolution="))
    {
        Some(raw) if extended_pickup => {
            let resolution: f64 = raw.parse().map_err(|_| "Invalid arc resolution")?;
            if !(0.25..=5.).contains(&resolution) {
                return Err("Arc resolution must be 0.25–5 mm".into());
            }
            (Some(resolution), &args[1..])
        }
        Some(_) => return Err("--arc-resolution requires an extended pickup option".into()),
        None => (None, args),
    };
    #[cfg(feature = "runtime-pickup-fast-json")]
    let (fast_json, args) = match args.first() {
        Some(opt) if opt == "--fast-json" && extended_pickup => (true, &args[1..]),
        Some(opt) if opt == "--fast-json" => {
            return Err("--fast-json requires an extended pickup option".into())
        }
        _ => (false, args),
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
    #[cfg(feature = "runtime-pickup-arcs-preview")]
    let trace = |script: &str, start, options, geometries, wire, fixed, bank| {
        if let Some(resolution_mm) = arc_resolution {
            trace_file_debug_runtime_pickup_arcs(
                script,
                start,
                options,
                geometries,
                wire,
                fixed,
                bank,
                PickupArcs {
                    resolution_mm,
                    parallel,
                },
            )
        } else {
            trace(script, start, options, geometries, wire, fixed, bank)
        }
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
    #[cfg(feature = "runtime-pickup-fast-json")]
    if fast_json {
        let mut out = io::BufWriter::new(io::stdout().lock());
        write_direct(&mut out, m.clock_hz, &t.lanes).map_err(|e| e.to_string())?;
        return out.flush().map_err(|e| e.to_string());
    }
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

/// This changes only the writer, not the wire format or motion data.
#[cfg(feature = "runtime-pickup-fast-json")]
fn write_direct(w: &mut impl Write, hz: f64, lanes: &[Vec<Command>; 4]) -> io::Result<()> {
    w.write_all(br#"{"schema":"klipper.shared-corexy.runtime-shaper.v1","clock_hz":"#)?;
    serde_json::to_writer(&mut *w, &hz).map_err(io::Error::other)?;
    w.write_all(br#","lane_order":["corexy_a","corexy_b","z","shared_e"],"lanes":["#)?;
    let mut row = Vec::with_capacity(128);
    let mut integer = itoa::Buffer::new();
    for (lane_index, lane) in lanes.iter().enumerate() {
        w.write_all(if lane_index == 0 { b"[" } else { b",[" })?;
        for (index, command) in lane.iter().enumerate() {
            row.clear();
            if index > 0 {
                row.push(b',');
            }
            match command {
                Command::SetDirection(true) => {
                    row.extend_from_slice(br#"{"kind":"direction","positive":true}"#)
                }
                Command::SetDirection(false) => {
                    row.extend_from_slice(br#"{"kind":"direction","positive":false}"#)
                }
                Command::Steps(b) => {
                    row.extend_from_slice(br#"{"kind":"steps","interval":"#);
                    row.extend_from_slice(integer.format(b.interval).as_bytes());
                    row.extend_from_slice(br#","count":"#);
                    row.extend_from_slice(integer.format(b.count).as_bytes());
                    row.extend_from_slice(br#","add":"#);
                    row.extend_from_slice(integer.format(b.add).as_bytes());
                    row.extend_from_slice(br#","last_clock":"#);
                    row.extend_from_slice(integer.format(b.last_clock).as_bytes());
                    row.push(b'}');
                }
            }
            w.write_all(&row)?;
        }
        w.write_all(b"]")?;
    }
    w.write_all(b"]}\n")
}

#[cfg(all(test, feature = "runtime-pickup-fast-json"))]
#[test]
fn fast_output_matches_serde_with_empty_lanes_and_decimal_boundaries() {
    use klipper_host_protocol::step_compress::Block;
    let lanes = [
        vec![Command::SetDirection(true), Command::SetDirection(false)],
        vec![Command::Steps(Block {
            interval: u32::MAX,
            count: u16::MAX,
            add: i16::MIN,
            last_clock: u64::MAX,
        })],
        vec![],
        vec![Command::Steps(Block {
            interval: 0,
            count: 1,
            add: i16::MAX,
            last_clock: 1,
        })],
    ];
    for hz in [1., 50_000_000., 1.234567e-13] {
        let mut direct = Vec::new();
        write_direct(&mut direct, hz, &lanes).unwrap();
        let mut expected = serde_json::to_vec(&Preview {
            schema: "klipper.shared-corexy.runtime-shaper.v1",
            clock_hz: hz,
            lane_order: ["corexy_a", "corexy_b", "z", "shared_e"],
            lanes: lanes
                .each_ref()
                .map(|lane| lane.iter().copied().map(Record::from).collect()),
        })
        .unwrap();
        expected.push(b'\n');
        assert_eq!(direct, expected);
    }
}
