// SPDX-License-Identifier: GPL-3.0-only
//! Standalone offline trace preview. No serial transport or MCU output.

use klipper_host_protocol::{
    extrusion::{Extruder, Settings},
    lookahead::Limits,
    shared_debug_plan::{trace, Options, OutputOptions},
    step_clock::Command,
};
use serde::{Deserialize, Serialize};
use std::{
    env,
    fs::{self, File},
    io::{self, Read, Write},
    path::Path,
};

const HELP: &str = "Usage: klipper-shared-preview MODEL.json GCODE\nRender bounded offline CoreXY/shared-E traces; never controls a printer.";
const MAX_MODEL: u64 = 16_384;
const MAX_GCODE: u64 = 65_536;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Model {
    virtual_start_mm: [f64; 3],
    xyz_bounds_mm: [[f64; 2]; 3],
    max_velocity_mm_s: f64,
    max_accel_mm_s2: f64,
    square_corner_velocity_mm_s: f64,
    min_cruise_ratio: f64,
    max_z_velocity_mm_s: f64,
    max_z_accel_mm_s2: f64,
    simulation_temperatures_c: [f64; 2],
    extruders: [Geometry; 2],
    smooth_time_s: f64,
    initial_pressure_advance: f64,
    step_distances_mm: [f64; 4],
    invert_directions: [bool; 4],
    clock_hz: f64,
    max_error_ticks: u32,
}

#[derive(Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct Geometry {
    nozzle_mm: f64,
    filament_mm: f64,
    max_area_mm2: f64,
    max_retraction_mm: f64,
    max_retraction_velocity_mm_s: f64,
    max_retraction_accel_mm_s2: f64,
    corner_velocity_mm_s: f64,
    min_simulated_temperature_c: f64,
}

impl From<Geometry> for Settings {
    fn from(g: Geometry) -> Self {
        Self {
            nozzle: g.nozzle_mm,
            filament: g.filament_mm,
            max_area: g.max_area_mm2,
            retraction_distance: g.max_retraction_mm,
            retraction_velocity: g.max_retraction_velocity_mm_s,
            retraction_accel: g.max_retraction_accel_mm_s2,
            corner_velocity: g.corner_velocity_mm_s,
            min_simulated_temperature: g.min_simulated_temperature_c,
        }
    }
}

#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Record {
    Direction {
        positive: bool,
    },
    Steps {
        interval: u32,
        count: u16,
        add: i16,
        last_clock: u64,
    },
}

impl From<Command> for Record {
    fn from(value: Command) -> Self {
        match value {
            Command::SetDirection(positive) => Self::Direction { positive },
            Command::Steps(b) => Self::Steps {
                interval: b.interval,
                count: b.count,
                add: b.add,
                last_clock: b.last_clock,
            },
        }
    }
}

#[derive(Serialize)]
struct Preview {
    schema: &'static str,
    clock_hz: f64,
    lane_order: [&'static str; 4],
    lanes: [Vec<Record>; 4],
}

fn read(path: &Path, cap: u64) -> Result<String, String> {
    let mut data = String::new();
    let metadata = fs::symlink_metadata(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if !metadata.is_file() {
        return Err(format!(
            "{} must be an ordinary, non-symlink file",
            path.display()
        ));
    }
    let file = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if !file.metadata().map_err(|e| e.to_string())?.is_file() {
        return Err(format!("{} must be an ordinary file", path.display()));
    }
    file.take(cap + 1)
        .read_to_string(&mut data)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if data.len() as u64 > cap {
        return Err(format!("{} exceeds {cap} bytes", path.display()));
    }
    Ok(data)
}

#[inline(always)]
fn run<const FILE_DEBUG: bool>() -> Result<(), String> {
    let help = if FILE_DEBUG {
        "Usage: klipper-file-debug-preview [--lane-workers 4] MODEL.json GCODE\nRender pinned offline file-debug timing; requires the checked virtual-start header. Default is serial; use four workers only with multiple CPU cores. No device access."
    } else {
        HELP
    };
    let schema = if FILE_DEBUG {
        "klipper.shared-corexy.file-debug.v1"
    } else {
        "klipper.shared-corexy.offline.v1"
    };
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() == 1 && args[0] == "--help" {
        println!("{help}");
        return Ok(());
    }
    let (model, gcode, _parallel) = match &args[..] {
        [model, gcode] => (model, gcode, false),
        [flag, workers, model, gcode]
            if FILE_DEBUG && flag == "--lane-workers" && workers == "4" =>
        {
            (model, gcode, true)
        }
        _ => return Err(help.into()),
    };
    let raw = read(Path::new(model), MAX_MODEL)?;
    let m: Model = serde_json::from_str(&raw).map_err(|e| format!("Model: {e}"))?;
    let script = read(Path::new(gcode), MAX_GCODE)?;
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
    #[cfg(feature = "file-debug-preview")]
    let predictor = match (FILE_DEBUG, _parallel) {
        (true, true) => klipper_host_protocol::shared_debug_plan::trace_file_debug_parallel,
        (true, false) => klipper_host_protocol::shared_debug_plan::trace_file_debug,
        (false, _) => trace,
    };
    #[cfg(not(feature = "file-debug-preview"))]
    let predictor = trace;
    let t = predictor(
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
    )
    .map_err(|e| format!("Offline preview: {e:?}"))?;
    let p = Preview {
        schema,
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

pub(crate) fn entry<const FILE_DEBUG: bool>() {
    if let Err(err) = run::<FILE_DEBUG>() {
        eprintln!("{err}");
        std::process::exit(2);
    }
}

// Also included as a module by the separate opt-in binary.
#[allow(dead_code)]
fn main() {
    entry::<false>();
}
