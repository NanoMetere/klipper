// SPDX-License-Identifier: GPL-3.0-only
//! Offline config + G-code to JSON step command trace. Never opens a printer.

use klipper_host_protocol::{
    config,
    machine_preview::Machine,
    step_clock::{Clock, Command},
};
use serde_json::{json, Value};
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{self, Read, Write},
    path::{Component, Path},
};

const MAX_BYTES: usize = 8 * 1024 * 1024;

const HELP: &str =
    "Usage: klipper-preview ROOT ENTRY.cfg SCRIPT.gcode HZ X Y Z [INCLUDE.cfg ...]\n\
    ROOT contains config files; ENTRY and INCLUDE paths must be relative to it.\n\
    List every included file needed, including matches for wildcard includes.\n\
    XYZ is the explicit virtual starting position in millimeters.\n\
    Prints JSON to stdout. Does not connect to any printer or load macros.";

fn read(path: &Path, limit: usize) -> Result<String, String> {
    let status = fs::metadata(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if !status.is_file() || status.len() > limit as u64 {
        return Err(format!(
            "{} is not a regular text file within the remaining byte limit",
            path.display()
        ));
    }
    let file = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut text = String::new();
    file.take(limit as u64 + 1)
        .read_to_string(&mut text)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if text.len() > limit {
        return Err("input is too large".into());
    }
    Ok(text)
}

fn file_path(root: &Path, key: &str) -> Result<std::path::PathBuf, String> {
    let path = Path::new(key);
    if key.is_empty()
        || path
            .components()
            .any(|c| !matches!(c, Component::Normal(_)))
    {
        return Err(format!("invalid relative config path {key:?}"));
    }
    let resolved = root
        .join(path)
        .canonicalize()
        .map_err(|e| format!("{key}: {e}"))?;
    if !resolved.starts_with(root) {
        return Err(format!("config path outside ROOT: {key:?}"));
    }
    Ok(resolved)
}

fn run() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.as_slice() == ["--help"] {
        println!("{HELP}");
        return Ok(());
    }
    if args.len() < 7 || args.len() - 6 > 1024 {
        return Err(HELP.into());
    }
    let root = Path::new(&args[0])
        .canonicalize()
        .map_err(|e| format!("ROOT: {e}"))?;
    let entry = &args[1];
    let script = read(Path::new(&args[2]), MAX_BYTES)?;
    let number = |i: usize| {
        args[i]
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("invalid number: {:?}", args[i]))
    };
    let hz = number(3)?;
    if !(1.0..=1e12).contains(&hz) {
        return Err("clock rate is outside the offline range".into());
    }
    let start = [number(4)?, number(5)?, number(6)?];
    let mut files = BTreeMap::new();
    let mut bytes = MAX_BYTES;
    for key in std::iter::once(entry).chain(&args[7..]) {
        if files.contains_key(key) {
            return Err(format!("duplicate config file {key:?}"));
        }
        let text = read(&file_path(&root, key)?, bytes)?;
        bytes -= text.len();
        files.insert(key.clone(), text);
    }

    let parsed = config::parse(&files, entry).map_err(|e| format!("configuration: {e:?}"))?;
    let machine = Machine::from_config(&parsed).map_err(|e| format!("machine: {e:?}"))?;
    let trace = machine
        .trace(
            &script,
            start,
            2.0,
            Clock {
                last_encoded: 0,
                time_origin: 0.,
                hz,
            },
            0,
            [false; 3],
        )
        .map_err(|e| format!("preview: {e:?}"))?;
    let motors = trace.motors.map(|cmds| {
        cmds.into_iter()
            .map(|cmd| match cmd {
                Command::SetDirection(positive) => json!({"kind":"direction", "positive":positive}),
                Command::Steps(b) => json!({"kind":"steps","interval":b.interval,"count":b.count,
                                  "add":b.add,"last_clock":b.last_clock}),
            })
            .collect::<Vec<Value>>()
    });
    let out = json!({
        "schema":1, "mode":"offline", "virtual_start_mm":start, "virtual_start_time":2.0,
        "clock_hz":hz, "planned_duration_seconds":trace.duration, "motors":motors,
        "motor_step_distances_mm":machine.step_distances(),
    });
    let mut stdout = io::stdout().lock();
    serde_json::to_writer(&mut stdout, &out).map_err(|e| e.to_string())?;
    stdout.write_all(b"\n").map_err(|e| e.to_string())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
