// SPDX-License-Identifier: GPL-3.0-only
//! Offline config + G-code to JSON step command trace. Never opens a printer.

use klipper_host_protocol::{
    config,
    gcode_macros::Registry,
    machine_preview::{tool_profiles, Machine},
    step_clock::{Clock, Command},
    virtual_sd::Job as SdJob,
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
    "Usage: klipper-preview [--expand-macros] [--virtual-sd] [--tool-temps T:C,...] [--initial-tool T] ROOT ENTRY.cfg SCRIPT.gcode HZ X Y Z [INCLUDE.cfg ...]\n\
    ROOT contains config files; ENTRY and INCLUDE paths must be relative to it.\n\
    List every included file needed, including matches for wildcard includes.\n\
    XYZ is the explicit virtual starting position in millimeters.\n\
    Prints JSON to stdout, never connects to a printer. Opt in to limited offline macro expansion.\n\
    --virtual-sd previews complete LF-delimited lines and reports the byte checkpoint.\n\
    --tool-temps supplies one modeled temperature per tool ID (e.g. 0:210,2:215).\n\
    --initial-tool defaults to 0. Bare T lines only switch offline lanes; no docking.";

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

fn temperatures(text: &str) -> Result<BTreeMap<u8, f64>, String> {
    let mut out = BTreeMap::new();
    for item in text.split(',') {
        let (id, value) = item.split_once(':').ok_or("expected tool:temperature")?;
        let id = id.parse::<u8>().map_err(|_| "invalid tool ID")?;
        let value = value
            .parse::<f64>()
            .map_err(|_| "invalid modeled temperature")?;
        if !value.is_finite() || out.len() == 8 || out.insert(id, value).is_some() {
            return Err("duplicate, nonfinite, or too many tool temperatures".into());
        }
    }
    Ok(out)
}

fn records(input: Vec<Command>) -> Vec<Value> {
    input
        .into_iter()
        .map(|cmd| match cmd {
            Command::SetDirection(positive) => json!({"kind":"direction", "positive":positive}),
            Command::Steps(b) => json!({"kind":"steps","interval":b.interval,"count":b.count,
                                      "add":b.add,"last_clock":b.last_clock}),
        })
        .collect()
}

fn run() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.as_slice() == ["--help"] {
        println!("{HELP}");
        return Ok(());
    }
    let mut expand_macros = false;
    let mut virtual_sd = false;
    let mut tool_temps = None;
    let mut initial_tool = None;
    let mut args = args.as_slice();
    while let Some(flag) = args.first() {
        if flag == "--expand-macros" && !expand_macros {
            expand_macros = true;
        } else if flag == "--virtual-sd" && !virtual_sd {
            virtual_sd = true;
        } else if flag == "--tool-temps" && tool_temps.is_none() {
            tool_temps = Some(temperatures(
                args.get(1).ok_or("missing tool temperatures")?,
            )?);
            args = &args[1..];
        } else if flag == "--initial-tool" && initial_tool.is_none() {
            initial_tool = Some(
                args.get(1)
                    .ok_or("missing initial tool ID")?
                    .parse::<u8>()
                    .map_err(|_| "invalid initial tool ID")?,
            );
            args = &args[1..];
        } else {
            break;
        }
        args = &args[1..];
    }
    if args.len() < 7 || args.len() - 6 > 1024 || (initial_tool.is_some() && tool_temps.is_none()) {
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
    let sd = if virtual_sd {
        let mut job = SdJob::select(script.as_bytes());
        let parse = |e| format!("virtual SD: {e:?}");
        job.resume().map_err(parse)?;
        while let Some(line) = job.issue().map_err(parse)? {
            job.commit(line.token, None).map_err(parse)?;
        }
        Some(job.snapshot())
    } else {
        None
    };
    let script = if let Some(checkpoint) = sd {
        &script[..checkpoint.position]
    } else {
        &script
    };
    let expanded;
    let script = if expand_macros {
        expanded = Registry::from_config(&parsed)
            .and_then(|registry| registry.expand(script))
            .map_err(|e| format!("macro expansion: {e:?}"))?;
        expanded.as_str()
    } else {
        script
    };
    let machine = Machine::from_config(&parsed).map_err(|e| format!("machine: {e:?}"))?;
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.,
        hz,
    };
    let (motors, duration, tools) = if let Some(temps) = tool_temps {
        let profiles = tool_profiles::read(&parsed).map_err(|e| format!("tool profiles: {e:?}"))?;
        if profiles.len() != temps.len() {
            return Err("supply one modeled temperature for every configured tool".into());
        }
        let lanes = profiles
            .iter()
            .map(|p| {
                let t = temps
                    .get(&p.id)
                    .ok_or_else(|| format!("missing modeled temperature for T{}", p.id))?;
                p.lane(*t, 0.).map_err(|e| format!("tool profiles: {e:?}"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let trace = machine
            .trace_tool_gcode_at_virtual_origin(
                start,
                script,
                initial_tool.unwrap_or(0),
                &lanes,
                2.0,
                clock,
                0,
                [false; 3],
            )
            .map_err(|e| format!("preview: {e:?}"))?;
        let tools = profiles
            .iter()
            .zip(trace.tools)
            .map(|(p, (id, commands))| {
                debug_assert_eq!(p.id, id);
                json!({"tool":id, "extruder":p.section, "step_distance_mm":p.step_distance,
                   "commands":records(commands)})
            })
            .collect::<Vec<_>>();
        (trace.motors, trace.duration, Some(tools))
    } else {
        let trace = machine
            .trace(script, start, 2.0, clock, 0, [false; 3])
            .map_err(|e| format!("preview: {e:?}"))?;
        (trace.motors, trace.duration, None)
    };
    let mut out = json!({
        "schema":1, "mode":"offline", "virtual_start_mm":start, "virtual_start_time":2.0,
        "clock_hz":hz, "planned_duration_seconds":duration, "motors":motors.map(records),
        "motor_step_distances_mm":machine.step_distances(),
    });
    if let Some(tools) = tools {
        out["tool_mode"] = json!("lane_selection_without_docking");
        out["extruder_lanes"] = json!(tools);
    }
    if let Some(checkpoint) = sd {
        out["virtual_sd"] = json!({"position":checkpoint.position, "size":checkpoint.size});
    }
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
