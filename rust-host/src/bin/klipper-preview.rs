// SPDX-License-Identifier: GPL-3.0-only
//! Offline config + G-code to JSON step command trace. Never opens a printer.

use klipper_host_protocol::{
    bed_mesh, config,
    gcode_macros::Registry,
    machine_preview::{mixed, tool_profiles, Machine},
    step_clock::{Clock, Command},
    thermal::{
        commands::{self, Source, Wait},
        profile as thermal_profiles,
    },
    virtual_sd::Job as SdJob,
};
use serde::{
    ser::{SerializeSeq, SerializeStruct},
    Serialize, Serializer,
};
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{self, BufWriter, Read, Write},
    path::{Component, Path},
};

const MAX_BYTES: usize = 8 * 1024 * 1024;

const HELP: &str =
    "Usage: klipper-preview [--expand-macros] [--virtual-sd] [--tool-temps T:C,...] [--initial-tool T] [--lane-workers N] [--mesh-profile NAME] [--mixed-plan] ROOT ENTRY.cfg SCRIPT.gcode HZ X Y Z [INCLUDE.cfg ...]\n\
    ROOT contains config files; ENTRY and INCLUDE paths must be relative to it.\n\
    List every included file needed, including matches for wildcard includes.\n\
    XYZ is the explicit virtual starting position in millimeters.\n\
    Prints JSON to stdout, never connects to a printer. Opt in to limited offline macro expansion.\n\
    --virtual-sd previews complete LF-delimited lines and reports the byte checkpoint.\n\
    --tool-temps supplies one modeled temperature per tool ID (e.g. 0:210,2:215).\n\
    --initial-tool defaults to 0. Bare T lines only switch offline lanes; no docking.\n\
    --lane-workers sets 1–8 motion workers for long multi-tool previews (default 1).\n\
    --mesh-profile uses a saved Klipper mesh for this offline preview.\n\
    --mixed-plan prints separate motion stages around stock heater commands; waits remain unresolved.
\
    A multi-tool mixed plan requires --initial-tool and every modeled tool temperature.";

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

struct Records<'a>(&'a [Command]);

impl Serialize for Records<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(Some(self.0.len()))?;
        for entry in self.0 {
            seq.serialize_element(&Record(entry))?;
        }
        seq.end()
    }
}

struct Record<'a>(&'a Command);

impl Serialize for Record<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self.0 {
            Command::SetDirection(positive) => {
                let mut row = serializer.serialize_struct("Direction", 2)?;
                row.serialize_field("kind", "direction")?;
                row.serialize_field("positive", positive)?;
                row.end()
            }
            Command::Steps(block) => {
                let mut row = serializer.serialize_struct("Steps", 5)?;
                row.serialize_field("add", &block.add)?;
                row.serialize_field("count", &block.count)?;
                row.serialize_field("interval", &block.interval)?;
                row.serialize_field("kind", "steps")?;
                row.serialize_field("last_clock", &block.last_clock)?;
                row.end()
            }
        }
    }
}

#[derive(Serialize)]
struct ToolOutput<'a> {
    commands: Records<'a>,
    extruder: &'a str,
    step_distance_mm: f64,
    tool: u8,
}

#[derive(Serialize)]
struct SdOutput {
    position: usize,
    size: usize,
}

#[derive(Serialize)]
struct PreviewOutput<'a> {
    clock_hz: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    extruder_lanes: Option<Vec<ToolOutput<'a>>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    mesh_profile: Option<&'a str>,
    mode: &'static str,
    motor_step_distances_mm: [f64; 3],
    motors: [Records<'a>; 3],
    planned_duration_seconds: f64,
    schema: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    tool_mode: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    virtual_sd: Option<SdOutput>,
    virtual_start_mm: [f64; 3],
    virtual_start_time: f64,
}

#[derive(Serialize)]
#[serde(tag = "stage", rename_all = "snake_case")]
enum Phase<'a> {
    Motion {
        first_line: usize,
        #[serde(skip_serializing_if = "Option::is_none")]
        tool: Option<u8>,
        motors: [Records<'a>; 3],
        extruder: Option<Records<'a>>,
        motion_seconds: f64,
    },
    Heater {
        line: usize,
        #[serde(skip_serializing_if = "Option::is_none")]
        active_tool: Option<u8>,
        targets_c: &'a [f64],
        wait: Option<WaitJson<'a>>,
    },
    Select {
        line: usize,
        from: u8,
        to: u8,
    },
}

#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum WaitJson<'a> {
    Controller {
        heater_index: usize,
        target_c: f64,
    },
    Sensor {
        source: SensorJson<'a>,
        minimum_c: Option<f64>,
        maximum_c: Option<f64>,
    },
}

#[derive(Serialize)]
#[serde(untagged)]
enum SensorJson<'a> {
    Heater { heater_index: usize },
    Named { name: &'a str },
}

#[derive(Serialize)]
struct MixedOutput<'a> {
    mode: &'static str,
    schema: u8,
    clock_hz: f64,
    final_position_mm: [f64; 4],
    #[serde(skip_serializing_if = "Option::is_none")]
    initial_tool: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    final_extruders_mm: Option<&'a [(u8, f64)]>,
    heater_names: Vec<&'a str>,
    phases: Vec<Phase<'a>>,
}

fn wait_json<'a>(outcome: &commands::Output<'a>) -> Option<WaitJson<'a>> {
    let condition = match outcome {
        commands::Output::Set { wait, .. } => *wait,
        commands::Output::Range(wait) => Some(*wait),
        _ => None,
    }?;
    Some(match condition {
        Wait::Controller { index, target } => WaitJson::Controller {
            heater_index: index,
            target_c: target,
        },
        Wait::Range {
            source,
            minimum,
            maximum,
        } => {
            let source = match source {
                Source::Heater(index) => SensorJson::Heater {
                    heater_index: index,
                },
                Source::Sensor(name) => SensorJson::Named { name },
            };
            WaitJson::Sensor {
                source,
                minimum_c: minimum.is_finite().then_some(minimum),
                maximum_c: maximum.is_finite().then_some(maximum),
            }
        }
    })
}

fn motion_phase(
    first_line: usize,
    tool: Option<u8>,
    trace: &klipper_host_protocol::machine_preview::Trace,
) -> Phase<'_> {
    Phase::Motion {
        first_line,
        tool,
        motors: std::array::from_fn(|i| Records(&trace.motors[i])),
        extruder: trace.extruder.as_ref().map(|v| Records(v)),
        motion_seconds: trace.duration,
    }
}

fn emit(out: &impl Serialize) -> Result<(), String> {
    let mut stdout = BufWriter::new(io::stdout().lock());
    serde_json::to_writer(&mut stdout, out).map_err(|e| e.to_string())?;
    stdout
        .write_all(b"\n")
        .and_then(|()| stdout.flush())
        .map_err(|e| e.to_string())
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
    let mut workers = None;
    let mut mesh_profile = None;
    let mut mixed_plan = false;
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
        } else if flag == "--lane-workers" && workers.is_none() {
            workers = Some(
                args.get(1)
                    .ok_or("missing worker count")?
                    .parse::<usize>()
                    .map_err(|_| "invalid worker count")?,
            );
            args = &args[1..];
        } else if flag == "--mixed-plan" && !mixed_plan {
            mixed_plan = true;
        } else if flag == "--mesh-profile" && mesh_profile.is_none() {
            mesh_profile = Some(args.get(1).ok_or("missing bed mesh profile name")?.as_str());
            args = &args[1..];
        } else {
            break;
        }
        args = &args[1..];
    }
    if args.len() < 7
        || args.len() - 6 > 1024
        || ((initial_tool.is_some() || workers.is_some()) && tool_temps.is_none())
    {
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
    let mut machine = Machine::from_config(&parsed)
        .and_then(|m| m.with_lane_workers(workers.unwrap_or(1)))
        .map_err(|e| format!("machine: {e:?}"))?;
    if let Some(name) = mesh_profile {
        let transform = bed_mesh::profile::load(&parsed, &files[entry], name)
            .map_err(|e| format!("mesh profile: {e:?}"))?;
        machine = machine.with_bed_mesh(transform);
    }
    if mixed_plan {
        if workers.is_some() {
            return Err("--mixed-plan uses one phase-local lane worker".into());
        }
        let catalog = thermal_profiles::Catalog::read(&parsed)
            .map_err(|e| format!("heater profile: {e:?}"))?;
        let bank = catalog
            .bank(false)
            .map_err(|e| format!("heater profile: {e:?}"))?;
        let multi = parsed.values("toolchanger").is_some()
            || tool_temps.as_ref().is_some_and(|temps| temps.len() > 1)
            || initial_tool.is_some();
        if multi {
            let initial = initial_tool.ok_or("multi-tool --mixed-plan requires --initial-tool")?;
            let temps = tool_temps
                .ok_or("multi-tool --mixed-plan requires one modeled temperature per tool")?;
            let profiles =
                tool_profiles::read(&parsed).map_err(|e| format!("tool profiles: {e:?}"))?;
            if profiles.len() != temps.len() {
                return Err("supply one modeled temperature for every configured tool".into());
            }
            let slots = profiles
                .iter()
                .map(|p| {
                    let temperature = temps
                        .get(&p.id)
                        .ok_or_else(|| format!("missing modeled temperature for T{}", p.id))?;
                    let heater = catalog
                        .extruder_index(&p.section)
                        .ok_or_else(|| format!("no stock heater role for {:?}", p.section))?;
                    let lane = p
                        .lane(*temperature, 0.)
                        .map_err(|e| format!("tool profile: {e:?}"))?;
                    Ok(mixed::ToolSlot { lane, heater })
                })
                .collect::<Result<Vec<_>, String>>()?;
            if !slots.iter().any(|t| t.lane.id == initial) {
                return Err(format!("initial tool T{initial} is not configured"));
            }
            let plan = machine
                .trace_mixed_tools_at_virtual_origin(
                    start,
                    &bank,
                    mixed::ToolInput {
                        script,
                        slots: &slots,
                        initial_tool: initial,
                        hz,
                        max_error: 0,
                        invert: [false; 3],
                    },
                )
                .map_err(|e| format!("mixed preview: {e:?}"))?;
            let phases = plan
                .phases
                .iter()
                .map(|p| match p {
                    mixed::ToolStage::Motion {
                        first_line,
                        tool,
                        trace,
                    } => motion_phase(*first_line, Some(*tool), trace),
                    mixed::ToolStage::Heater {
                        line,
                        active_tool,
                        outcome,
                        targets,
                    } => Phase::Heater {
                        line: *line,
                        active_tool: Some(*active_tool),
                        targets_c: targets,
                        wait: wait_json(outcome),
                    },
                    mixed::ToolStage::Select { line, from, to } => Phase::Select {
                        line: *line,
                        from: *from,
                        to: *to,
                    },
                })
                .collect();
            return emit(&MixedOutput {
                mode: "offline_mixed_plan",
                schema: 1,
                clock_hz: plan.hz,
                final_position_mm: plan.final_position,
                initial_tool: Some(plan.initial_tool),
                final_extruders_mm: Some(&plan.final_extruders),
                heater_names: catalog.names().collect(),
                phases,
            });
        }
        let profiles;
        let extruder = if let Some(dict) = tool_temps {
            if dict.len() != 1 || !dict.contains_key(&0) {
                return Err("--mixed-plan accepts only one modeled temperature, for T0".into());
            }
            profiles = tool_profiles::read(&parsed).map_err(|e| format!("tool profiles: {e:?}"))?;
            let primary = profiles
                .iter()
                .find(|p| p.id == 0)
                .ok_or("missing T0 extruder profile")?;
            Some(
                primary
                    .lane(dict[&0], 0.)
                    .map_err(|e| format!("tool profile: {e:?}"))?
                    .extruder,
            )
        } else {
            None
        };
        let plan = machine
            .trace_mixed_at_virtual_origin(
                start,
                &bank,
                mixed::Input {
                    script,
                    extruder,
                    hz,
                    max_error: 0,
                    invert: [false; 3],
                },
            )
            .map_err(|e| format!("mixed preview: {e:?}"))?;
        let phases = plan
            .phases
            .iter()
            .map(|phase| match phase {
                mixed::Stage::Motion { first_line, trace } => {
                    motion_phase(*first_line, None, trace)
                }
                mixed::Stage::Heater {
                    line,
                    outcome,
                    targets,
                } => Phase::Heater {
                    line: *line,
                    active_tool: None,
                    targets_c: targets,
                    wait: wait_json(outcome),
                },
            })
            .collect();
        return emit(&MixedOutput {
            mode: "offline_mixed_plan",
            schema: 1,
            clock_hz: plan.hz,
            final_position_mm: plan.final_position,
            initial_tool: None,
            final_extruders_mm: None,
            heater_names: catalog.names().collect(),
            phases,
        });
    }
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
        let tools = profiles.into_iter().zip(trace.tools).collect::<Vec<_>>();
        (trace.motors, trace.duration, Some(tools))
    } else {
        let trace = machine
            .trace(script, start, 2.0, clock, 0, [false; 3])
            .map_err(|e| format!("preview: {e:?}"))?;
        (trace.motors, trace.duration, None)
    };
    let (tool_mode, extruder_lanes) = if let Some(tool_commands) = &tools {
        let lanes = tool_commands
            .iter()
            .map(|(profile, (id, commands))| {
                debug_assert_eq!(profile.id, *id);
                ToolOutput {
                    commands: Records(commands),
                    extruder: &profile.section,
                    step_distance_mm: profile.step_distance,
                    tool: *id,
                }
            })
            .collect();
        (Some("lane_selection_without_docking"), Some(lanes))
    } else {
        (None, None)
    };
    let out = PreviewOutput {
        clock_hz: hz,
        extruder_lanes,
        mesh_profile,
        mode: "offline",
        motor_step_distances_mm: machine.step_distances(),
        motors: std::array::from_fn(|i| Records(&motors[i])),
        planned_duration_seconds: duration,
        schema: 1,
        tool_mode,
        virtual_sd: sd.map(|s| SdOutput {
            position: s.position,
            size: s.size,
        }),
        virtual_start_mm: start,
        virtual_start_time: 2.0,
    };
    emit(&out)
}

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
