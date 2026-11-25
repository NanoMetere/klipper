// SPDX-License-Identifier: GPL-3.0-only
//! Run the demonstrator with a firmware dictionary but no physical device.

use klipper_host_protocol::{
    config,
    gcode_macros::Registry,
    homing::Counter,
    identify::Dictionary,
    machine_preview::Machine,
    simulated_printer::clock_watch::{Advance, ClockWatchedSimulation},
    simulated_printer::{Input, Simulation},
    step_wire::StepMessages,
    virtual_job::State,
};
use serde_json::json;
use std::{collections::BTreeMap, env, fs};

fn run() -> Result<(), String> {
    let mut args = env::args_os().skip(1);
    let path = args
        .next()
        .ok_or("usage: clock_watched_sim_replay FIRMWARE.dict")?;
    if args.next().is_some() {
        return Err("usage: clock_watched_sim_replay FIRMWARE.dict".into());
    }
    let text = fs::read(path).map_err(|e| e.to_string())?;
    let dict: Dictionary = serde_json::from_slice(&text).map_err(|e| e.to_string())?;
    let codec = StepMessages::from_dictionary(&dict).map_err(|e| format!("{e:?}"))?;
    let hz = dict
        .config
        .get("CLOCK_FREQ")
        .and_then(serde_json::Value::as_f64)
        .ok_or("missing clock rate")?;
    let cfg = config::parse(
        &BTreeMap::from([
            (
                "printer.cfg".into(),
                include_str!("preview/printer.cfg").into(),
            ),
            ("rails.cfg".into(), include_str!("preview/rails.cfg").into()),
        ]),
        "printer.cfg",
    )
    .map_err(|e| format!("{e:?}"))?;
    let machine = Machine::from_config(&cfg).map_err(|e| format!("{e:?}"))?;
    let macros = Registry::from_config(&cfg).map_err(|e| format!("{e:?}"))?;
    let start = [1.0, 1.0, 0.2];
    let mut homing = machine
        .virtual_homing(start)
        .map_err(|e| format!("{e:?}"))?;
    let sample = Some(Counter {
        start: 20,
        trigger: 16,
        halt: 16,
    });
    // Explicit fake snapshots; no physical homing or endstop reading.
    for (axis, coordinate) in start.into_iter().enumerate() {
        let token = homing
            .arm(axis as u8, coordinate, true)
            .map_err(|e| format!("{e:?}"))?;
        let motors = if axis < 2 {
            [sample, sample, None]
        } else {
            [None, None, sample]
        };
        homing
            .complete(token, motors, true)
            .map_err(|e| format!("{e:?}"))?;
    }
    let prepare = || {
        Simulation::prepare(
            &machine,
            &homing,
            Input {
                name: "path_macro_tail.gcode",
                script: include_str!("preview/path_macro_tail.gcode"),
                macros: Some(&macros),
                discovered: &codec,
                oids: &[3, 4, 5],
                extruder: None,
                hz,
            },
        )
        .map_err(|e| format!("{e:?}"))
    };
    let wire = |item: klipper_host_protocol::virtual_job::Issued| {
        (item.lane, item.command.required_tick, item.command.bytes)
    };
    let mut bare = prepare()?;
    bare.start(0).map_err(|e| format!("{e:?}"))?;
    let target = bare.snapshot().planned_end;
    let mut expected_output = Vec::new();
    for _ in 0..1000 {
        expected_output.extend(
            bare.advance(target, 256)
                .map_err(|e| format!("{e:?}"))?
                .into_iter()
                .map(wire),
        );
        if bare.snapshot().state == State::Finished {
            break;
        }
    }
    if bare.snapshot().state != State::Finished {
        return Err("bare reference did not finish".into());
    }
    let mut rows = Vec::new();
    for scenario in [
        "running",
        "paused",
        "ready",
        "file",
        "reply-before-five",
        "finished",
    ] {
        let mut simulation = prepare()?;
        let target = simulation.snapshot().planned_end;
        if scenario != "ready" {
            simulation.start(0).map_err(|e| format!("{e:?}"))?;
        }
        if scenario == "paused" {
            simulation.pause().map_err(|e| format!("{e:?}"))?;
        }
        let mut output = Vec::new();
        if scenario == "finished" {
            for _ in 0..1000 {
                output.extend(
                    simulation
                        .advance(target, 256)
                        .map_err(|e| format!("{e:?}"))?
                        .into_iter()
                        .map(wire),
                );
                if simulation.snapshot().state == State::Finished {
                    break;
                }
            }
            if simulation.snapshot().state != State::Finished {
                return Err("bare baseline did not finish".into());
            }
        }
        let mut watched = ClockWatchedSimulation::new(simulation, scenario == "file");
        for i in 0..4 {
            watched
                .query_sent(f64::from(i))
                .map_err(|e| format!("{e:?}"))?;
        }
        if scenario == "running" {
            let Advance::Output(first) =
                watched.advance(target, 1).map_err(|e| format!("{e:?}"))?
            else {
                return Err("four queries did not remain active".into());
            };
            if first.len() != 1 || watched.snapshot().issued != 1 {
                return Err("fixture did not issue exactly one initial record".into());
            }
            output.extend(first.into_iter().map(wire));
        }
        if scenario == "reply-before-five" {
            watched.reply_received();
        }
        watched.query_sent(4.).map_err(|e| format!("{e:?}"))?;
        let mut lost_reports = 0;
        let before = watched.snapshot().issued;
        let mut issue_after_check = 0;
        for _ in 0..1000 {
            match watched.advance(target, 256).map_err(|e| format!("{e:?}"))? {
                Advance::LostCommunication => {
                    lost_reports += 1;
                    break;
                }
                Advance::Output(commands) => {
                    issue_after_check += commands.len();
                    output.extend(commands.into_iter().map(wire));
                }
            }
            if watched.snapshot().state == State::Finished {
                break;
            }
        }
        watched.reply_received();
        let Advance::Output(late) = watched.advance(target, 256).map_err(|e| format!("{e:?}"))?
        else {
            return Err("timeout was not one-shot".into());
        };
        let snapshot = watched.snapshot();
        let expected = match scenario {
            "file" | "reply-before-five" => State::Finished,
            "finished" => State::Finished,
            _ => State::Faulted,
        };
        if !late.is_empty()
            || snapshot.state != expected
            || snapshot.issued != before + issue_after_check
        {
            return Err("late reply changed terminal output or state".into());
        }
        if output.len() != snapshot.issued || !expected_output.starts_with(&output) {
            return Err("watched simulator changed the released motion records".into());
        }
        rows.push(json!({"case":scenario,"state":match snapshot.state {
            State::Faulted=>"faulted", State::Finished=>"complete", _=>"unexpected",
        },"total":snapshot.total,"issued_before_check":before,"issued":snapshot.issued,
        "lost_reports":lost_reports,"late_output":late.len()}));
    }
    println!("{}", json!({"mode":"simulation","cases":rows}));
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
