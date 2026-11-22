// SPDX-License-Identifier: GPL-3.0-only
//! Run the demonstrator with a firmware dictionary but no physical device.

use klipper_host_protocol::{
    config,
    gcode_macros::Registry,
    homing::Counter,
    identify::Dictionary,
    machine_preview::Machine,
    simulated_printer::{Input, Simulation},
    step_wire::StepMessages,
    virtual_job::State,
    virtual_stepper::Stepper,
};
use serde_json::json;
use std::{collections::BTreeMap, env, fs};

fn run() -> Result<(), String> {
    let mut args = env::args_os().skip(1);
    let path = args.next().ok_or("usage: simulated_job FIRMWARE.dict")?;
    if args.next().is_some() {
        return Err("usage: simulated_job FIRMWARE.dict".into());
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
    let mut simulation = Simulation::prepare(
        &machine,
        &homing,
        Input {
            name: "path_macro_tail.gcode",
            script: include_str!("preview/path_macro_tail.gcode"),
            macros: Some(&macros),
            discovered: &codec,
            oids: [3, 4, 5],
            hz,
        },
    )
    .map_err(|e| format!("{e:?}"))?;
    let mut lanes = [0; 3];
    let mut steppers = [3, 4, 5].map(|oid| Stepper::new(oid, 0, 0));
    simulation.start(0).map_err(|e| format!("{e:?}"))?;
    let target = simulation.snapshot().planned_end;
    for _ in 0..1000 {
        let emitted = simulation
            .advance(target, 256)
            .map_err(|e| format!("{e:?}"))?;
        for item in emitted {
            lanes[item.lane] += 1;
            steppers[item.lane]
                .accept_payload(&codec, &item.command.bytes)
                .map_err(|e| format!("{e:?}"))?;
        }
        if simulation.snapshot().state == State::Finished {
            break;
        }
    }
    let end = simulation.snapshot();
    if end.state != State::Finished {
        return Err("simulation did not finish".into());
    }
    let mut displacement = [0; 3];
    for (slot, stepper) in displacement.iter_mut().zip(&mut steppers) {
        let end = stepper.advance(target).map_err(|e| format!("{e:?}"))?;
        if end.pending_blocks != 0 {
            return Err("undrained virtual stepper".into());
        }
        *slot = end.position_steps;
    }
    let (preflight_boundary, source_bytes) = simulation.preflight_boundary();
    println!(
        "{}",
        json!({"mode":"simulation", "state":"complete", "total":end.total,
        "issued":end.issued, "lanes":lanes, "clock_hz":hz, "planned_end_tick":end.planned_end,
        "preflight_boundary":preflight_boundary, "source_bytes":source_bytes,
        "motor_displacement_steps":displacement})
    );
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
