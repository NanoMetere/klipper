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
    webhooks::{Session, Snapshot, ETX},
};
use serde_json::json;
use std::{collections::BTreeMap, env, fs};

fn run() -> Result<(), String> {
    let mut args = env::args_os().skip(1);
    let path = args
        .next()
        .ok_or("usage: clock_watched_status_replay FIRMWARE.dict")?;
    if args.next().is_some() {
        return Err("usage: clock_watched_status_replay FIRMWARE.dict".into());
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
    let mut simulation = prepare()?;
    simulation.start(0).map_err(|e| format!("{e:?}"))?;
    let target = simulation.snapshot().planned_end;
    let mut watched = ClockWatchedSimulation::new(simulation, false);
    let mut source = Snapshot::default();
    let mut client = Session::default();
    let mut messages = Vec::new();
    let mut emit = |data: &[u8]| {
        assert_eq!(data.last(), Some(&ETX));
        messages
            .push(serde_json::from_slice::<serde_json::Value>(&data[..data.len() - 1]).unwrap());
    };
    let error = |e| format!("{e:?}");
    assert!(watched.publish_watched(&mut source).map_err(error)?);
    let select = json!({"id":1,"method":"objects/subscribe","params":{
        "objects":{"print_stats":["state"],"virtual_job":["state","issued"],
        "virtual_clock_liveness":["mode","active","timeout_reported","file_output"]},
        "response_template":{"method":"clock_status_update"}}});
    let mut request = serde_json::to_vec(&select).map_err(|e| e.to_string())?;
    request.push(ETX);
    assert_eq!(client.feed(&request, &source, 0., &mut emit).requests, 1);
    for i in 0..4 {
        watched
            .query_sent(f64::from(i))
            .map_err(|e| format!("{e:?}"))?;
    }
    let Advance::Output(first) = watched.advance(target, 1).map_err(error)? else {
        return Err("early timeout".into());
    };
    assert_eq!(first.len(), 1);
    assert!(watched.publish_watched(&mut source).map_err(error)?);
    assert!(client.update(&source, 1., &mut emit));

    watched.query_sent(4.).map_err(|e| format!("{e:?}"))?;
    assert!(watched.publish_watched(&mut source).map_err(error)?);
    assert!(client.update(&source, 2., &mut emit));
    assert!(matches!(
        watched.advance(target, 256).map_err(error)?,
        Advance::LostCommunication
    ));
    assert!(watched.publish_watched(&mut source).map_err(error)?);
    assert!(client.update(&source, 3., &mut emit));

    watched.reply_received();
    let Advance::Output(late) = watched.advance(target, 256).map_err(error)? else {
        return Err("late report repeated".into());
    };
    assert!(late.is_empty());
    assert!(watched.publish_watched(&mut source).map_err(error)?);
    assert!(client.update(&source, 4., &mut emit));
    assert!(!client.update(&source, 5., &mut emit));

    let mut request =
        b"{\"id\":2,\"method\":\"gcode/script\",\"params\":{\"script\":\"M112\"}}".to_vec();
    request.push(ETX);
    assert_eq!(client.feed(&request, &source, 6., &mut emit).requests, 1);
    assert_eq!(watched.snapshot().state, State::Faulted);
    assert_eq!(watched.snapshot().issued, 1);
    println!("{}", json!({"messages":messages,"late_output":late.len()}));
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
