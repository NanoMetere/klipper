// SPDX-License-Identifier: GPL-3.0-only
//! Bounded offline polling fixture; no network, files, or printer I/O.
use klipper_host_protocol::webhooks::{Session, Snapshot};
use serde_json::{json, Value};
use std::{
    env,
    io::{self, Write},
};

const POLLS: usize = 12_000;
const CLIENTS: usize = 8;

fn put(snapshot: &mut Snapshot, name: &str, value: Value) {
    assert!(snapshot.set_object(name.into(), value.as_object().unwrap().clone()));
}

fn main() -> io::Result<()> {
    let mut args = env::args().skip(1);
    let emit = args.next().is_some_and(|arg| {
        assert_eq!(arg, "--emit");
        true
    });
    assert!(args.next().is_none());
    let mut stdout = io::BufWriter::new(io::stdout().lock());
    let mut snapshot = Snapshot::default();
    let command_help: Vec<_> = (0..90).map(|i| format!("device option {i}")).collect();
    put(
        &mut snapshot,
        "configfile",
        json!({"profiles": command_help, "keys": ["mcu", "heater", "input_shaper"]}),
    );
    put(
        &mut snapshot,
        "toolhead",
        json!({"position":[0,10,2,0],
        "homed_axes":"xyz","axis_minimum":[0,0,0,0],"axis_maximum":[300,300,300,0],
        "max_velocity":300,"max_accel":5000,"extruder":"extruder"}),
    );
    put(
        &mut snapshot,
        "print_stats",
        json!({"filename":"fixture.gcode","state":"printing",
        "print_duration":0,"filament_used":0,"message":"","info":{"total_layer":90}}),
    );
    put(
        &mut snapshot,
        "heater_bed",
        json!({"temperature":60,"target":60,"power":0.18}),
    );
    put(
        &mut snapshot,
        "extruder",
        json!({"temperature":210,
        "target":210,"power":0.28,"pressure_advance":0.04}),
    );
    put(
        &mut snapshot,
        "webhooks",
        json!({"state":"ready","state_message":"Offline fixture"}),
    );

    let mut sessions: Vec<_> = (0..CLIENTS).map(|_| Session::default()).collect();
    let mut frames = 0;
    let mut byte_count = 0;
    let mut result = Ok(());
    let mut changes = |frame: &[u8]| {
        frames += 1;
        byte_count += frame.len();
        if emit && result.is_ok() {
            result = stdout.write_all(frame);
        }
    };
    for (i, session) in sessions.iter_mut().enumerate() {
        let objects = if i % 2 == 0 {
            json!({"toolhead":null,"print_stats":null,"heater_bed":null,
                   "extruder":null,"webhooks":null,"configfile":null})
        } else {
            json!({"toolhead":["position"],"heater_bed":["temperature"],
                   "webhooks":["state"],"extruder":["temperature","target"]})
        };
        let mut input = serde_json::to_vec(&json!({"id":i,"method":"objects/subscribe",
            "params":{"objects":objects,"response_template":{"method":"process_status_update"}}}))?;
        input.push(3);
        assert_eq!(
            session.feed(&input, &snapshot, 0., &mut changes).requests,
            1
        );
    }
    for tick in 1..=POLLS {
        if tick % 37 == 0 {
            put(
                &mut snapshot,
                "toolhead",
                json!({"position":[tick % 300,10,2,tick * 2],
                "homed_axes":"xyz","axis_minimum":[0,0,0,0],"axis_maximum":[300,300,300,0],
                "max_velocity":300,"max_accel":5000,"extruder":"extruder"}),
            );
        }
        if tick % 113 == 0 {
            put(
                &mut snapshot,
                "heater_bed",
                json!({"temperature":60 + ((tick / 113) % 2),"target":60,"power":0.18}),
            );
        }
        for session in &mut sessions {
            session.update(&snapshot, tick as f64 / 20., &mut changes);
        }
    }
    result?;
    if emit {
        stdout.flush()?;
    }
    eprintln!("{POLLS} polls × {CLIENTS} clients, {frames} frames and {byte_count} exact bytes");
    Ok(())
}
