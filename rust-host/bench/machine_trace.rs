// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    config,
    extrusion::{Extruder, Settings},
    machine_preview::{Machine, SimulatedExtruder},
    step_clock::{Clock, Command},
};
use std::{
    collections::BTreeMap,
    fs,
    io::{self, Read},
};
fn main() {
    let filename = std::env::args_os().nth(1).unwrap();
    let config_text = fs::read_to_string(filename).unwrap();
    let cfg = config::parse(
        &BTreeMap::from([("machine.cfg".into(), config_text)]),
        "machine.cfg",
    )
    .unwrap();
    let machine = Machine::from_config(&cfg).unwrap();
    let mut script = String::new();
    io::stdin().read_to_string(&mut script).unwrap();
    let e = Extruder::new(Settings {
        nozzle: 0.4,
        filament: 1.75,
        max_area: 0.64,
        retraction_distance: 15.0,
        retraction_velocity: 20.0,
        retraction_accel: 300.0,
        corner_velocity: 1.0,
        min_simulated_temperature: 170.0,
    })
    .unwrap();
    let extra = std::env::args()
        .nth(2)
        .is_some()
        .then_some(SimulatedExtruder {
            geometry: &e,
            temperature: 210.0,
            step_distance: 0.004,
            inverse_direction: true,
        });
    let trace = machine
        .trace_with_extruder(
            &script,
            [1., 1., 0.2],
            2.0,
            Clock {
                last_encoded: 0,
                time_origin: 0.,
                hz: 20e6,
            },
            20,
            [false, true, false],
            extra,
        )
        .unwrap();
    for motor in trace.motors.into_iter().chain(trace.extruder) {
        for cmd in motor {
            match cmd {
                Command::SetDirection(v) => println!("d {}", u8::from(v)),
                Command::Steps(b) => {
                    println!("s {} {} {} {}", b.interval, b.count, b.add, b.last_clock)
                }
            }
        }
        println!("end");
    }
    println!("t {:.17e}", trace.duration);
}
