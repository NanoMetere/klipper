// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    config,
    machine_preview::Machine,
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
    let trace = machine
        .trace(
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
        )
        .unwrap();
    for motor in trace.motors {
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
