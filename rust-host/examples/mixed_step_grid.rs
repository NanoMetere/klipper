// SPDX-License-Identifier: GPL-3.0-only
//! Signed net motor steps across explicit local clock phases; no transport.
use klipper_host_protocol::{
    config,
    homing::Counter,
    machine_preview::{
        mixed::{Input, Stage},
        Machine,
    },
    step_clock::Command,
    thermal::commands::{Bank, Profile, Role},
};
use std::{
    collections::BTreeMap,
    io::{self, Read},
};

fn main() {
    let config = std::fs::read_to_string(std::env::args_os().nth(1).unwrap()).unwrap();
    let cfg = config::parse(&BTreeMap::from([("main.cfg".into(), config)]), "main.cfg").unwrap();
    let machine = Machine::from_config(&cfg).unwrap();
    let start = [1., 1., 0.2];
    let mut home = machine.virtual_homing(start).unwrap();
    for (axis, n) in start.into_iter().enumerate() {
        let mark = Some(Counter {
            start: 3,
            trigger: 2,
            halt: 2,
        });
        let ticket = home.arm(axis as u8, n, true).unwrap();
        home.complete(
            ticket,
            if axis < 2 {
                [mark, mark, None]
            } else {
                [None, None, mark]
            },
            true,
        )
        .unwrap();
    }
    let heaters = [Profile {
        name: "extruder",
        role: Role::Extruder(0),
        minimum: 0.,
        maximum: 300.,
    }];
    let bank = Bank::new(&heaters, &[], Some(0), false).unwrap();
    let mut corpus = String::new();
    io::stdin().read_to_string(&mut corpus).unwrap();
    for code in corpus.split("\n--case--\n") {
        let plan = machine
            .trace_mixed(
                &home,
                &bank,
                Input {
                    script: code,
                    extruder: None,
                    hz: 20e6,
                    max_error: 20,
                    invert: [false, true, false],
                },
            )
            .unwrap();
        let mut total = [0_i64; 3];
        for stage in plan.phases {
            if let Stage::Motion { trace, .. } = stage {
                for (i, cmds) in trace.motors.into_iter().enumerate() {
                    let mut sign = 0;
                    for cmd in cmds {
                        match cmd {
                            Command::SetDirection(positive) => sign = if positive { 1 } else { -1 },
                            Command::Steps(b) => total[i] += i64::from(b.count) * sign,
                        }
                    }
                }
            }
        }
        println!("{} {} {}", total[0], total[1], total[2]);
    }
}
