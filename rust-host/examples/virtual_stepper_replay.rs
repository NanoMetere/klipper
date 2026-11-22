// SPDX-License-Identifier: GPL-3.0-only
//! Replay validated compressed blocks with no motor interface.

use klipper_host_protocol::{step_wire::Message, virtual_stepper::Stepper};
use std::io::{self, BufRead};

fn main() {
    let mut stepper = Stepper::new(1, 0, 0);
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_whitespace();
        match fields.next().unwrap() {
            "start" => {
                stepper = Stepper::new(
                    1,
                    fields.next().unwrap().parse().unwrap(),
                    fields.next().unwrap().parse().unwrap(),
                );
            }
            "dir" => stepper
                .accept(Message::NextDirection {
                    oid: 1,
                    positive: fields.next().unwrap() == "1",
                })
                .unwrap(),
            "block" => {
                stepper
                    .accept(Message::Block {
                        oid: 1,
                        interval: fields.next().unwrap().parse().unwrap(),
                        count: fields.next().unwrap().parse().unwrap(),
                        add: fields.next().unwrap().parse().unwrap(),
                    })
                    .unwrap();
            }
            "at" => {
                let state = stepper
                    .advance(fields.next().unwrap().parse().unwrap())
                    .unwrap();
                println!(
                    "{} {} {}",
                    state.observed_clock, state.position_steps, state.pending_blocks
                );
            }
            _ => unreachable!(),
        }
        assert!(fields.next().is_none());
    }
}
