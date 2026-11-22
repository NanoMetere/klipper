// SPDX-License-Identifier: GPL-3.0-only
//! Replay synthetic heater observations without issuing any heater command.

use klipper_host_protocol::verify_heater::{Decision, Monitor, Settings};
use std::io::{self, BufRead};

fn main() {
    let mut mon = Monitor::new(Settings::standard(false)).unwrap();
    for row in io::stdin().lock().lines() {
        let row = row.unwrap();
        let mut fields = row.split_whitespace();
        match fields.next().unwrap() {
            "new" => {
                let mut read = || fields.next().unwrap().parse().unwrap();
                mon = Monitor::new(Settings {
                    hysteresis: read(),
                    max_error: read(),
                    heating_gain: read(),
                    check_gain_time: read(),
                })
                .unwrap();
            }
            "sample" => {
                let mut read = || fields.next().unwrap().parse().unwrap();
                let decision = mon.sample(read(), read(), read());
                let state = mon.snapshot();
                println!(
                    "{} {} {} {:.17e} {:.17e} {:.17e} {:.17e} {:.17e}",
                    u8::from(matches!(decision, Decision::Fault(_))),
                    u8::from(state.approaching),
                    u8::from(state.starting),
                    state.target,
                    state.goal,
                    state.error,
                    state.at_goal.unwrap_or(1e30),
                    match decision {
                        Decision::Continue { next } => next,
                        _ => 1e30,
                    }
                );
            }
            _ => unreachable!(),
        }
        assert!(fields.next().is_none());
    }
}
