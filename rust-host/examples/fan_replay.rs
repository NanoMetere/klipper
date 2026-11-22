// SPDX-License-Identifier: GPL-3.0-only
//! In-memory fan duty trace; no peripheral commands are issued.

use klipper_host_protocol::fan::{Fan, Output, Settings};
use std::io::{self, BufRead};

fn main() {
    let mut fan = None;
    for row in io::stdin().lock().lines() {
        let row = row.unwrap();
        let mut words = row.split_ascii_whitespace();
        let op = words.next().unwrap();
        let mut read = || words.next().unwrap().parse::<f64>().unwrap();
        if op == "new" {
            fan = Some(
                Fan::new(Settings {
                    max_power: read(),
                    off_below: read(),
                    kick_seconds: read(),
                    min_schedule_seconds: read(),
                    enable_present: read() != 0.0,
                })
                .unwrap(),
            );
        } else {
            let fan = fan.as_mut().unwrap();
            let at = read();
            let out = match op {
                "queue" => {
                    fan.queue(at, read()).unwrap();
                    Vec::new()
                }
                "flush" => fan.flush(at).unwrap(),
                "async" => fan.asynchronous(at, read()).unwrap(),
                _ => unreachable!(),
            };
            for write in out {
                match write {
                    Output::Pwm { clock, duty } => println!("P {clock:.17e} {duty:.17e}"),
                    Output::Enable { clock, on } => println!("E {clock:.17e} {}", u8::from(on)),
                }
            }
            println!(
                "S {:.17e} {:.17e} {}",
                fan.speed(),
                fan.next_min_flush_time(),
                fan.queued()
            );
        }
        assert!(words.next().is_none());
    }
}
