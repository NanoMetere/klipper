// SPDX-License-Identifier: GPL-3.0-only
//! Fixture adapter, not a printer command dispatcher.
use klipper_host_protocol::shared_extruder::{Queue, SharedStepper};
use std::io::{self, BufRead};

fn queue(s: &str) -> Queue {
    match s {
        "extruder" => Queue::Primary,
        "extruder1" => Queue::Secondary,
        _ => panic!("unknown synthetic queue"),
    }
}

fn name(queue: Queue) -> &'static str {
    match queue {
        Queue::Primary => "extruder",
        Queue::Secondary => "extruder1",
    }
}

fn main() {
    let mut args = std::env::args().skip(1).map(|s| s.parse::<f64>().unwrap());
    let distance = args.next().unwrap();
    let initial = args.next().unwrap();
    assert!(args.next().is_none());
    let mut model = SharedStepper::new(distance, initial).unwrap();
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let words: Vec<_> = line.split_ascii_whitespace().collect();
        let tag = match words.as_slice() {
            ["move", target, end] => {
                model
                    .commit_move(queue(target), end.parse().unwrap())
                    .unwrap();
                continue;
            }
            ["activate", target] => {
                model.activate(queue(target));
                continue;
            }
            ["sync", target, flushed] => {
                let target = (*target != "-").then(|| queue(target));
                model.sync(target, flushed.parse().unwrap()).unwrap();
                "sync"
            }
            ["advance", gain] => {
                model.set_advance(gain.parse().unwrap()).unwrap();
                "advance"
            }
            _ => panic!("bad synthetic command"),
        };
        let status = model.snapshot();
        println!(
            "{tag} {} {} {} {} {} {} {}",
            name(status.active),
            status.bound.map(name).unwrap_or("-"),
            status.logical_end[0],
            status.logical_end[1],
            status.advance,
            status.commanded,
            status.mcu_steps
        );
    }
}
