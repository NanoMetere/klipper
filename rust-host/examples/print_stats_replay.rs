// SPDX-License-Identifier: GPL-3.0-only
//! Compare simulated print reporting without access to an actual printer.

use klipper_host_protocol::print_stats::{Extrusion, State, Stats};
use serde_json::json;
use std::io::{self, BufRead};

fn main() {
    let mut stats = Stats::default();
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_whitespace();
        match fields.next().unwrap() {
            "file" => stats.select(fields.next().unwrap()).unwrap(),
            "layers" => {
                let mut next = || {
                    let f = fields.next().unwrap();
                    (f != "-").then(|| f.parse().unwrap())
                };
                stats.note_layers(next(), next());
            }
            cmd => {
                let at = fields.next().unwrap().parse().unwrap();
                let position = fields.next().unwrap().parse().unwrap();
                let factor = fields.next().unwrap().parse().unwrap();
                let e = Extrusion { position, factor };
                match cmd {
                    "start" => stats.note_start(at, e).unwrap(),
                    "pause" => stats.note_pause(at, e).unwrap(),
                    "switch" => stats.switch_extruder(at, e).unwrap(),
                    "cancel" => stats.note_finish(at, State::Cancelled, "").unwrap(),
                    "finish" => stats.note_finish(at, State::Complete, "").unwrap(),
                    "error" => stats
                        .note_finish(at, State::Error, "fixture_error")
                        .unwrap(),
                    "status" => {
                        let s = stats.observe(at, e).unwrap();
                        println!(
                            "{}",
                            json!({
                                "filename": s.filename, "total_duration": s.total_duration,
                                "print_duration": s.print_duration, "filament_used": s.filament_used,
                                "state": s.state.as_str(), "message": s.message,
                                "info": {"total_layer": s.layers[0], "current_layer": s.layers[1]},
                            })
                        );
                    }
                    _ => unreachable!(),
                }
            }
        }
        assert!(fields.next().is_none());
    }
}
