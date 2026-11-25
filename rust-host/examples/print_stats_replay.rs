// SPDX-License-Identifier: GPL-3.0-only
//! Compare simulated print reporting without access to an actual printer.

use klipper_host_protocol::print_stats::{Extrusion, State, Stats};
use klipper_host_protocol::webhooks::{Session, Snapshot};
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
                        let mut output = Snapshot::default();
                        assert!(s.publish_status(&mut output));
                        let mut session = Session::default();
                        session.feed(b"{\"id\":1,\"method\":\"objects/query\",\"params\":{\"objects\":{\"print_stats\":null}}}\x03",
                                     &output, at, &mut |raw: &[u8]| {
                            let result: serde_json::Value = serde_json::from_slice(&raw[..raw.len()-1]).unwrap();
                            println!("{}", result["result"]["status"]["print_stats"]);
                        });
                    }
                    _ => unreachable!(),
                }
            }
        }
        assert!(fields.next().is_none());
    }
}
