// SPDX-License-Identifier: GPL-3.0-only
//! Read-only display replay with explicit synthetic time and SD observations.
use klipper_host_protocol::{
    gcode::display::Display,
    webhooks::{Session, Snapshot},
};
use serde_json::Value;
use std::io::{self, BufRead};

fn main() {
    let mut display = Display::default();
    let mut snapshot = Snapshot::default();
    let mut session = Session::default();
    for row in io::stdin().lock().lines() {
        let (at, printing, sd, line): (f64, bool, Option<f64>, Option<String>) =
            serde_json::from_str(&row.unwrap()).unwrap();
        if line
            .as_deref()
            .is_some_and(|gcode| display.observe(gcode, at).is_err())
        {
            println!("null");
            continue;
        }
        assert!(display.publish(&mut snapshot, at, printing, sd).unwrap());
        let mut output = Vec::new();
        session.feed(
            b"{\"id\":1,\"method\":\"objects/query\",\"params\":{\"objects\":{\"display_status\":null}}}\x03",
            &snapshot,
            at,
            |data| output.extend_from_slice(data),
        );
        let json: Value = serde_json::from_slice(output.strip_suffix(&[3]).unwrap()).unwrap();
        println!("{}", json["result"]["status"]["display_status"]);
    }
}
