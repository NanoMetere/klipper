// SPDX-License-Identifier: GPL-3.0-only
//! Replay fan handoff choices, without peripheral access.

use klipper_host_protocol::toolchange::FanTransfer;
use std::io::{self, BufRead};

fn main() {
    let mut transfer = FanTransfer::new(false);
    for row in io::stdin().lock().lines() {
        let row = row.unwrap();
        let mut args = row.split_whitespace();
        let op = args.next().unwrap();
        if op == "new" {
            transfer = FanTransfer::new(args.next().unwrap() == "1");
            continue;
        }
        let id = |field: &str| {
            if field == "-" {
                None
            } else {
                Some(field.parse::<u8>().unwrap())
            }
        };
        let val = |field: &str| {
            if field == "-" {
                None
            } else {
                Some(field.parse::<f64>().unwrap())
            }
        };
        let fan = id(args.next().unwrap());
        let speed = val(args.next().unwrap());
        let intents = match op {
            "set" => transfer
                .set(fan, speed.unwrap())
                .unwrap()
                .into_iter()
                .collect(),
            "activate" => transfer.activate(fan, speed).unwrap(),
            _ => unreachable!(),
        };
        for out in intents {
            println!("I {} {:.17e}", out.fan, out.speed);
        }
        let active = transfer.active().map_or("-".into(), |i| i.to_string());
        let pending = transfer
            .pending()
            .map_or("-".into(), |f| format!("{f:.17e}"));
        println!("S {active} {pending}");
    }
}
