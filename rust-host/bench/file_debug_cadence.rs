// SPDX-License-Identifier: GPL-3.0-only
#[path = "../src/shared_debug_plan/cadence.rs"]
mod cadence;

use std::io::{self, BufRead};

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let fields: Vec<f64> = line
            .split_whitespace()
            .map(|v| v.parse().unwrap())
            .collect();
        let [need, generation, wire] = fields[..] else {
            std::process::exit(2)
        };
        let Some(mut cadence) = cadence::Cadence::new(need, generation, wire) else {
            std::process::exit(2);
        };
        for (wire, generation) in cadence.by_ref() {
            println!("{wire:.17e} {generation:.17e}");
        }
        println!("end {}", u8::from(cadence.more_armed()));
    }
}
