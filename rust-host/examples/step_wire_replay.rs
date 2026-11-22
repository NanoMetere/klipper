// SPDX-License-Identifier: GPL-3.0-only
//! Offline fixture. Writes hex strings only, no binary device output.
use klipper_host_protocol::{
    encode, identify::Dictionary, step_clock::Command, step_compress::Block,
    step_wire::StepMessages,
};
use std::io::{self, BufRead};

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn main() {
    let file = std::env::args_os().nth(1).unwrap();
    let bytes = std::fs::read(file).unwrap();
    let dict: Dictionary = serde_json::from_slice(&bytes).unwrap();
    let codec = StepMessages::from_dictionary(&dict).unwrap();
    let (mut oid, mut first) = (0, 0);
    let mut trace = Vec::new();
    for (sequence, line) in io::stdin().lock().lines().enumerate() {
        let line = line.unwrap();
        let mut v = line.split_ascii_whitespace();
        match v.next().unwrap() {
            "m" => {
                oid = v.next().unwrap().parse().unwrap();
                first = v.next().unwrap().parse().unwrap();
            }
            "d" => trace.push(Command::SetDirection(v.next().unwrap() == "1")),
            "s" => trace.push(Command::Steps(Block {
                interval: v.next().unwrap().parse().unwrap(),
                count: v.next().unwrap().parse().unwrap(),
                add: v.next().unwrap().parse().unwrap(),
                last_clock: v.next().unwrap().parse().unwrap(),
            })),
            "e" => {
                for (i, p) in codec
                    .payloads(oid, first, &trace)
                    .unwrap()
                    .into_iter()
                    .enumerate()
                {
                    println!(
                        "{} {} {} {}",
                        p.earliest_tick,
                        p.required_tick,
                        hex(&p.bytes),
                        hex(&encode((sequence + i) as u8, &p.bytes).unwrap())
                    );
                }
                trace.clear();
            }
            _ => panic!(),
        }
    }
}
