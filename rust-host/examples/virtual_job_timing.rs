// SPDX-License-Identifier: GPL-3.0-only
//! Offline scheduler measurement: no transport, printer, or live clock input.
use klipper_host_protocol::{
    identify::Dictionary,
    step_clock::Command,
    step_compress::Block,
    step_wire::{StepMessages, QUEUE, SET_DIR},
    virtual_job::{Job, State},
};
use std::time::Instant;

fn main() {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    assert_eq!(args.len(), 3);
    let parse = |i: usize| args[i].parse::<usize>().unwrap();
    let (n, rounds, variation) = (parse(0), parse(1), parse(2));
    assert!((1..=16).contains(&n) && (1..=100).contains(&rounds) && variation < 3);
    let dict: Dictionary = serde_json::from_value(serde_json::json!({
        "commands": {QUEUE: 15, SET_DIR: 16},
        "responses": {}, "version": "simulation", "build_versions":"scheduler bench"
    }))
    .unwrap();
    let codec = StepMessages::from_dictionary(&dict).unwrap();
    let mut all = Vec::new();
    let mut end = 0;
    for lane in 0..n {
        let mut rows = vec![Command::SetDirection(true)];
        let mut at = 0;
        let mut direction = true;
        for j in 0..4000 {
            if j > 0 && (j + lane) % 7 == 0 {
                direction = !direction;
                rows.push(Command::SetDirection(direction));
            }
            let interval = match variation {
                0 => 30 + (j % 10) as u32,             // many cross-lane ties
                1 => 30 + (lane * 9 + j % 53) as u32,  // uneven lane rates
                _ => 10 + (lane * 31 + j % 23) as u32, // lanes finish at different times
            };
            at += u64::from(interval);
            rows.push(Command::Steps(Block {
                interval,
                count: 1,
                add: 0,
                last_clock: at,
            }));
        }
        end = end.max(at);
        all.push(rows);
    }
    let borrowed = all.iter().map(Vec::as_slice).collect::<Vec<_>>();
    let oids = (0..n as u8).map(|lane| lane + 7).collect::<Vec<_>>();
    for _ in 0..=rounds {
        let mut job = Job::for_lanes(&codec, &borrowed, &oids, 0, end, 1).unwrap();
        job.start(0).unwrap();
        let mut tick = 0;
        let mut elapsed = 0_u128;
        let mut count = 0;
        // FNV over full order/ticks and payload bytes outside the timed window.
        let mut hash = 0xcbf29ce484222325_u64;
        let mut digest = |value: u64| {
            hash = (hash ^ value).wrapping_mul(0x100000001b3);
        };
        while job.snapshot().state == State::Running {
            let start = Instant::now();
            let batch = job
                .poll(tick, if variation == 0 { 256 } else { 61 })
                .unwrap();
            elapsed += start.elapsed().as_nanos();
            count += batch.len();
            for entry in batch {
                digest(entry.lane as u64);
                digest(entry.command.required_tick);
                digest(entry.command.earliest_tick);
                for b in entry.command.bytes {
                    digest(u64::from(b));
                }
            }
            tick = tick
                .saturating_add(if variation == 1 { 399 } else { 1940 })
                .min(end);
        }
        assert_eq!(count, job.snapshot().total);
        println!("{elapsed} {count} {hash}");
    }
}
