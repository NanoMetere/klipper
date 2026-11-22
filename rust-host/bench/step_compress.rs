// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::step_compress::compress_run;
use std::io::{self, BufRead};

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut v = line
            .split_ascii_whitespace()
            .map(|s| s.parse::<u64>().unwrap());
        let base = v.next().unwrap();
        let error = v.next().unwrap() as u32;
        let count = v.next().unwrap() as usize;
        let clocks: Vec<_> = v.collect();
        assert_eq!(count, clocks.len());
        for b in compress_run(base, &clocks, error).unwrap() {
            println!("{} {} {} {}", b.interval, b.count, b.add, b.last_clock);
        }
        println!("end");
    }
}
