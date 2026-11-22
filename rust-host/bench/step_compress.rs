// SPDX-License-Identifier: GPL-3.0-only
#[cfg(not(previous_allocator))]
use klipper_host_protocol::compress_each;
#[cfg(previous_allocator)]
use klipper_host_protocol::compress_run;
use klipper_host_protocol::Block;
use std::{
    hint::black_box,
    io::{self, BufRead},
    time::Instant,
};

fn run(data: &[(u64, u32, Vec<u64>)], report: bool) -> u64 {
    let mut result = 0u64;
    for (base, error, clocks) in data {
        #[allow(unused_mut)]
        let mut consume = |b: Block| {
            if report {
                println!("{} {} {} {}", b.interval, b.count, b.add, b.last_clock);
            }
            result = result.wrapping_add(
                b.last_clock ^ u64::from(b.count) ^ u64::from(b.add as u16) ^ u64::from(b.interval),
            );
        };
        #[cfg(not(previous_allocator))]
        compress_each(*base, clocks, *error, consume).unwrap();
        #[cfg(previous_allocator)]
        for block in compress_run(*base, clocks, *error).unwrap() {
            consume(block);
        }
        if report {
            println!("end");
        }
    }
    result
}

fn main() {
    let data: Vec<_> = io::stdin()
        .lock()
        .lines()
        .map(|line| {
            let line = line.unwrap();
            let mut v = line
                .split_ascii_whitespace()
                .map(|s| s.parse::<u64>().unwrap());
            let base = v.next().unwrap();
            let error = v.next().unwrap() as u32;
            let count = v.next().unwrap() as usize;
            let clocks: Vec<_> = v.collect();
            assert_eq!(count, clocks.len());
            (base, error, clocks)
        })
        .collect();
    if let Some(arg) = std::env::args().nth(1) {
        let rounds: usize = arg.parse().unwrap();
        black_box(run(&data, false));
        let now = Instant::now();
        let mut sum = 0u64;
        for _ in 0..rounds {
            sum = sum.wrapping_add(black_box(run(black_box(&data), false)));
        }
        println!("{} {sum}", now.elapsed().as_nanos());
    } else {
        run(&data, true);
    }
}
