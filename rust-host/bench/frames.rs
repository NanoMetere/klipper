// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::decode_prefix;
use std::{hint::black_box, time::Instant};

fn main() {
    let corpus = std::fs::read(std::env::args_os().nth(1).expect("corpus path")).unwrap();
    assert_eq!(corpus.len(), 4 * 64);
    let frames: Vec<_> = corpus.chunks_exact(64).map(|s| &s[..s[0] as usize]).collect();
    let mut sum = 0;
    let iterations = 2_000_000;
    let start = Instant::now();
    for i in 0..iterations {
        sum += decode_prefix(black_box(frames[i & 3])).unwrap().unwrap().1;
    }
    let elapsed = start.elapsed();
    assert_eq!(sum, iterations / 4 * (5 + 8 + 11 + 64));
    println!("{:.3}", elapsed.as_nanos() as f64 / iterations as f64);
}
