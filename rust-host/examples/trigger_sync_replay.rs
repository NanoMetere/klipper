// SPDX-License-Identifier: GPL-3.0-only
//! Replays trigger schedule vectors in memory.

use klipper_host_protocol::trigger_sync::Timing;
use std::io::{self, BufRead};

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_whitespace();
        let start = fields.next().unwrap().parse().unwrap();
        let hz = fields.next().unwrap().parse().unwrap();
        let peer = fields.next().unwrap().parse().unwrap();
        let total = fields.next().unwrap().parse().unwrap();
        assert!(fields.next().is_none());
        let timing = Timing::for_peer(start, hz, peer, total).unwrap();
        println!(
            "{} {} {} {}",
            timing.report_clock, timing.report_ticks, timing.expires, timing.extend_threshold
        );
    }
}
