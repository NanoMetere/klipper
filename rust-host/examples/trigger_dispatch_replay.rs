// SPDX-License-Identifier: GPL-3.0-only
//! In-memory so stock C and Rust trigger dispatch can replay the same trace.

use klipper_host_protocol::{
    clock_sync::ReleaseEstimate,
    trigger_dispatch::{Action, Dispatch, Peer},
    trigger_sync::{Reason, State, Timing},
};
use std::{
    io::{self, BufRead},
    str::FromStr,
};

fn next<T: FromStr>(fields: &mut std::str::SplitWhitespace<'_>) -> T {
    fields.next().unwrap().parse().ok().unwrap()
}

fn estimate(fields: &mut std::str::SplitWhitespace<'_>) -> ReleaseEstimate {
    ReleaseEstimate {
        hz: next(fields),
        at: next(fields),
        ticks: next(fields),
    }
}

fn main() {
    let mut peers = Vec::new();
    let mut dispatch = None;
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_whitespace();
        match fields.next().unwrap() {
            "P" => {
                peers.clear();
                dispatch = None;
            }
            "M" => {
                let oid = next(&mut fields);
                let start = next(&mut fields);
                let expires = next(&mut fields);
                let timeout = next(&mut fields);
                let extend_threshold = next(&mut fields);
                peers.push(Peer {
                    oid,
                    last_clock: start,
                    timeout,
                    timing: Timing {
                        report_clock: start,
                        report_ticks: 0,
                        expires,
                        extend_threshold,
                    },
                    estimate: estimate(&mut fields),
                });
            }
            "S" => {
                let peer = next(&mut fields);
                let state = State {
                    oid: next(&mut fields),
                    can_trigger: next::<u8>(&mut fields) != 0,
                    reason: Reason::Other(next(&mut fields)),
                    clock: next(&mut fields),
                };
                let host_time = next(&mut fields);
                let estimate = estimate(&mut fields);
                let d = dispatch.get_or_insert_with(|| {
                    Dispatch::new(std::mem::take(&mut peers), Reason::HostRequest).unwrap()
                });
                for action in d.observe(peer, state, host_time, estimate).unwrap() {
                    match action {
                        Action::Extend { peer, oid, clock } => print!("E:{peer}:{oid}:{clock} "),
                        Action::Trigger { peer, oid, reason } => {
                            print!("T:{peer}:{oid}:{} ", reason.code())
                        }
                    }
                }
                println!();
            }
            _ => unreachable!(),
        }
        assert!(fields.next().is_none());
    }
}
