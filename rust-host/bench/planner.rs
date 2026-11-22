// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    clock_sync::ReleaseEstimate,
    encode,
    planner::{Decision, Planner, Priority, Snapshot, Wire},
};
use std::{
    io::{self, BufRead},
    str::{FromStr, SplitAsciiWhitespace},
};

fn value<T: FromStr>(words: &mut SplitAsciiWhitespace<'_>) -> T {
    words.next().unwrap().parse().ok().unwrap()
}

fn main() {
    let mut planner = Planner::new(8, 4096).unwrap();
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut w = line.split_ascii_whitespace();
        match w.next() {
            Some("p") => {
                let lane = value(&mut w);
                let release = value(&mut w);
                let raw: u64 = value(&mut w);
                let len = value(&mut w);
                let byte = value(&mut w);
                let priority = if raw == 0x7fff_ffff_0000_0000 {
                    Priority::Background
                } else {
                    Priority::Deadline(raw)
                };
                planner
                    .push(lane, release, priority, &[byte; 59][..len])
                    .unwrap();
            }
            Some("d") => {
                let wire = value::<u8>(&mut w);
                let speed = value(&mut w);
                let wire = match wire {
                    1 => Wire::Uart8N1 { baud: speed },
                    2 => Wire::ClassicCan { bitrate: speed },
                    _ => Wire::Pty,
                };
                let hz = value(&mut w);
                let at = value(&mut w);
                let ticks = value(&mut w);
                let ce = (hz != 0.0).then_some(ReleaseEstimate { at, ticks, hz });
                let s = Snapshot {
                    now: value(&mut w),
                    link_idle: value(&mut w),
                    burst_bytes: value(&mut w),
                    in_flight: value(&mut w),
                    outstanding_bytes: value(&mut w),
                    prior_ack_bytes: value(&mut w),
                    receive_window: value(&mut w),
                };
                match planner.plan(wire, ce, s).unwrap() {
                    Decision::Send(bytes) => {
                        let frame = encode(s.in_flight as u8, bytes.as_slice()).unwrap();
                        print!("send ");
                        for byte in frame {
                            print!("{byte:02x}");
                        }
                        println!();
                    }
                    Decision::AwaitAck { check_at } => println!("ack {check_at:.17}"),
                    Decision::DrainBurst => println!("drain {:.17}", s.now),
                    Decision::Idle => println!("idle"),
                    Decision::SleepUntil(t) => println!("sleep {t:.17}"),
                }
            }
            _ => panic!("unknown vector"),
        }
    }
}
