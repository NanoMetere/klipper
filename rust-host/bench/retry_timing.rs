// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    flight::Retry,
    retry_timing::{Timer, Timing},
};
use std::{
    collections::VecDeque,
    io::{self, BufRead},
};

fn main() {
    let (mut wire, mut speed) = (0usize, 0_f64);
    let mut send = 17;
    let mut receive = 17;
    let mut timing = Timing::default();
    let mut pending = VecDeque::<(usize, f64)>::new();
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut w = line.split_ascii_whitespace();
        let command = w.next().unwrap();
        let n: usize = w.next().unwrap().parse().unwrap();
        let at: f64 = w.next().unwrap().parse().unwrap();
        let bits = |len: usize| match wire {
            0 => 0.0,
            1 => (10.0 / speed) * len as f64,
            _ => (1.0 / speed) * (len * 8 + len.div_ceil(8) * 47 - 4) as f64,
        };
        match command {
            "n" => {
                wire = n;
                speed = at;
                timing = Timing::default();
                pending.clear();
                send = 17;
                receive = 17;
            }
            "s" => {
                timing.sent(send, at, pending.is_empty()).unwrap();
                pending.push_back((n, at));
                send += 1;
            }
            "a" => {
                let retired = pending.drain(..n).last().unwrap().1;
                receive += n as u64;
                timing
                    .acknowledged(
                        receive,
                        at,
                        Some(retired),
                        pending.front().map(|(len, _)| bits(*len)),
                    )
                    .unwrap();
            }
            "r" => {
                let r = if n == 1 {
                    timing.arm_fast_retry().unwrap();
                    Retry::DuplicateAck
                } else {
                    Retry::Timeout
                };
                timing
                    .retried(r, at, bits(1 + pending.front().unwrap().0))
                    .unwrap();
            }
            _ => panic!("{command}"),
        }
        let timer = match timing.timer() {
            Timer::Idle => 9_999_999_999_999_999.0,
            Timer::Fast => 0.0,
            Timer::At(t) => t,
        };
        println!(
            "{:.17e} {:.17e} {:.17e} {timer:.17e}",
            timing.timeout(),
            timing.mean(),
            timing.variation()
        );
    }
}
