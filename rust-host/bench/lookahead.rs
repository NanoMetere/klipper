// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::lookahead::{Limits, LookAhead, Move};
use std::{
    hint::black_box,
    io::{self, BufRead},
    time::Instant,
};

fn solve(cases: &[(char, Vec<f64>)], emit: bool) -> (usize, f64) {
    let mut q = LookAhead::new(4096, None).unwrap();
    let mut limits = Limits::new(200.0, 3000.0, 5.0, 0.5).unwrap();
    let mut pos = [0.0; 4];
    let mut count = 0;
    let mut duration = 0.0;
    for (op, v) in cases {
        match op {
            'r' => {
                assert_eq!(q.queued(), 0);
                q = LookAhead::new(4096, (v[4] >= 0.0).then_some(v[4])).unwrap();
                limits = Limits::new(v[0], v[1], v[2], v[3]).unwrap();
                pos = [0.0; 4];
            }
            'm' => {
                let end = v[..4].try_into().unwrap();
                let mut move_ = Move::new(limits, pos, end, v[4]).unwrap();
                if v[5] > 0.0 {
                    move_.limit(v[5], v[6]).unwrap();
                }
                if v[7] >= 0.0 {
                    move_.limit_next_junction(v[7]).unwrap();
                }
                pos = move_.end_position();
                let lazy = q.push(move_).unwrap();
                if emit {
                    println!("push {} {}", u8::from(lazy), q.queued());
                }
            }
            'f' => {
                let result = q.flush(v[0] != 0.0).unwrap();
                if emit {
                    println!("flush {} {}", result.len(), q.queued());
                }
                count += result.len();
                for m in result {
                    duration += m.accel_t + m.cruise_t + m.decel_t;
                    if emit {
                        print!("move {}", u8::from(m.kinematic));
                        for v in m.start.into_iter().chain(m.end).chain(m.direction).chain([
                            m.start_v, m.cruise_v, m.end_v, m.accel, m.accel_t, m.cruise_t,
                            m.decel_t,
                        ]) {
                            print!(" {v:.17e}");
                        }
                        println!();
                    }
                }
            }
            _ => panic!("{op}"),
        }
    }
    assert_eq!(q.queued(), 0);
    (count, duration)
}

fn main() {
    let cases: Vec<_> = io::stdin()
        .lock()
        .lines()
        .map(|line| {
            let line = line.unwrap();
            let mut fields = line.split_ascii_whitespace();
            (
                fields.next().unwrap().chars().next().unwrap(),
                fields.map(|v| v.parse::<f64>().unwrap()).collect(),
            )
        })
        .collect();
    if let Some(n) = std::env::args().nth(1) {
        let rounds = n.parse().unwrap();
        black_box(solve(&cases, false));
        for _ in 0..rounds {
            let start = Instant::now();
            let (n, duration) = black_box(solve(&cases, false));
            println!("{} {n} {duration:.17e}", start.elapsed().as_nanos());
        }
    } else {
        solve(&cases, true);
    }
}
