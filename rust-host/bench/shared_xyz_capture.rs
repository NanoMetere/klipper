// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    extrusion::prune_prepared,
    step_clock::{captured_cutoff, encode_captured_windows, encode_trace, Clock, Command},
    step_times::{Axis, Solver},
    trajectory::{Trajectory, Trapezoid},
};
use std::io::{self, BufRead};
fn main() {
    let options: Vec<_> = std::env::args().skip(1).collect();
    assert_eq!(options.len(), 8);
    let hz = options[0].parse::<f64>().unwrap();
    let max_error = options[1].parse::<u32>().unwrap();
    let lengths: Vec<f64> = options[2..=4].iter().map(|s| s.parse().unwrap()).collect();
    let inverse: Vec<_> = options[5..]
        .iter()
        .map(|s| match s.as_str() {
            "0" => false,
            "1" => true,
            _ => panic!("bad inversion flag"),
        })
        .collect();
    let layouts = [
        (Axis::CoreXYPlus, lengths[0]),
        (Axis::CoreXYMinus, lengths[1]),
        (Axis::Z, lengths[2]),
    ];
    let mut motors: Vec<_> = layouts
        .iter()
        .map(|&(axis, dist)| Solver::new(axis, dist, [0.; 3]).unwrap())
        .collect();
    let mut events = [Vec::new(), Vec::new(), Vec::new()];
    let mut captured = [Vec::new(), Vec::new(), Vec::new()];
    let mut queue = Trajectory::new(1024).unwrap();
    let mut from = 0.0_f64;
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let w: Vec<_> = line.split_ascii_whitespace().collect();
        let f = |i: usize| w[i].parse::<f64>().unwrap();
        let cruise = |i| {
            let value = f(i);
            // The stock trap queue skips a non-positive cruise phase.
            // Accept only the same round-off-sized negative as append_profile.
            if (-1e-12..0.0).contains(&value) {
                0.0
            } else {
                value
            }
        };
        match w[0] {
            "a" => queue
                .append(Trapezoid {
                    at: f(1),
                    phase_sec: [f(2), cruise(3), f(4)],
                    start: [f(5), f(6), f(7)],
                    direction: [f(8), f(9), f(10)],
                    start_velocity: f(11),
                    cruise_velocity: f(12),
                    accel: f(13),
                })
                .unwrap(),
            "b" => {
                assert!(events.iter().all(Vec::is_empty) && queue.segments().is_empty());
                let pos = [f(1), f(2), f(3)];
                motors = layouts
                    .iter()
                    .map(|&(axis, dist)| Solver::new(axis, dist, pos).unwrap())
                    .collect();
            }
            "t" => {
                prune_prepared(&mut queue, f(1)).unwrap();
            }
            "w" => {
                for (window, events) in captured.iter_mut().zip(&mut events) {
                    window.push((std::mem::take(events), captured_cutoff(f(1), hz).unwrap()));
                }
            }
            "f" => {
                let to = f(1);
                assert!(from <= to);
                for s in queue.segments() {
                    if s.end().min(to) <= s.at.max(from) {
                        continue;
                    }
                    for (motor, out) in motors.iter_mut().zip(&mut events) {
                        out.extend(
                            motor
                                .crossing_events(s, from.max(s.at), to.min(s.end()), 65535)
                                .unwrap(),
                        );
                    }
                }
                from = to;
            }
            _ => unreachable!(),
        }
    }
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.,
        hz,
    };
    for ((events, captured), inverse) in events.into_iter().zip(captured).zip(inverse) {
        let result = if captured.is_empty() {
            encode_trace(clock, &events, max_error, inverse)
        } else {
            encode_captured_windows(clock, &captured, &events, max_error, inverse)
        }
        .unwrap();
        for c in result {
            match c {
                Command::SetDirection(v) => println!("d {}", v as u8),
                Command::Steps(b) => {
                    println!("s {} {} {} {}", b.interval, b.count, b.add, b.last_clock)
                }
            }
        }
        println!("end");
    }
}
