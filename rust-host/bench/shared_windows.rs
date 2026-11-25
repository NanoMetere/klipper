// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    extrusion::append_prepared,
    pressure_advance::{Advance, WindowStepper},
    step_clock::{encode_trace, Clock, Command},
    step_times::Event,
    trajectory::{Trajectory, Trapezoid},
};
use std::io::{self, BufRead};

fn main() {
    let wire = std::env::args().nth(1).is_some_and(|s| s == "--wire");
    let mut all = Vec::new();
    let mut motor = WindowStepper::new(22.17 / 3200., 0.04).unwrap();
    let mut queues = [
        Trajectory::new(2048).unwrap(),
        Trajectory::new(2048).unwrap(),
    ];
    let mut gains = Vec::new();
    let mut bound = None;
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let fields: Vec<_> = line.split_ascii_whitespace().collect();
        let values = |i: usize| fields[i].parse::<f64>().unwrap();
        match fields[0] {
            "a" => {
                append_prepared(
                    &mut queues[fields[1].parse::<usize>().unwrap()],
                    Trapezoid {
                        at: values(3),
                        phase_sec: [values(4), values(5), values(6)],
                        start: [values(7), 0., 0.],
                        direction: [1., values(2), 0.],
                        start_velocity: values(8),
                        cruise_velocity: values(9),
                        accel: values(10),
                    },
                )
                .unwrap();
            }
            "p" => gains.push((values(1), values(2))),
            "b" => {
                let q = fields[1].parse::<i8>().unwrap();
                bound = (q >= 0).then_some(q as usize);
                if bound.is_some() {
                    motor.rebase_after_flush(values(2)).unwrap();
                }
                println!("bind {q} {:.17e}", motor.commanded());
            }
            "f" => {
                let end = values(1);
                let a = Advance::new(0.04, &gains).unwrap();
                let events = motor
                    .flush(bound.map(|i| &queues[i]), &a, end, 65535)
                    .unwrap();
                if wire {
                    all.extend(events);
                } else {
                    for e in events {
                        match e {
                            Event::Commit => println!("commit"),
                            Event::Crossing {
                                direction_positive,
                                segment_time,
                                offset,
                            } => {
                                println!(
                                    "step {} {:.17e} {:.17e}",
                                    direction_positive as u8, segment_time, offset
                                );
                            }
                        }
                    }
                }
                println!("flush {:.17e} {:.17e}", end, motor.commanded());
            }
            _ => unreachable!(),
        }
    }
    if wire {
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.,
            hz: 20_000_000.,
        };
        for command in encode_trace(clock, &all, 0, false).unwrap() {
            match command {
                Command::SetDirection(v) => println!("d {}", v as u8),
                Command::Steps(b) => {
                    println!("s {} {} {} {}", b.interval, b.count, b.add, b.last_clock)
                }
            }
        }
    }
}
