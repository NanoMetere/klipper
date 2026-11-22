// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::trajectory::{Trajectory, Trapezoid};
use std::io::{self, BufRead};

fn main() {
    let mut q = Trajectory::new(16_384).unwrap();
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut fields = line.split_ascii_whitespace();
        match fields.next().unwrap() {
            "a" => {
                let v: Vec<f64> = fields.map(|f| f.parse().unwrap()).collect();
                q.append(Trapezoid {
                    at: v[0],
                    phase_sec: v[1..4].try_into().unwrap(),
                    start: v[4..7].try_into().unwrap(),
                    direction: v[7..10].try_into().unwrap(),
                    start_velocity: v[10],
                    cruise_velocity: v[11],
                    accel: v[12],
                })
                .unwrap();
            }
            "p" => match q.position(fields.next().unwrap().parse().unwrap()).unwrap() {
                None => println!("empty"),
                Some([x, y, z]) => println!("p {x:.17e} {y:.17e} {z:.17e}"),
            },
            "d" => {
                for s in q.segments() {
                    print!("s");
                    for v in [s.at, s.duration, s.start_velocity, s.half_acceleration]
                        .into_iter()
                        .chain(s.position)
                        .chain(s.direction)
                    {
                        print!(" {v:.17e}");
                    }
                    println!();
                }
            }
            cmd => panic!("{cmd}"),
        }
    }
}
