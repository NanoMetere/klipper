// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    step_times::{Axis, Event, Solver},
    trajectory::Segment,
};
use std::io::{self, BufRead};

fn main() {
    let mut solver = Solver::new(Axis::X, 1.0, [0.; 3]).unwrap();
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut parts = line.split_ascii_whitespace();
        match parts.next().unwrap() {
            "r" => {
                let axis = match parts.next().unwrap() {
                    "x" => Axis::X,
                    "y" => Axis::Y,
                    "z" => Axis::Z,
                    "+" => Axis::CoreXYPlus,
                    "-" => Axis::CoreXYMinus,
                    _ => panic!(),
                };
                let v: Vec<f64> = parts.map(|v| v.parse().unwrap()).collect();
                solver = Solver::new(axis, v[0], v[1..4].try_into().unwrap()).unwrap();
            }
            "m" => {
                let v: Vec<f64> = parts.map(|v| v.parse().unwrap()).collect();
                let m = Segment {
                    at: v[0],
                    duration: v[1],
                    position: v[2..5].try_into().unwrap(),
                    direction: v[5..8].try_into().unwrap(),
                    start_velocity: v[8],
                    half_acceleration: v[9],
                };
                for event in solver.crossing_events(&m, v[10], v[11], 65536).unwrap() {
                    match event {
                        Event::Commit => println!("commit"),
                        Event::Crossing {
                            direction_positive,
                            segment_time,
                            offset,
                        } => println!(
                            "step {} {segment_time:.17e} {offset:.17e}",
                            u8::from(direction_positive)
                        ),
                    }
                }
                println!("pos {:.17e}", solver.commanded_position());
            }
            _ => panic!(),
        }
    }
}
