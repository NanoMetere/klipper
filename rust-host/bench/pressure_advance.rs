// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    extrusion::append_nominal, lookahead::Profile, pressure_advance::Advance,
    trajectory::Trajectory,
};
use std::io::{self, BufRead};

fn main() {
    let mut trajectory = Trajectory::new(4096).unwrap();
    let mut parameters = Vec::new();
    let mut smooth = 0.0;
    let mut advance = None;
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut parts = line.split_ascii_whitespace();
        let op = parts.next().unwrap();
        let v: Vec<f64> = parts.map(|s| s.parse().unwrap()).collect();
        match op {
            "r" => {
                trajectory = Trajectory::new(4096).unwrap();
                parameters.clear();
                advance = None;
            }
            "g" => {
                parameters.push((v[0], v[1]));
                smooth = v[2];
            }
            "m" => {
                let r = v[5].signum();
                let p = Profile {
                    start: [0., 0., 0., v[4]],
                    end: [0.; 4],
                    direction: [v[8], 0., 0., r],
                    kinematic: v[8] != 0.0,
                    accel: v[7] / r,
                    start_v: v[5] / r,
                    cruise_v: v[6] / r,
                    end_v: 0.,
                    accel_t: v[1],
                    cruise_t: v[2],
                    decel_t: v[3],
                };
                append_nominal(&mut trajectory, v[0], &p).unwrap();
            }
            "p" => {
                let model =
                    advance.get_or_insert_with(|| Advance::new(smooth, &parameters).unwrap());
                println!("{:.17e}", model.sample(&trajectory, v[0]).unwrap().unwrap());
            }
            _ => panic!(),
        }
    }
}
