// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::thermal::{Mode, Model};
use std::io::{self, BufRead};
fn main() {
    let mut model = None;
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut w = line.split_ascii_whitespace();
        let op = w.next().unwrap();
        let v: Vec<f64> = w.map(|n| n.parse().unwrap()).collect();
        if op != "u" {
            let mode = if op == "p" {
                Mode::Pid {
                    kp: v[2],
                    ki: v[3],
                    kd: v[4],
                }
            } else {
                Mode::Watermark { delta: v[2] }
            };
            model = Some(Model::new(mode, -20., 325., v[0], v[1]).unwrap());
        } else {
            let s = model
                .as_mut()
                .unwrap()
                .update(v[0], v[1], v[2], v[3] != 0.)
                .unwrap();
            println!(
                "{:.17e} {:.17e} {}",
                s.modeled_duty,
                s.smoothed_temp,
                u8::from(s.busy)
            );
        }
    }
}
