// SPDX-License-Identifier: GPL-3.0-only
//! Offline dock-state recipe for reference verification.

use klipper_host_protocol::{
    gcode::{Preview, TransformOwner},
    lookahead::Limits,
    toolchange::Dock,
};
use std::io::{self, BufRead};

fn main() {
    for input in io::stdin().lock().lines() {
        let input = input.unwrap();
        let mut fields = input.split_whitespace();
        let mask = fields.next().unwrap().parse().unwrap();
        let mut vector =
            || std::array::from_fn::<_, 3, _>(|_| fields.next().unwrap().parse::<f64>().unwrap());
        let old = vector();
        let new = vector();
        let [x, y, z] = vector();
        let [a, b, c] = vector();
        assert!(fields.next().is_none());
        let owner = TransformOwner(1);
        let limits = Limits::new(172.0, 3420.0, 5.7, 0.63).unwrap();
        let mut p = Preview::new(limits, 200, Some(1.0)).unwrap();
        p.wrap_translation(owner, old, true).unwrap();
        p.reset_last_position().unwrap();
        p.line(&format!("G1 X{x:.13} Y{y:.13} Z{z:.13} E3 F1500"))
            .unwrap();
        p.line("SET_GCODE_OFFSET X=.3 Y=-.1 Z=.12").unwrap();
        p.line("M221 S110").unwrap();
        let dock = Dock::enter(p, owner, mask)
            .unwrap()
            .preview_lines(&format!(
                "G90\nG0 X{a:.13} Y{b:.13} Z{c:.13}\nG91\nM83\nG1 E-.3 F600"
            ))
            .unwrap();
        let (mut p, _) = dock.finish(new).unwrap();
        p.line("G1 E3.1").unwrap();
        p.line("M400").unwrap();
        for v in p
            .coords()
            .position()
            .into_iter()
            .chain(p.coords().gcode_position())
            .chain(p.coords().homing_origin())
            .chain(p.physical_position())
            .chain([p.coords().physical_feed_rate()])
        {
            print!("{v:.17e} ");
        }
        println!();
    }
}
