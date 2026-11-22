// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{gcode::Preview, lookahead::Limits, toolchange::ExtruderPositions};
use std::io::{self, BufRead};

fn main() {
    let limits = Limits::new(172., 3420., 5.7, 0.63).unwrap();
    let mut p = Preview::new(limits, 256, Some(1.)).unwrap();
    let mut extruders = ExtruderPositions::new(&[(0, 0.), (1, 1.3), (2, -0.5)], 0).unwrap();
    for row in io::stdin().lock().lines() {
        let row = row.unwrap();
        if row == "!reset_tools" {
            p = Preview::new(limits, 256, Some(1.)).unwrap();
            extruders = ExtruderPositions::new(&[(0, 0.), (1, 1.3), (2, -0.5)], 0).unwrap();
        } else if let Some(id) = row.strip_prefix("!use ") {
            extruders.switch(id.parse().unwrap(), &mut p).unwrap();
        } else if row == "!state" {
            print!(
                "{} {:.17e} {:.17e}",
                extruders.active(),
                p.coords().gcode_position()[3],
                p.physical_position()[3]
            );
            for lane in 0..3 {
                print!(" {:.17e}", extruders.observed_position(lane, &p).unwrap());
            }
            println!();
        } else {
            p.line(&row).unwrap();
        }
    }
}
