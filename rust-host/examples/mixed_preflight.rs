// SPDX-License-Identifier: GPL-3.0-only
//! Dump inert motion stages and heater boundaries for stock comparison.
use klipper_host_protocol::{
    gcode::Preview,
    lookahead::Limits,
    thermal::{
        commands::{Bank, Profile, Role},
        planned::{preflight, Stage},
    },
};
use std::io::{self, Read};

fn main() {
    let profiles = [
        Profile {
            name: "extruder",
            role: Role::Extruder(0),
            minimum: 0.,
            maximum: 300.,
        },
        Profile {
            name: "heater_bed",
            role: Role::Bed,
            minimum: 0.,
            maximum: 130.,
        },
    ];
    let bank = Bank::new(&profiles, &[], Some(0), false).unwrap();
    let mut all = String::new();
    io::stdin().lock().read_to_string(&mut all).unwrap();
    for text in all.split("\n--case--\n") {
        let preview =
            Preview::new(Limits::new(160., 2500., 5., 0.5).unwrap(), 4096, Some(1.)).unwrap();
        let result = preflight(preview, &bank, text, |_, _, _| Ok(())).unwrap();
        for stage in result.stages {
            match stage {
                Stage::Heater { line, .. } => println!("heater {line}"),
                Stage::Motion {
                    first_line,
                    outputs,
                    ..
                } => {
                    println!("motion {first_line}");
                    for output in outputs {
                        for m in output.moves {
                            print!("move {}", u8::from(m.kinematic));
                            for value in
                                m.start.into_iter().chain(m.end).chain(m.direction).chain([
                                    m.start_v, m.cruise_v, m.end_v, m.accel, m.accel_t, m.cruise_t,
                                    m.decel_t,
                                ])
                            {
                                print!(" {value:.17e}");
                            }
                            println!();
                        }
                        if let Some(t) = output.dwell_seconds {
                            println!("dwell {t:.17e}");
                        }
                    }
                }
            }
        }
        println!("next");
    }
}
