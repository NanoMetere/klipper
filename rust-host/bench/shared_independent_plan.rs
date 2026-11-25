// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    extrusion::{Extruder, Settings},
    lookahead::Limits,
    shared_debug_plan::{plan, Event, Options},
};
use std::io::{self, Read};
fn main() {
    let xyz = std::env::args().nth(1).is_some_and(|s| s == "--xyz");
    let mut script = String::new();
    io::stdin().read_to_string(&mut script).unwrap();
    let area = 4. * 0.4_f64.powi(2);
    let ratio = area / (std::f64::consts::PI * 0.875_f64.powi(2));
    let geometry = Extruder::new(Settings {
        nozzle: 0.4,
        filament: 1.75,
        max_area: area,
        retraction_distance: 50.,
        retraction_velocity: 250. * ratio,
        retraction_accel: 4000. * ratio,
        corner_velocity: 1.,
        min_simulated_temperature: 170.,
    })
    .unwrap();
    let events = plan(
        &script,
        [10., 10., 1.],
        Options {
            limits: Limits::new(250., 4000., 5., 0.5).unwrap(),
            z_velocity: 15.,
            z_accel: 200.,
            xyz_bounds: [[0., 300.]; 3],
            simulation_temp: [210.; 2],
            smooth: 0.04,
            initial_advance: 0.04,
        },
        [&geometry, &geometry],
    )
    .unwrap();
    if xyz {
        println!("b 10 10 1");
    }
    for event in events {
        match event {
            Event::Profile {
                at,
                queue,
                movement: p,
            } => {
                let r = p.direction[3];
                if xyz && p.kinematic {
                    print!("a {at:.17e}");
                    for v in [
                        p.accel_t,
                        p.cruise_t,
                        p.decel_t,
                        p.start[0],
                        p.start[1],
                        p.start[2],
                        p.direction[0],
                        p.direction[1],
                        p.direction[2],
                        p.start_v,
                        p.cruise_v,
                        p.accel,
                    ] {
                        print!(" {v:.17e}");
                    }
                    println!();
                } else if !xyz && r != 0. {
                    let pa = (r > 0. && (p.direction[0] != 0. || p.direction[1] != 0.)) as u8;
                    println!("a {queue} {pa} {at:.17e} {:.17e} {:.17e} {:.17e} {:.17e} {:.17e} {:.17e} {:.17e}",
                        p.accel_t,p.cruise_t,p.decel_t,p.start[3],p.start_v*r,p.cruise_v*r,p.accel*r);
                }
            }
            Event::Gain { at, advance } if !xyz => println!("p {at:.17e} {advance:.17e}"),
            Event::Generation { at, retirement } => {
                println!("f {at:.17e}");
                if xyz {
                    println!("t {retirement:.17e}");
                } else {
                    for q in 0..2 {
                        println!("t {q} {retirement:.17e} 0");
                    }
                }
            }
            Event::Bind { queue, anchor } if !xyz => {
                println!("b {} {anchor:.17e}", queue.map_or(-1, i32::from))
            }
            _ => {}
        }
    }
}
