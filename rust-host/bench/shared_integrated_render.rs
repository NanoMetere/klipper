// SPDX-License-Identifier: GPL-3.0-only
use klipper_host_protocol::{
    extrusion::{Extruder, Settings},
    lookahead::Limits,
    shared_debug_plan::{trace, Options, OutputOptions},
    step_clock::Command,
};
use std::io::{self, Read};
fn main() {
    let max_error_ticks: u32 = std::env::args().nth(1).unwrap().parse().unwrap();
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
    let output = trace(
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
        OutputOptions {
            step_distances: [40. / 3200., 40. / 3200., 8. / 3200., 22.17 / 3200.],
            invert: [false; 4],
            clock_hz: 50_000_000.,
            max_error_ticks,
        },
    )
    .unwrap();
    for lane in output.lanes {
        for command in lane {
            match command {
                Command::Steps(b) => {
                    println!("s {} {} {} {}", b.interval, b.count, b.add, b.last_clock)
                }
                Command::SetDirection(v) => println!("d {}", v as u8),
            }
        }
        println!("end");
    }
}
