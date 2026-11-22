// SPDX-License-Identifier: GPL-3.0-only
//! Reproduce a fixed multi-extruder motion trace without any hardware.

use klipper_host_protocol::{
    config,
    extrusion::{Extruder, Settings},
    homing::Counter,
    machine_preview::{Machine, SimulatedExtruder, ToolLane, ToolSegment},
    pressure_advance::Advance,
    step_clock::{Clock, Command},
};
use std::collections::BTreeMap;

fn main() {
    let mut printer = include_str!("preview/printer.cfg").to_owned();
    printer.push_str(
        "\n[input_shaper]\nshaper_freq_x: 56.6\nshaper_type_y: ei\nshaper_freq_y: 48.4\n",
    );
    let all = config::parse(
        &BTreeMap::from([
            ("printer.cfg".into(), printer),
            ("rails.cfg".into(), include_str!("preview/rails.cfg").into()),
        ]),
        "printer.cfg",
    )
    .unwrap();
    let machine = Machine::from_config(&all).unwrap();
    let origin = [1., 1., 0.2];
    let mut home = machine.virtual_homing(origin).unwrap();
    let count = Some(Counter {
        start: 5,
        trigger: 4,
        halt: 4,
    });
    for (axis, p) in origin.into_iter().enumerate() {
        let t = home.arm(axis as u8, p, true).unwrap();
        let read = if axis < 2 {
            [count, count, None]
        } else {
            [None, None, count]
        };
        home.complete(t, read, true).unwrap();
    }
    let geometry = Extruder::new(Settings {
        nozzle: 0.4,
        filament: 1.75,
        max_area: 0.64,
        retraction_distance: 15.,
        retraction_velocity: 20.,
        retraction_accel: 300.,
        corner_velocity: 1.,
        min_simulated_temperature: 170.,
    })
    .unwrap();
    let advance = Advance::new(0.04, &[(0., 0.04)]).unwrap();
    let lane = |id, parked_e| ToolLane {
        id,
        parked_e,
        extruder: SimulatedExtruder {
            geometry: &geometry,
            temperature: 210.,
            step_distance: 0.004,
            inverse_direction: false,
            pressure_advance: Some(&advance),
        },
    };
    let pieces = [
        ToolSegment {
            tool: 0,
            gcode: include_str!("preview/multitool-a.gcode"),
        },
        ToolSegment {
            tool: 1,
            gcode: include_str!("preview/multitool-b.gcode"),
        },
        ToolSegment {
            tool: 0,
            gcode: include_str!("preview/multitool-c.gcode"),
        },
    ];
    let trace = machine
        .trace_tool_segments(
            &home,
            &pieces,
            &[lane(0, 0.), lane(1, 1.3)],
            2.,
            Clock {
                last_encoded: 0,
                time_origin: 0.,
                hz: 20e6,
            },
            20,
            [false; 3],
        )
        .unwrap();
    for segment in trace
        .motors
        .into_iter()
        .chain(trace.tools.into_iter().map(|(_, t)| t))
    {
        for cmd in segment {
            match cmd {
                Command::SetDirection(d) => println!("d {}", u8::from(d)),
                Command::Steps(block) => println!(
                    "s {} {} {} {}",
                    block.interval, block.count, block.add, block.last_clock
                ),
            }
        }
        println!("end");
    }
    println!("t {:.17e}", trace.duration);
}
