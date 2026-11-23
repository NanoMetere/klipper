// SPDX-License-Identifier: GPL-3.0-only
//! Read-only comparator for signed step totals across sparse heater phases.
use klipper_host_protocol::{
    config,
    extrusion::{Extruder, Settings},
    machine_preview::{
        mixed::{ToolInput, ToolSlot, ToolStage},
        Machine, SimulatedExtruder, ToolLane,
    },
    pressure_advance::Advance,
    step_clock::Command,
    thermal::commands::{Bank, Profile, Role},
};
use std::{
    collections::BTreeMap,
    io::{self, Read},
};

fn signed(commands: &[Command]) -> i64 {
    let (mut sign, mut count) = (0, 0);
    for c in commands {
        match c {
            Command::SetDirection(on) => sign = if *on { 1 } else { -1 },
            Command::Steps(b) => count += i64::from(b.count) * sign,
        }
    }
    count
}

fn main() {
    let config = std::fs::read_to_string(std::env::args_os().nth(1).unwrap()).unwrap();
    let cfg = config::parse(&BTreeMap::from([("main.cfg".into(), config)]), "main.cfg").unwrap();
    let machine = Machine::from_config(&cfg).unwrap();
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
    let ids = [0, 2, 6];
    let park = [0., 1.3, 2.1];
    let profile_names = ["extruder", "extruder1", "extruder2"];
    let profiles: [_; 3] = std::array::from_fn(|i| Profile {
        name: profile_names[i],
        role: Role::Extruder(i as u32),
        minimum: 0.,
        maximum: 300.,
    });
    let bank = Bank::new(&profiles, &[], Some(0), false).unwrap();
    let slots: [_; 3] = std::array::from_fn(|i| ToolSlot {
        lane: ToolLane {
            id: ids[i],
            parked_e: park[i],
            extruder: SimulatedExtruder {
                geometry: &geometry,
                temperature: 210.,
                step_distance: 0.004,
                inverse_direction: i == 1,
                pressure_advance: Some(&advance),
            },
        },
        heater: i as u32,
    });
    let mut corpus = String::new();
    io::stdin().read_to_string(&mut corpus).unwrap();
    for code in corpus.split("\n--case--\n") {
        let plan = machine
            .trace_mixed_tools_at_virtual_origin(
                [10., 10., 0.3],
                &bank,
                ToolInput {
                    script: code,
                    slots: &slots,
                    initial_tool: 0,
                    print_fan: false,
                    hz: 20e6,
                    max_error: 0,
                    invert: [false; 3],
                },
            )
            .unwrap();
        let mut all = [0i64; 6];
        let mut active = 0;
        let mut line = 0;
        for phase in plan.phases {
            let next = match phase {
                ToolStage::Select { line, from, to } => {
                    assert_eq!(from, active);
                    active = to;
                    line
                }
                ToolStage::Fan {
                    line, active_tool, ..
                } => {
                    assert_eq!(active_tool, active);
                    line
                }
                ToolStage::Heater {
                    line, active_tool, ..
                } => {
                    assert_eq!(active_tool, active);
                    line
                }
                ToolStage::Motion {
                    first_line,
                    tool,
                    trace,
                } => {
                    assert_eq!(tool, active);
                    for (i, lane) in trace.motors.iter().enumerate() {
                        all[i] += signed(lane);
                    }
                    let index = ids.iter().position(|t| *t == tool).unwrap();
                    all[index + 3] += signed(trace.extruder.as_ref().unwrap());
                    first_line
                }
            };
            assert!(next > line);
            line = next;
        }
        println!("{}", all.map(|x| x.to_string()).join(" "));
    }
}
