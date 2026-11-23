// SPDX-License-Identifier: GPL-3.0-only
//! Only an attested GPIO-free fixture may receive this virtual motion.
use crate::{gpio_dual::Peer, linux::Link};
use klipper_host_protocol::{
    config,
    extrusion::{Extruder, Settings},
    identify::Dictionary,
    machine_preview::{
        mixed::{ToolInput, ToolSlot},
        Machine, SimulatedExtruder, ToolLane,
    },
    simulated_printer::mixed::{Program, State, ToolOids},
    step_wire::{Message, StepMessages},
    thermal::commands::{Bank, Gate, Observation, Profile, Role, Source},
};
use std::collections::BTreeMap;

const EXPECTED: [[i32; 5]; 3] = [
    [240, 240, 0, 25, 0],
    [560, -80, 0, 25, 50],
    [640, 0, 0, 35, 50],
];

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

pub(super) fn run(link: Link, dictionary: &Dictionary, fail_second: bool) -> Result<(), String> {
    let mut peer = Peer::from_identified(link, dictionary)?;
    let hz = dictionary
        .config
        .get("CLOCK_FREQ")
        .and_then(serde_json::Value::as_f64)
        .filter(|v| (1e6..=100e6).contains(v))
        .ok_or("unknown simulator frequency")?;
    peer.initial()?;
    let cfg = config::parse(
        &BTreeMap::from([
            (
                "printer.cfg".into(),
                include_str!("../preview/printer.cfg").into(),
            ),
            (
                "rails.cfg".into(),
                include_str!("../preview/rails.cfg").into(),
            ),
        ]),
        "printer.cfg",
    )
    .map_err(err)?;
    let machine = Machine::from_config(&cfg).map_err(err)?;
    let geometry = Extruder::new(Settings {
        nozzle: 0.4,
        filament: 1.75,
        max_area: 0.64,
        retraction_distance: 4.,
        retraction_velocity: 20.,
        retraction_accel: 300.,
        corner_velocity: 1.,
        min_simulated_temperature: 170.,
    })
    .map_err(err)?;
    let profiles = [
        Profile {
            name: "extruder",
            role: Role::Extruder(0),
            minimum: 0.,
            maximum: 300.,
        },
        Profile {
            name: "extruder1",
            role: Role::Extruder(1),
            minimum: 0.,
            maximum: 300.,
        },
    ];
    let bank = Bank::new(&profiles, &[], Some(0), false).map_err(err)?;
    let slot = |id, heater| ToolSlot {
        lane: ToolLane {
            id,
            parked_e: 0.,
            extruder: SimulatedExtruder {
                geometry: &geometry,
                temperature: 210.,
                step_distance: 0.004,
                inverse_direction: false,
                pressure_advance: None,
            },
        },
        heater,
    };
    let slots = [slot(2, 0), slot(5, 1)];
    let codec = StepMessages::from_dictionary(dictionary).map_err(err)?;
    let trace = machine.trace_mixed_tools_at_virtual_origin([1.,1.,0.2],&bank,ToolInput {
        script:"M109 S210\nG90\nM83\nG1 X4 E.1 F800\nT5\nM109 S220\nG1 Y5 E.2 F800\nT2\nG1 X5 E.04 F800\nTURN_OFF_HEATERS\n",
        slots:&slots,initial_tool:2,hz,max_error:20,invert:[false;3],
    }).map_err(err)?;
    let mut program = Program::new_tools(
        trace,
        &codec,
        ToolOids {
            xyz: [0, 1, 2],
            extruders: &[(2, 3), (5, 4)],
        },
        bank.targets(),
        0.5,
    )
    .map_err(err)?;
    let lead = (hz * 0.1) as u32;
    let capacity = usize::from(peer.configure(5, lead, hz)?);
    let tick = |n: f64| (n * hz) as u64;
    let reading = |index, at, value| Observation {
        source: Source::Heater(index),
        sampled_at: at,
        temperature: value,
        controller_busy: false,
    };
    program.start(0).map_err(err)?;
    let mut dispatched = [0usize; 3];
    let mut last_generation = 0;
    for (stage, (active, e_axis, temperature)) in
        [(2, 3, Some(210.)), (5, 4, Some(220.)), (2, 3, None)]
            .into_iter()
            .enumerate()
    {
        let wall = 16. * stage as f64;
        if program.active_tool() != Some(active) {
            return Err("wrong virtual tool before motion phase".into());
        }
        if let Some(temp) = temperature {
            if program.state() != State::Waiting
                || !program
                    .advance(tick(wall + 1.), 256)
                    .map_err(err)?
                    .is_empty()
                || program
                    .temperature(tick(wall + 2.), Some(reading(stage, wall, temp)), false)
                    .map_err(err)?
                    != Gate::Pending
            {
                return Err("stale observation released a mixed-tool phase".into());
            }
            if stage == 1 && fail_second {
                if program
                    .temperature(tick(wall + 3.), None, true)
                    .map_err(err)?
                    != Gate::Shutdown
                    || !program
                        .advance(tick(wall + 12.), 256)
                        .map_err(err)?
                        .is_empty()
                    || program.state() != State::Faulted
                {
                    return Err("failed observation leaked mixed-tool motion".into());
                }
                for oid in 0..5 {
                    if peer.position(oid)? != EXPECTED[0][usize::from(oid)] {
                        return Err("inert step counter changed after failed observation".into());
                    }
                }
                eprintln!("TOOL_THERMAL_FAULT {} 0 0 240 240 0 25 0", dispatched[0]);
                return Ok(());
            }
            if program
                .temperature(
                    tick(wall + 4.),
                    Some(reading(stage, wall + 4., temp)),
                    false,
                )
                .map_err(err)?
                != Gate::Ready
            {
                return Err("fresh observation did not release mixed-tool phase".into());
            }
        }
        if stage > 0 {
            peer.reset_phase(5, lead)?;
        }
        for _ in 0..16 {
            if program.state() != State::Running {
                break;
            }
            for item in program.advance(tick(wall + 12.), 256).map_err(err)? {
                let oid = match codec.decode(&item.command.bytes).map_err(err)? {
                    Message::NextDirection { oid, .. } | Message::Block { oid, .. } => oid,
                };
                if oid > 2 && oid != e_axis {
                    return Err("emitted command for an inactive extruder".into());
                }
                dispatched[stage] += 1;
                if dispatched[stage] > capacity {
                    return Err("simulator move capacity exceeded".into());
                }
                peer.send(&item.command.bytes)?;
            }
        }
        peer.advance(tick(5.))?;
        for oid in 0..5 {
            let got = peer.position(oid)?;
            if got != EXPECTED[stage][usize::from(oid)] {
                return Err(format!(
                    "phase {stage} lane {oid}={got}, expected {:?}",
                    EXPECTED[stage]
                ));
            }
        }
        let mut proof = program
            .pending_motion_commit()
            .ok_or("motion phase did not require proof")?;
        if proof.generation <= last_generation || proof.simulated_tick > tick(5.) {
            return Err("unverified mixed-tool generation or time".into());
        }
        last_generation = proof.generation;
        proof.accepted_commands = dispatched[stage];
        proof.simulated_tick = tick(5.);
        program
            .commit_motion(tick(wall + 13.), proof)
            .map_err(err)?;
    }
    if program.state() != State::Finished || program.targets() != [0., 0.] {
        return Err("virtual sparse program did not finish with cleared heater targets".into());
    }
    eprintln!(
        "TOOL_THERMAL {} {} {} 640 0 0 35 50",
        dispatched[0], dispatched[1], dispatched[2]
    );
    Ok(())
}
