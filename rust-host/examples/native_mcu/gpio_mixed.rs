// SPDX-License-Identifier: GPL-3.0-only
//! Exercise isolated heater boundaries on an attested no-GPIO MCU.
//! The temperatures and sensor identities are explicitly synthetic.

use crate::{gpio_dual::Peer, linux::Link};
use klipper_host_protocol::{
    config,
    extrusion::{Extruder, Settings},
    homing::Counter,
    identify::Dictionary,
    machine_preview::{mixed::Input, Machine, SimulatedExtruder},
    simulated_printer::mixed::{Program, State},
    step_wire::StepMessages,
    thermal::commands::{Bank, Gate, Observation, Profile, Role, Source},
};
use std::collections::BTreeMap;

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

pub(super) fn run(
    link: Link,
    dictionary: &Dictionary,
    shutdown_after_first: bool,
) -> Result<(), String> {
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
    let pos = [1., 1., 0.2];
    let mut home = machine.virtual_homing(pos).map_err(err)?;
    for (axis, position) in pos.into_iter().enumerate() {
        let mark = Some(Counter {
            start: 10,
            trigger: 9,
            halt: 9,
        });
        let arm = home.arm(axis as u8, position, true).map_err(err)?;
        home.complete(
            arm,
            if axis < 2 {
                [mark, mark, None]
            } else {
                [None, None, mark]
            },
            true,
        )
        .map_err(err)?;
    }
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
            name: "heater_bed",
            role: Role::Bed,
            minimum: 0.,
            maximum: 130.,
        },
    ];
    let bank = Bank::new(&profiles, &[], Some(0), false).map_err(err)?;
    let wire = StepMessages::from_dictionary(dictionary).map_err(err)?;
    let trace=machine.trace_mixed(&home,&bank,Input{
        script:"M109 S210\nG90\nM83\nG1 X4 E.1 F800\nM190 S60\nG1 Y5 E.2 F800\nTURN_OFF_HEATERS\n",
        extruder:Some(SimulatedExtruder{geometry:&geometry,temperature:210.,step_distance:0.004,
                inverse_direction:false,pressure_advance:None}),
        hz,max_error:20,invert:[false;3],
    }).map_err(err)?;
    let mut program = Program::new(trace, &wire, &[0, 1, 2, 3], bank.targets(), 0.5)
        .map_err(err)?
        .require_motion_commit()
        .map_err(err)?;
    let lead = (hz * 0.1) as u32;
    let capacity = peer.configure(4, lead, hz)?;
    let tick = |n: f64| (n * hz) as u64;
    program.start(0).map_err(err)?;
    let reading = |index, sampled_at, temperature| Observation {
        source: Source::Heater(index),
        sampled_at,
        temperature,
        controller_busy: false,
    };
    let mut dispatched = [0; 2];
    let expected = [[240, 240, 0, 25], [560, -80, 0, 75]];
    for (stage, (line, temperature)) in [(1, 210.), (5, 60.)].into_iter().enumerate() {
        let wall = 16. * stage as f64;
        if program.state() != State::Waiting
            || program.line() != Some(line)
            || !program
                .advance(tick(wall + 1.), 256)
                .map_err(err)?
                .is_empty()
            || program
                .temperature(
                    tick(wall + 2.),
                    Some(reading(stage, wall, temperature)),
                    false,
                )
                .map_err(err)?
                != Gate::Pending
            || !program
                .advance(tick(wall + 3.), 256)
                .map_err(err)?
                .is_empty()
        {
            return Err("a heater barrier leaked motion without a fresh sample".into());
        }
        if stage == 1 && shutdown_after_first {
            if program
                .temperature(tick(wall + 4.), None, true)
                .map_err(err)?
                != Gate::Shutdown
                || program.state() != State::Faulted
                || !program
                    .advance(tick(wall + 12.), 256)
                    .map_err(err)?
                    .is_empty()
            {
                return Err("failed virtual heat wait released later motion".into());
            }
            for oid in 0..4 {
                let value = peer.position(oid)?;
                if value != expected[0][usize::from(oid)] {
                    return Err("inert motor moved during the failed wait".into());
                }
            }
            eprintln!("THERMAL_FIXTURE_FAULT {} 0 240 240 0 25", dispatched[0]);
            return Ok(());
        }
        let gate = program
            .temperature(
                tick(wall + 4.),
                Some(reading(stage, wall + 4., temperature)),
                false,
            )
            .map_err(err)?;
        if gate != Gate::Ready {
            return Err("synthetic matching sensor did not release the phase".into());
        }
        if stage > 0 {
            peer.reset_phase(4, lead)?;
        }
        for _ in 0..16 {
            if program.state() != State::Running {
                break;
            }
            let batch = program.advance(tick(wall + 12.), 256).map_err(err)?;
            for item in batch {
                dispatched[stage] += 1;
                if dispatched[stage] > usize::from(capacity) {
                    return Err("fixture move capacity exceeded".into());
                }
                peer.send(&item.command.bytes)?;
            }
        }
        if program.state() == State::Running {
            return Err("mixed fixture phase did not drain within bounds".into());
        }
        peer.advance(tick(5.))
            .map_err(|e| format!("phase {stage} clock: {e}"))?;
        for oid in 0..4 {
            let got = peer.position(oid)?;
            if got != expected[stage][usize::from(oid)] {
                return Err(format!(
                    "phase {stage} lane {oid}={got}, expected {:?}",
                    expected[stage]
                ));
            }
        }
        let mut proof = program
            .pending_motion_commit()
            .ok_or("ended phase not awaiting MCU proof")?;
        let full_tick = tick(5.);
        if proof.simulated_tick > full_tick {
            return Err("fixture phase was longer than its observed test advance".into());
        }
        proof.accepted_commands = dispatched[stage];
        proof.simulated_tick = full_tick;
        program
            .commit_motion(tick(wall + 13.), proof)
            .map_err(err)?;
    }
    if program.state() != State::Finished || program.targets() != [0., 0.] {
        return Err("virtual job did not finish with heater targets cleared".into());
    }
    eprintln!(
        "THERMAL_FIXTURE {} {} 560 -80 0 75",
        dispatched[0], dispatched[1]
    );
    Ok(())
}
