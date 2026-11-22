// SPDX-License-Identifier: GPL-3.0-only
//! Runs fixed steps only against the attested, no-GPIO fixture firmware.

use crate::linux::{fixture_ack, read_clock, read_config, Link};
use klipper_host_protocol::{
    clock::{ClockMessages, Response},
    config, encode,
    extrusion::{Extruder, Settings},
    fixture_mcu::{Codec, Error as CodecError, ShutdownWatch},
    gcode_macros::Registry,
    homing::Counter,
    identify::Dictionary,
    machine_preview::{Machine, SimulatedExtruder},
    mcu_config::Messages,
    pressure_advance::Advance,
    simulated_printer::{Input, Simulation},
    step_wire::StepMessages,
    virtual_job::State,
    virtual_stepper::Stepper,
};
use std::collections::BTreeMap;

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

pub(super) fn run(mut link: Link, dictionary: &Dictionary, extrusion: bool) -> Result<(), String> {
    // Fail before any configuration or motion command to an unmarked peer.
    let motion = Codec::from_dictionary(dictionary).map_err(err)?;
    link.shutdown_watch = ShutdownWatch::new(dictionary);
    let config = Messages::from_dictionary(dictionary).map_err(err)?;
    let clocks = ClockMessages::from_dictionary(dictionary).map_err(err)?;
    let wire = StepMessages::from_dictionary(dictionary).map_err(err)?;
    let hz = dictionary
        .config
        .get("CLOCK_FREQ")
        .and_then(serde_json::Value::as_f64)
        .filter(|v| (1e6..=250e6).contains(v))
        .ok_or("unexpected firmware clock")?;
    let initial = read_config(&mut link, &config).map_err(|e| format!("initial config: {e}"))?;
    if initial.configured || initial.shutdown {
        return Err("fixture requires an unconfigured peer".into());
    }

    let mut printer = include_str!("../preview/printer.cfg").to_owned();
    if extrusion {
        printer.push_str(
            "\n[input_shaper]\nshaper_freq_x: 56.6\nshaper_type_y: ei\nshaper_freq_y: 48.4\n",
        );
    }
    let cfg = config::parse(
        &BTreeMap::from([
            ("printer.cfg".into(), printer),
            (
                "rails.cfg".into(),
                include_str!("../preview/rails.cfg").into(),
            ),
        ]),
        "printer.cfg",
    )
    .map_err(err)?;
    let machine = Machine::from_config(&cfg).map_err(err)?;
    let macros = Registry::from_config(&cfg).map_err(err)?;
    let virtual_start = [1., 1., 0.2];
    let mut home = machine.virtual_homing(virtual_start).map_err(err)?;
    let reading = Some(Counter {
        start: 8,
        trigger: 4,
        halt: 4,
    });
    for (axis, coordinate) in virtual_start.into_iter().enumerate() {
        let token = home.arm(axis as u8, coordinate, true).map_err(err)?;
        let counters = if axis < 2 {
            [reading, reading, None]
        } else {
            [None, None, reading]
        };
        home.complete(token, counters, true).map_err(err)?;
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
    .map_err(err)?;
    let advance = Advance::new(0.04, &[(0.0, 0.04)]).map_err(err)?;
    let count = if extrusion { 4 } else { 3 };
    let oids = [0, 1, 2, 3];
    let mut simulation = Simulation::prepare(
        &machine,
        &home,
        Input {
            name: "macros.gcode",
            script: if extrusion {
                include_str!("../preview/path_extrusion.gcode")
            } else {
                include_str!("../preview/path_macro_tail.gcode")
            },
            macros: Some(&macros),
            discovered: &wire,
            oids: &oids[..count],
            extruder: extrusion.then_some(SimulatedExtruder {
                geometry: &geometry,
                temperature: 210.,
                step_distance: 0.004,
                inverse_direction: false,
                pressure_advance: Some(&advance),
            }),
            hz,
        },
    )
    .map_err(err)?;
    let planned = simulation.snapshot();
    let (crc, items) = motion
        .setup(count as u8, (hz * 0.000_002).ceil() as u32)
        .map_err(err)?;
    for item in items {
        let frame = encode(link.sequence, &item).map_err(err)?;
        fixture_ack(&mut link, &frame)?;
    }
    let cfg_state = read_config(&mut link, &config).map_err(|e| format!("finalize config: {e}"))?;
    if !cfg_state.configured
        || cfg_state.shutdown
        || cfg_state.crc != crc
        || usize::from(cfg_state.move_count) < planned.total
        || planned.planned_end > i32::MAX as u64
    {
        return Err(format!(
            "unsafe fixture setup or motion capacity: {cfg_state:?}"
        ));
    }
    let Response::Clock(now) = read_clock(&mut link, &clocks, false)?.value else {
        return Err("missing fixture clock".into());
    };
    let start = now.wrapping_add((hz * 0.1) as u32);
    for &oid in &oids[..count] {
        let reset = motion.reset_clock(link.sequence, oid, start).map_err(err)?;
        fixture_ack(&mut link, &reset)?;
    }
    simulation.start(0).map_err(err)?;
    let mut virtual_motors = oids.map(|oid| Stepper::new(oid, 0, 0));
    while simulation.snapshot().state != State::Finished {
        for item in simulation.advance(planned.planned_end, 256).map_err(err)? {
            virtual_motors[item.lane]
                .accept_payload(&wire, &item.command.bytes)
                .map_err(err)?;
            let frame = encode(link.sequence, &item.command.bytes).map_err(err)?;
            fixture_ack(&mut link, &frame)?;
        }
    }

    let mut positions = [0; 4];
    let lead = (hz * 2.0) as u64;
    if planned.planned_end < lead {
        return Err("fixture planned end is before motion lead".into());
    }
    let range = planned.planned_end - lead;
    for index in 0..=16 {
        let relative = lead + range * index / 16;
        let at = start.wrapping_add(relative as u32);
        let advance = motion.advance_time(link.sequence, at).map_err(err)?;
        if let Err(e) = fixture_ack(&mut link, &advance) {
            let reason = link
                .shutdown_watch
                .as_ref()
                .and_then(|w| w.description(dictionary));
            return Err(format!(
                "advance fixture clock at checkpoint {index}: {e}; {reason:?}"
            ));
        }
        let after =
            read_config(&mut link, &config).map_err(|e| format!("post-motion config: {e}"))?;
        if !after.configured || after.shutdown || after.crc != crc {
            return Err(format!(
                "software firmware did not remain ready at checkpoint {index}: {after:?}, {:?}",
                link.shutdown_watch
                    .as_ref()
                    .and_then(|w| w.description(dictionary))
            ));
        }
        for &oid in &oids[..count] {
            let response = link
                .exchange(
                    |seq| motion.get_position(seq, oid).map_err(err),
                    |frame| match motion.position(frame) {
                        Ok((target, pos)) if target == oid => Ok(Some(pos)),
                        Ok(_) | Err(CodecError::UnknownMessage) => Ok(None),
                        Err(e) => Err(err(e)),
                    },
                )
                .map_err(|e| format!("position {oid}: {e}"))?;
            let expected = virtual_motors[usize::from(oid)]
                .advance(relative)
                .map_err(err)?
                .position_steps;
            if i64::from(response.value) != expected {
                return Err(format!(
                    "firmware motion mismatch for {oid} at checkpoint {index}: {} != {expected}",
                    response.value
                ));
            }
            positions[usize::from(oid)] = response.value;
        }
    }
    eprint!("GPIO_FIXTURE {crc} {}", planned.total);
    for position in &positions[..count] {
        eprint!(" {position}");
    }
    eprintln!();
    Ok(())
}
