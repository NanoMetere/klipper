// SPDX-License-Identifier: GPL-3.0-only
//! Runs fixed steps only against the attested, no-GPIO fixture firmware.

use crate::linux::{fixture_ack, read_clock, read_config, Link};
use klipper_host_protocol::{
    clock::{ClockMessages, Response},
    config, encode,
    fixture_mcu::{Codec, Error as CodecError},
    gcode_macros::Registry,
    homing::Counter,
    identify::Dictionary,
    machine_preview::Machine,
    mcu_config::Messages,
    simulated_printer::{Input, Simulation},
    step_wire::StepMessages,
    virtual_job::State,
    virtual_stepper::Stepper,
};
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

pub(super) fn run(mut link: Link, dictionary: &Dictionary) -> Result<(), String> {
    // Fail before any configuration or motion command to an unmarked peer.
    let motion = Codec::from_dictionary(dictionary).map_err(err)?;
    let config = Messages::from_dictionary(dictionary).map_err(err)?;
    let clocks = ClockMessages::from_dictionary(dictionary).map_err(err)?;
    let wire = StepMessages::from_dictionary(dictionary).map_err(err)?;
    let hz = dictionary
        .config
        .get("CLOCK_FREQ")
        .and_then(serde_json::Value::as_f64)
        .filter(|v| (1e6..=250e6).contains(v))
        .ok_or("unexpected firmware clock")?;
    let initial = read_config(&mut link, &config)?;
    if initial.configured || initial.shutdown {
        return Err("fixture requires an unconfigured peer".into());
    }

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
    let mut simulation = Simulation::prepare(
        &machine,
        &home,
        Input {
            name: "macros.gcode",
            script: include_str!("../preview/path_macro_tail.gcode"),
            macros: Some(&macros),
            discovered: &wire,
            oids: [0, 1, 2],
            hz,
        },
    )
    .map_err(err)?;
    let planned = simulation.snapshot();
    let (crc, items) = motion.setup((hz * 0.000_002).ceil() as u32).map_err(err)?;
    for item in items {
        let frame = encode(link.sequence, &item).map_err(err)?;
        fixture_ack(&mut link, &frame)?;
    }
    let cfg_state = read_config(&mut link, &config)?;
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
    for oid in 0..3 {
        let reset = motion.reset_clock(link.sequence, oid, start).map_err(err)?;
        fixture_ack(&mut link, &reset)?;
    }
    simulation.start(0).map_err(err)?;
    let mut virtual_motors = [0, 1, 2].map(|oid| Stepper::new(oid, 0, 0));
    let enqueue = Instant::now();
    while simulation.snapshot().state != State::Finished {
        if enqueue.elapsed() >= Duration::from_millis(800) {
            return Err("motion fixture setup exceeded its lead time".into());
        }
        for item in simulation.advance(planned.planned_end, 256).map_err(err)? {
            virtual_motors[item.lane]
                .accept_payload(&wire, &item.command.bytes)
                .map_err(err)?;
            let frame = encode(link.sequence, &item.command.bytes).map_err(err)?;
            fixture_ack(&mut link, &frame)?;
        }
    }

    let until = Instant::now() + Duration::from_secs(5);
    loop {
        let Response::Clock(now) = read_clock(&mut link, &clocks, false)?.value else {
            return Err("no fixture clock".into());
        };
        if now.wrapping_sub(start) as i32 >= planned.planned_end as i32 {
            break;
        }
        if Instant::now() >= until {
            return Err("timed out waiting for fixture motion".into());
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let mut positions = [0; 3];
    for oid in 0..3 {
        let response = link.exchange(
            |seq| motion.get_position(seq, oid).map_err(err),
            |frame| match motion.position(frame) {
                Ok((target, pos)) if target == oid => Ok(Some(pos)),
                Ok(_) | Err(CodecError::UnknownMessage) => Ok(None),
                Err(e) => Err(err(e)),
            },
        )?;
        let expected = virtual_motors[usize::from(oid)]
            .advance(planned.planned_end)
            .map_err(err)?
            .position_steps;
        if i64::from(response.value) != expected {
            return Err(format!(
                "firmware motion mismatch for {oid}: {} != {expected}",
                response.value
            ));
        }
        positions[usize::from(oid)] = response.value;
    }
    if !read_config(&mut link, &config)?.configured {
        return Err("fixture lost configuration".into());
    }
    eprintln!(
        "GPIO_FIXTURE {crc} {} {} {} {}",
        planned.total, positions[0], positions[1], positions[2]
    );
    Ok(())
}
