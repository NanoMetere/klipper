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
    machine_preview::{Machine, SimulatedExtruder, ToolLane, ToolSegment},
    mcu_config::Messages,
    pressure_advance::Advance,
    simulated_printer::{Input, Simulation},
    step_clock::{Clock, Command},
    step_wire::StepMessages,
    virtual_job::{Issued, Job, Snapshot, State},
    virtual_stepper::Stepper,
    MAX_PAYLOAD,
};
use std::{collections::BTreeMap, time::Instant};

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

enum Run {
    Single(Box<Simulation>),
    Multiple(Job),
}

impl Run {
    fn snapshot(&self) -> Snapshot {
        match self {
            Self::Single(sim) => sim.snapshot(),
            Self::Multiple(job) => job.snapshot(),
        }
    }

    fn start(&mut self) -> Result<(), String> {
        match self {
            Self::Single(sim) => sim.start(0).map_err(err),
            Self::Multiple(job) => job.start(0).map_err(err),
        }
    }

    fn advance(&mut self, end: u64) -> Result<Vec<Issued>, String> {
        match self {
            Self::Single(sim) => sim.advance(end, 256).map_err(err),
            Self::Multiple(job) => job.poll(end, 256).map_err(err),
        }
    }
}

pub(super) fn run(
    mut link: Link,
    dictionary: &Dictionary,
    extrusion: bool,
    batch: bool,
    multitool: bool,
) -> Result<(), String> {
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
    let count = if multitool {
        5
    } else if extrusion {
        4
    } else {
        3
    };
    let oids = [0, 1, 2, 3, 4];
    let make_ex = || SimulatedExtruder {
        geometry: &geometry,
        temperature: 210.,
        step_distance: 0.004,
        inverse_direction: false,
        pressure_advance: Some(&advance),
    };
    let mut simulation = if multitool {
        let pieces = [
            ToolSegment {
                tool: 0,
                gcode: include_str!("../preview/multitool-a.gcode"),
            },
            ToolSegment {
                tool: 1,
                gcode: include_str!("../preview/multitool-b.gcode"),
            },
            ToolSegment {
                tool: 0,
                gcode: include_str!("../preview/multitool-c.gcode"),
            },
        ];
        let lanes = [
            ToolLane {
                id: 0,
                parked_e: 0.,
                extruder: make_ex(),
            },
            ToolLane {
                id: 1,
                parked_e: 1.3,
                extruder: make_ex(),
            },
        ];
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.,
            hz,
        };
        let trace = machine
            .trace_tool_segments(&home, &pieces, &lanes, 2.0, clock, 20, [false; 3])
            .map_err(err)?;
        let traces: Vec<_> = trace
            .motors
            .iter()
            .chain(trace.tools.iter().map(|(_, t)| t))
            .map(Vec::as_slice)
            .collect();
        let trailing = traces
            .iter()
            .flat_map(|lane| lane.iter())
            .filter_map(|c| match c {
                Command::Steps(b) => Some(b.last_clock),
                _ => None,
            })
            .max()
            .unwrap_or_default();
        let end = (((2.0 + trace.duration) * hz).ceil() as u64).max(trailing) + 1;
        Run::Multiple(Job::for_lanes(&wire, &traces, &oids[..count], 0, end, 1).map_err(err)?)
    } else {
        Run::Single(Box::new(
            Simulation::prepare(
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
                    extruder: extrusion.then(make_ex),
                    hz,
                },
            )
            .map_err(err)?,
        ))
    };
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
    simulation.start()?;
    let mut virtual_motors = oids.map(|oid| Stepper::new(oid, 0, 0));
    let mut frame_count = 0usize;
    let mut payload = Vec::with_capacity(MAX_PAYLOAD);
    let enqueue = Instant::now();
    let mut send = |data: &[u8]| -> Result<(), String> {
        let frame = encode(link.sequence, data).map_err(err)?;
        fixture_ack(&mut link, &frame).map_err(|e| format!("motion frame {frame_count}: {e}"))?;
        frame_count += 1;
        Ok(())
    };
    while simulation.snapshot().state != State::Finished {
        for item in simulation.advance(planned.planned_end)? {
            virtual_motors[item.lane]
                .accept_payload(&wire, &item.command.bytes)
                .map_err(err)?;
            let command = &item.command.bytes;
            if command.is_empty() || command.len() > MAX_PAYLOAD {
                return Err("invalid fixture motion payload size".into());
            }
            if batch {
                if payload.len() + command.len() > MAX_PAYLOAD {
                    send(&payload)?;
                    payload.clear();
                }
                payload.extend_from_slice(command);
            } else {
                send(command)?;
            }
        }
    }
    if !payload.is_empty() {
        send(&payload)?;
    }
    let enqueue_ns = enqueue.elapsed().as_nanos();

    let mut positions = [0; 5];
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
    eprintln!("ENQUEUE {frame_count} {enqueue_ns}");
    Ok(())
}
