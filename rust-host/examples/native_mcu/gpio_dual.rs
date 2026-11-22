// SPDX-License-Identifier: GPL-3.0-only
//! Two disposable no-GPIO peers: one independent event clock per PTY.

use crate::linux::{fixture_ack, identify_fixture, read_clock, read_config, Link};
use klipper_host_protocol::{
    clock::{ClockMessages, Response},
    config, encode,
    extrusion::{Extruder, Settings},
    fixture_mcu::{Codec, Error as CodecError, ShutdownWatch},
    homing::Counter,
    identify::Dictionary,
    machine_preview::{DockInput, Machine, MultiTrace, SimulatedExtruder, ToolLane},
    mcu_config::Messages,
    step_clock::{Clock, Command},
    step_wire::StepMessages,
    toolchange::{Catalog, Scripts},
    virtual_job::{Job, RoutedLane, State},
    virtual_stepper::Stepper,
    MAX_PAYLOAD,
};
use std::{collections::BTreeMap, time::Instant};

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

const TOOL_CONFIG: &str = "\n[toolchanger]\n[tool T0]\ntool_number:0\ndetection_pin:A\n[tool T2]\ntool_number:2\ndetection_pin:B\ngcode_x_offset:0.25\n";

fn dock(hz: f64) -> Result<MultiTrace, String> {
    let machine_file = format!("{}{}", include_str!("../preview/printer.cfg"), TOOL_CONFIG);
    let cfg = config::parse(
        &BTreeMap::from([
            ("printer.cfg".into(), machine_file),
            (
                "rails.cfg".into(),
                include_str!("../preview/rails.cfg").into(),
            ),
        ]),
        "printer.cfg",
    )
    .map_err(err)?;
    let catalog = Catalog::from_config(&cfg).map_err(err)?;
    let machine = Machine::from_config(&cfg).map_err(err)?;
    let mut changer = catalog.build().map_err(err)?;
    changer.initialize_from_detection(Some(0)).map_err(err)?;
    let coordinates = [1., 1., 0.2];
    let mut home = machine.virtual_homing(coordinates).map_err(err)?;
    for (axis, at) in coordinates.into_iter().enumerate() {
        let arm = home.arm(axis as u8, at, true).map_err(err)?;
        let hit = Some(Counter {
            start: 8,
            trigger: 5,
            halt: 5,
        });
        home.complete(
            arm,
            if axis < 2 {
                [hit, hit, None]
            } else {
                [None, None, hit]
            },
            true,
        )
        .map_err(err)?;
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
    let lane = |id, parked_e| ToolLane {
        id,
        parked_e,
        extruder: SimulatedExtruder {
            geometry: &geometry,
            temperature: 210.,
            step_distance: 0.004,
            inverse_direction: false,
            pressure_advance: None,
        },
    };
    machine
        .trace_dock(
            &home,
            &mut changer,
            DockInput {
                catalog: &catalog,
                outgoing: lane(0, 0.),
                incoming: lane(2, 1.3),
                detected_after_pickup: Some(2),
                scripts: Scripts {
                    before: "M83\nG1 E-.1 F600",
                    dropoff: "G0 X10 Y12 Z1 F1200",
                    pickup: "M83\nG1 E.15 F600",
                    after: "G1 E.05 F600",
                },
            },
            2.,
            Clock {
                last_encoded: 0,
                time_origin: 0.,
                hz,
            },
            20,
            [false; 3],
        )
        .map_err(err)
}

pub(super) struct Peer {
    link: Link,
    motion: Codec,
    config: Messages,
    clock: ClockMessages,
    start: u32,
    crc: u32,
}

impl Peer {
    pub(super) fn from_identified(mut link: Link, dictionary: &Dictionary) -> Result<Self, String> {
        let motion = Codec::from_dictionary(dictionary).map_err(err)?;
        link.shutdown_watch = ShutdownWatch::new(dictionary);
        Ok(Self {
            link,
            motion,
            config: Messages::from_dictionary(dictionary).map_err(err)?,
            clock: ClockMessages::from_dictionary(dictionary).map_err(err)?,
            start: 0,
            crc: 0,
        })
    }

    pub(super) fn initial(&mut self) -> Result<(), String> {
        let state = read_config(&mut self.link, &self.config)?;
        if state.configured || state.shutdown {
            return Err("multi-peer fixture requires fresh peers".into());
        }
        Ok(())
    }

    pub(super) fn send(&mut self, payload: &[u8]) -> Result<(), String> {
        let frame = encode(self.link.sequence, payload).map_err(err)?;
        fixture_ack(&mut self.link, &frame)
    }

    fn ready(&mut self) -> Result<u16, String> {
        let result = read_config(&mut self.link, &self.config)?;
        if !result.configured || result.shutdown || result.crc != self.crc {
            return Err(format!("peer configuration changed: {result:?}"));
        }
        Ok(result.move_count)
    }

    pub(super) fn configure(&mut self, count: u8, lead: u32, hz: f64) -> Result<u16, String> {
        let (crc, list) = self
            .motion
            .setup(count, (hz * 0.000_002).ceil() as u32)
            .map_err(err)?;
        self.crc = crc;
        for payload in list {
            self.send(&payload)?;
        }
        let capacity = self.ready()?;
        self.reset_phase(count, lead)?;
        Ok(capacity)
    }

    pub(super) fn reset_phase(&mut self, count: u8, lead: u32) -> Result<(), String> {
        let Response::Clock(now) = read_clock(&mut self.link, &self.clock, false)?.value else {
            return Err("missing virtual clock".into());
        };
        self.start = now.wrapping_add(lead);
        for oid in 0..count {
            let frame = self
                .motion
                .reset_clock(self.link.sequence, oid, self.start)
                .map_err(err)?;
            fixture_ack(&mut self.link, &frame)?;
        }
        Ok(())
    }

    pub(super) fn advance(&mut self, relative: u64) -> Result<(), String> {
        let frame = self
            .motion
            .advance_time(self.link.sequence, self.start.wrapping_add(relative as u32))
            .map_err(err)?;
        fixture_ack(&mut self.link, &frame)?;
        self.ready()?;
        Ok(())
    }

    pub(super) fn position(&mut self, oid: u8) -> Result<i32, String> {
        self.link
            .exchange(
                |seq| self.motion.get_position(seq, oid).map_err(err),
                |frame| match self.motion.position(frame) {
                    Ok((target, pos)) if target == oid => Ok(Some(pos)),
                    Ok(_) | Err(CodecError::UnknownMessage) => Ok(None),
                    Err(e) => Err(err(e)),
                },
            )
            .map(|r| r.value)
    }
}

pub(super) fn run(links: [Link; 2], batched: bool) -> Result<(), String> {
    let mut attested = Vec::new();
    let mut dictionaries = Vec::new();
    // Identify and attest both peers before either receives a single config write.
    for mut link in links {
        let dictionary = identify_fixture(&mut link)?;
        attested.push(Peer::from_identified(link, &dictionary)?);
        dictionaries.push(dictionary);
    }
    let hz = dictionaries[0]
        .config
        .get("CLOCK_FREQ")
        .and_then(serde_json::Value::as_f64)
        .filter(|v| (1e6..=250e6).contains(v))
        .ok_or("unknown virtual clock rate")?;
    if dictionaries[0].commands != dictionaries[1].commands
        || dictionaries[1]
            .config
            .get("CLOCK_FREQ")
            .and_then(serde_json::Value::as_f64)
            != Some(hz)
    {
        return Err("the deterministic fixture peers have incompatible dictionaries".into());
    }
    for peer in &mut attested {
        peer.initial()?;
    }
    let wire = StepMessages::from_dictionary(&dictionaries[0]).map_err(err)?;
    let trace = dock(hz)?;
    let parts = trace
        .motors
        .iter()
        .chain(trace.tools.iter().map(|(_, v)| v))
        .map(Vec::as_slice)
        .collect::<Vec<_>>();
    let trailing = parts
        .iter()
        .flat_map(|v| v.iter())
        .filter_map(|cmd| match cmd {
            Command::Steps(b) => Some(b.last_clock),
            _ => None,
        })
        .max()
        .unwrap_or_default();
    let end = (((2. + trace.duration) * hz).ceil() as u64)
        .max(trailing)
        .checked_add(1)
        .filter(|&v| v < i32::MAX as u64)
        .ok_or("fixture horizon")?;
    const ROUTE: [(usize, u8); 5] = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 0)];
    let lanes = std::array::from_fn::<_, 5, _>(|i| RoutedLane {
        peer: ROUTE[i].0 as u8,
        oid: ROUTE[i].1,
        commands: parts[i],
    });
    let mut job = Job::for_routed(&wire, &lanes, 0, end, 1).map_err(err)?;
    let mut motors = ROUTE.map(|(_, oid)| Stepper::new(oid, 0, 0));
    for (i, (peer, count)) in attested.iter_mut().zip([4, 1]).enumerate() {
        let capacity = peer.configure(count, ((0.1 + i as f64 * 0.125) * hz) as u32, hz)?;
        let required = ROUTE
            .iter()
            .enumerate()
            .filter(|(_, (b, _))| *b == i)
            .map(|(n, _)| parts[n].len())
            .sum::<usize>();
        if required > usize::from(capacity) {
            return Err("virtual peer is below motion capacity".into());
        }
    }
    if attested[0].start == attested[1].start {
        return Err("fixture epochs did not differ".into());
    }
    job.start(0).map_err(err)?;
    let mut sent = [0, 0];
    let mut frames = [0, 0];
    let began = Instant::now();
    let mut submit = |board: usize, payload: &[u8]| -> Result<(), String> {
        attested[board]
            .send(payload)
            .map_err(|e| format!("peer {board} motion: {e}"))?;
        frames[board] += 1;
        Ok(())
    };
    let mut frame = [0; MAX_PAYLOAD];
    let mut used = 0;
    let mut owner = 0;
    while job.snapshot().state != State::Finished {
        for item in job.poll(end, 256).map_err(err)? {
            let lane = item.lane;
            let (board, _) = ROUTE[lane];
            let bytes = item.command.bytes.as_slice();
            motors[lane].accept_payload(&wire, bytes).map_err(err)?;
            if bytes.is_empty() || bytes.len() > MAX_PAYLOAD {
                return Err("invalid local fixture command size".into());
            }
            if batched {
                // Flush at every peer change. This preserves the global
                // planned order, not just each board's local wire order.
                if used > 0 && (owner != board || used + bytes.len() > MAX_PAYLOAD) {
                    submit(owner, &frame[..used])?;
                    used = 0;
                }
                owner = board;
                frame[used..used + bytes.len()].copy_from_slice(bytes);
                used += bytes.len();
            } else {
                submit(board, bytes)?;
            }
            sent[board] += 1;
        }
    }
    if used > 0 {
        submit(owner, &frame[..used])?;
    }
    let enqueue_ns = began.elapsed().as_nanos();
    let lead = (hz * 2.) as u64;
    let mut final_pos = [0; 5];
    for n in 0..=16 {
        let relative = lead + (end - lead) * n / 16;
        for peer in &mut attested {
            peer.advance(relative)?;
        }
        for (lane, (board, oid)) in ROUTE.into_iter().enumerate() {
            let got = attested[board].position(oid)?;
            let expected = motors[lane].advance(relative).map_err(err)?.position_steps;
            if i64::from(got) != expected {
                return Err(format!("checkpoint {n} lane {lane}: {got} != {expected}"));
            }
            final_pos[lane] = got;
        }
    }
    eprintln!(
        "GPIO_DUAL {} {} {} {} {} {} {}",
        sent[0], sent[1], final_pos[0], final_pos[1], final_pos[2], final_pos[3], final_pos[4]
    );
    eprintln!("DUAL_ENQUEUE {} {} {enqueue_ns}", frames[0], frames[1]);
    Ok(())
}
