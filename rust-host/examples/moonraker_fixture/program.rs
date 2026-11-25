// SPDX-License-Identifier: GPL-3.0-only
//! Private deterministic driver for a read-only fixture, never device I/O.
use klipper_host_protocol::{
    config,
    extrusion::{Extruder, Settings},
    identify::Dictionary,
    machine_preview::{
        mixed::{ToolInput, ToolSlot},
        Machine, SimulatedExtruder, ToolLane,
    },
    simulated_printer::mixed::{Program, Route, State, ToolRoutes},
    step_wire::{StepMessages, QUEUE, SET_DIR},
    thermal::commands::{Bank, Gate, Observation, Profile, Role, Source},
    webhooks::Snapshot,
};
use serde_json::json;
use std::{collections::BTreeMap, io};

#[path = "sd_status.rs"]
mod sd_status;

const HZ: f64 = 20e6;
const SCRIPT: &str =
    "M107\nM109 S210\nG90\nM83\nG1 X4 E.1 F800\nM106 S102\nT5\nM109 S220\nG1 Y5 E.2 F800\n";

fn err(e: impl std::fmt::Debug) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, format!("{e:?}"))
}

pub struct FixtureProgram {
    program: Program,
    step: u8,
    prior_generation: Option<u64>,
    sd_status: sd_status::SdStatus,
}

impl FixtureProgram {
    pub fn new() -> io::Result<Self> {
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
        let dictionary: Dictionary = serde_json::from_value(json!({
            "commands": {QUEUE:13,SET_DIR:14},"responses":{},"version":"software-fixture"
        }))?;
        let codec = StepMessages::from_dictionary(&dictionary).map_err(err)?;
        let trace = machine
            .trace_mixed_tools_at_virtual_origin(
                [1., 1., 0.2],
                &bank,
                ToolInput {
                    script: SCRIPT,
                    slots: &[slot(2, 0), slot(5, 1)],
                    initial_tool: 2,
                    print_fan: true,
                    hz: HZ,
                    max_error: 20,
                    invert: [false; 3],
                },
            )
            .map_err(err)?;
        let route = |peer, oid| Route { peer, oid };
        let program = Program::new_tools_routed(
            trace,
            &codec,
            ToolRoutes {
                xyz: [route(0, 0), route(0, 1), route(0, 2)],
                extruders: &[(2, route(1, 0)), (5, route(2, 0))],
            },
            bank.targets(),
            0.5,
        )
        .map_err(err)?;
        Ok(Self {
            program,
            step: 0,
            prior_generation: None,
            sd_status: sd_status::SdStatus::new()?,
        })
    }

    pub fn advance(&mut self, next: u8) -> io::Result<()> {
        if next == self.step {
            return Ok(());
        }
        if next != self.step + 1 {
            return Err(err("unexpected fixture control step"));
        }
        let at = |seconds: f64| (seconds * HZ) as u64;
        match next {
            1 => self.program.start(0).map_err(err)?,
            2 => {
                let ready = self
                    .program
                    .temperature(
                        at(1.),
                        Some(Observation {
                            source: Source::Heater(0),
                            sampled_at: 1.,
                            temperature: 210.,
                            controller_busy: false,
                        }),
                        false,
                    )
                    .map_err(err)?;
                if ready != Gate::Ready {
                    return Err(err("fixture temperature did not clear"));
                }
            }
            3 => {
                let issued = self.program.advance_routed(at(9.), 256).map_err(err)?;
                if issued.is_empty() || self.program.state() != State::AwaitingMotionCommit {
                    return Err(err("fixture did not stop for independent peer evidence"));
                }
            }
            4 | 5 => {
                let (generation, actual) = self
                    .program
                    .pending_peer_commit()
                    .ok_or_else(|| err("missing peer receipt metadata"))?;
                if actual.len() != 2 {
                    return Err(err("fixture expected two peer counts"));
                }
                if next == 4 {
                    if self
                        .program
                        .commit_peers(at(10.), generation, &actual[..1])
                        .is_ok()
                        || !self
                            .program
                            .advance_routed(at(11.), 256)
                            .map_err(err)?
                            .is_empty()
                    {
                        return Err(err("partial fixture receipt incorrectly cleared"));
                    }
                } else {
                    // Synthetic proof for this read-only display exercise,
                    // unlike the separately verified two-firmware test.
                    self.program
                        .commit_peers(at(12.), generation, &actual)
                        .map_err(err)?;
                    self.prior_generation = Some(generation);
                }
            }
            6 => {
                let ready = self
                    .program
                    .temperature(
                        at(13.),
                        Some(Observation {
                            source: Source::Heater(1),
                            sampled_at: 13.,
                            temperature: 220.,
                            controller_busy: false,
                        }),
                        false,
                    )
                    .map_err(err)?;
                if ready != Gate::Ready {
                    return Err(err("second fixture temperature did not clear"));
                }
            }
            7 => {
                let issued = self.program.advance_routed(at(20.), 256).map_err(err)?;
                if issued.is_empty() || self.program.state() != State::AwaitingMotionCommit {
                    return Err(err(
                        "second phase did not wait for independent peer evidence",
                    ));
                }
            }
            8 | 9 => {
                let (generation, actual) = self
                    .program
                    .pending_peer_commit()
                    .ok_or_else(|| err("missing terminal proof"))?;
                let prior = self
                    .prior_generation
                    .ok_or_else(|| err("missing prior phase generation"))?;
                if generation <= prior
                    || actual.iter().map(|r| r.peer).collect::<Vec<_>>() != [0, 2]
                {
                    return Err(err("unexpected terminal phase evidence"));
                }
                if next == 8 {
                    // Deliberately keep the correct peer identities, counts and
                    // ticks: only the earlier generation should block this receipt.
                    if self.program.commit_peers(at(21.), prior, &actual).is_ok()
                        || self.program.pending_peer_commit() != Some((generation, actual))
                        || !self
                            .program
                            .advance_routed(at(22.), 256)
                            .map_err(err)?
                            .is_empty()
                    {
                        return Err(err(
                            "earlier phase generation incorrectly cleared terminal gate",
                        ));
                    }
                } else {
                    self.program
                        .commit_peers(at(23.), generation, &actual)
                        .map_err(err)?;
                    if self.program.state() != State::Finished
                        || self.program.pending_peer_commit().is_some()
                        || !self
                            .program
                            .advance_routed(at(24.), 256)
                            .map_err(err)?
                            .is_empty()
                    {
                        return Err(err("terminal fixture state leaked commands or proof"));
                    }
                }
            }
            _ => return Err(err("unknown fixture control step")),
        }
        self.sd_status.advance(next)?;
        self.step = next;
        Ok(())
    }

    pub fn publish(&mut self, source: &mut Snapshot) {
        assert!(self.program.publish_status(source));
        self.sd_status.publish(source, self.step);
        // Stock-shaped read-only fields for frontend compatibility. Speed
        // is simulated demand; there is no tachometer, PWM or command API.
        for (name, data) in [
            (
                "configfile",
                json!({"config":{"fan":{"pin":"virtual"}}, "warnings":[]}),
            ),
            (
                "fan",
                json!({"speed":self.program.fan_demand().unwrap_or(0.0), "rpm":null}),
            ),
        ] {
            assert!(source.set_object(name.into(), data.as_object().unwrap().clone()));
        }
        let extra = json!({"fixture_step":self.step});
        assert!(source.set_object("virtual_fixture".into(), extra.as_object().unwrap().clone()));
    }
}
