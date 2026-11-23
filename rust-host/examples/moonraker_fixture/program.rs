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

const HZ: f64 = 20e6;
const SCRIPT: &str =
    "M109 S210\nG90\nM83\nG1 X4 E.1 F800\nM106 S127.5\nT5\nM109 S220\nG1 Y5 E.2 F800\n";

fn err(e: impl std::fmt::Debug) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, format!("{e:?}"))
}

pub struct FixtureProgram {
    program: Program,
    step: u8,
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
        Ok(Self { program, step: 0 })
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
                }
            }
            _ => return Err(err("unknown fixture control step")),
        }
        self.step = next;
        Ok(())
    }

    pub fn publish(&self, source: &mut Snapshot) {
        assert!(self.program.publish_status(source));
        let extra = json!({"fixture_step":self.step});
        assert!(source.set_object("virtual_fixture".into(), extra.as_object().unwrap().clone()));
    }
}
