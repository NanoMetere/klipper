// SPDX-License-Identifier: GPL-3.0-only
//! One bounded offline execution path for SD input, virtual homing, motion, and status.
//! This is not a Klippy daemon and cannot open or drive a printer.

use crate::{
    gcode_macros::Registry,
    homing::VirtualHoming,
    machine_preview::{self, Machine, SimulatedExtruder, ToolLane},
    print_stats::{self, Extrusion, Stats},
    step_clock::{Clock, Command},
    step_wire::StepMessages,
    thermal::commands::{self as thermal, Gate, Observation, Source, Wait},
    verify_heater::{Decision as HeaterDecision, Fault as HeaterFault, Monitor},
    virtual_job::{self, Issued, Job, State},
    virtual_sd,
};
use std::{borrow::Cow, collections::VecDeque};

pub mod mixed;

const NO_EXTRUDER: Extrusion = Extrusion {
    position: 0.0,
    factor: 1.0,
};
const START_TIME: f64 = 2.0;
const MAX_SCRIPT_BYTES: usize = 8 * 1024 * 1024;

#[derive(Debug)]
pub enum Error {
    InputLimit,
    Clock,
    Source(virtual_sd::Error),
    Macro(crate::gcode_macros::Error),
    Machine(machine_preview::Error),
    Job(virtual_job::Error),
    Status(print_stats::Error),
    Thermal(thermal::Error),
    ThermalGate,
}

pub struct Simulation {
    job: Job,
    stats: Stats,
    validated_bytes: usize,
    source_bytes: usize,
    hz: f64,
    last_clock: u64,
    reading: Extrusion,
    extruder_lane: bool,
    tools: Option<ToolState>,
    heat_wait: Option<HeatWait>,
}

/// Inputs bound to the explicitly virtual run; no external paths are opened.
pub struct Input<'a> {
    pub name: &'a str,
    pub script: &'a str,
    pub macros: Option<&'a Registry>,
    pub discovered: &'a StepMessages,
    pub oids: &'a [u8],
    pub extruder: Option<SimulatedExtruder<'a>>,
    pub hz: f64,
}

/// Static lanes and a motion-only script for the fully inert multi-tool job.
/// The tool choices are planned selectors, not physical dock or probe events.
pub struct MultiInput<'a> {
    pub name: &'a str,
    pub script: &'a str,
    pub macros: Option<&'a Registry>,
    pub discovered: &'a StepMessages,
    pub oids: &'a [u8],
    pub lanes: &'a [ToolLane<'a>],
    pub initial_tool: u8,
    pub hz: f64,
}

struct ToolState {
    readings: Vec<(u8, Extrusion)>,
    changes: VecDeque<(u64, u8)>,
    planned_active: u8,
}

enum HeatSource {
    Heater(usize),
    Sensor(String),
}

enum HeatCondition {
    Controller {
        index: usize,
        target: f64,
    },
    Range {
        source: HeatSource,
        minimum: f64,
        maximum: f64,
    },
}

struct HeatWait {
    condition: HeatCondition,
    max_sample_age: f64,
}

impl HeatWait {
    fn new(wait: Wait<'_>, max_sample_age: f64) -> Result<Self, Error> {
        if !(0.0..=3600.).contains(&max_sample_age) {
            return Err(Error::Thermal(thermal::Error::Observation));
        }
        let condition = match wait {
            Wait::Controller { index, target }
                if index < 32 && target.is_finite() && target != 0. =>
            {
                HeatCondition::Controller { index, target }
            }
            Wait::Range {
                source,
                minimum,
                maximum,
            } if minimum < maximum && (minimum.is_finite() || maximum.is_finite()) => {
                let source = match source {
                    Source::Heater(index) if index < 32 => HeatSource::Heater(index),
                    Source::Sensor(name) if (1..=128).contains(&name.len()) => {
                        HeatSource::Sensor(name.into())
                    }
                    _ => return Err(Error::ThermalGate),
                };
                HeatCondition::Range {
                    source,
                    minimum,
                    maximum,
                }
            }
            _ => return Err(Error::ThermalGate),
        };
        Ok(Self {
            condition,
            max_sample_age,
        })
    }
    fn wait(&self) -> Wait<'_> {
        match &self.condition {
            HeatCondition::Controller { index, target } => Wait::Controller {
                index: *index,
                target: *target,
            },
            HeatCondition::Range {
                source,
                minimum,
                maximum,
            } => {
                let source = match source {
                    HeatSource::Heater(index) => Source::Heater(*index),
                    HeatSource::Sensor(name) => Source::Sensor(name),
                };
                Wait::Range {
                    source,
                    minimum: *minimum,
                    maximum: *maximum,
                }
            }
        }
    }
}

fn preflight<'a>(
    script: &'a str,
    macros: Option<&Registry>,
    hz: f64,
) -> Result<(Cow<'a, str>, usize), Error> {
    if script.len() > MAX_SCRIPT_BYTES {
        return Err(Error::InputLimit);
    }
    if !(1.0..=1e9).contains(&hz) {
        return Err(Error::Clock);
    }
    let mut sd = virtual_sd::Job::select(script.as_bytes());
    sd.resume().map_err(Error::Source)?;
    while let Some(line) = sd.issue().map_err(Error::Source)? {
        sd.commit(line.token, None).map_err(Error::Source)?;
    }
    let position = sd.snapshot().position;
    let source = &script[..position];
    let script = if let Some(registry) = macros {
        Cow::Owned(registry.expand(source).map_err(Error::Macro)?)
    } else {
        Cow::Borrowed(source)
    };
    Ok((script, position))
}

fn tick(at: f64, hz: f64) -> Result<u64, Error> {
    let ticks = (at * hz).ceil();
    if !(0.0..u64::MAX as f64).contains(&ticks) {
        return Err(Error::Clock);
    }
    Ok(ticks as u64)
}

fn job_end(time: f64, hz: f64, traces: &[&[Command]]) -> Result<u64, Error> {
    let ticks = tick(START_TIME + time, hz)?;
    let last = traces
        .iter()
        .flat_map(|lane| lane.iter())
        .filter_map(|cmd| match cmd {
            Command::Steps(block) => Some(block.last_clock),
            _ => None,
        })
        .max()
        .unwrap_or_default();
    ticks.max(last).checked_add(1).ok_or(Error::Clock)
}

impl Simulation {
    pub fn prepare(
        machine: &Machine,
        home: &VirtualHoming,
        input: Input<'_>,
    ) -> Result<Self, Error> {
        let (script, position) = preflight(input.script, input.macros, input.hz)?;
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: input.hz,
        };
        let trace = machine
            .trace_homed_with_extruder(
                &script,
                home,
                START_TIME,
                clock,
                20,
                [false; 3],
                input.extruder,
            )
            .map_err(Error::Machine)?;
        let lanes = trace
            .motors
            .iter()
            .chain(trace.extruder.iter())
            .map(Vec::as_slice)
            .collect::<Vec<_>>();
        let end = job_end(trace.duration, clock.hz, &lanes)?;
        let job =
            Job::for_lanes(input.discovered, &lanes, input.oids, 0, end, 1).map_err(Error::Job)?;
        let mut stats = Stats::default();
        stats.select(input.name).map_err(Error::Status)?;
        Ok(Self {
            job,
            stats,
            validated_bytes: position,
            source_bytes: input.script.len(),
            hz: input.hz,
            last_clock: 0,
            reading: NO_EXTRUDER,
            extruder_lane: trace.extruder.is_some(),
            tools: None,
            heat_wait: None,
        })
    }

    pub fn prepare_tools(
        machine: &Machine,
        home: &VirtualHoming,
        input: MultiInput<'_>,
    ) -> Result<Self, Error> {
        let (script, position) = preflight(input.script, input.macros, input.hz)?;
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: input.hz,
        };
        let trace = machine
            .trace_tool_gcode(
                home,
                &script,
                input.initial_tool,
                input.lanes,
                START_TIME,
                clock,
                20,
                [false; 3],
            )
            .map_err(Error::Machine)?;
        let mut stats = Stats::default();
        stats.select(input.name).map_err(Error::Status)?;
        let lanes = trace
            .motors
            .iter()
            .chain(trace.tools.iter().map(|(_, rows)| rows))
            .map(Vec::as_slice)
            .collect::<Vec<_>>();
        let end = job_end(trace.duration, clock.hz, &lanes)?;
        let job =
            Job::for_lanes(input.discovered, &lanes, input.oids, 0, end, 1).map_err(Error::Job)?;
        let changes = trace
            .switches
            .iter()
            .map(|change| Ok((tick(change.at, clock.hz)?, change.id)))
            .collect::<Result<VecDeque<_>, Error>>()?;
        let readings = input
            .lanes
            .iter()
            .map(|lane| {
                (
                    lane.id,
                    Extrusion {
                        position: lane.parked_e,
                        factor: 1.0,
                    },
                )
            })
            .collect::<Vec<_>>();
        let reading = readings
            .iter()
            .find(|(id, _)| *id == input.initial_tool)
            .map(|(_, reading)| *reading)
            .ok_or(Error::Machine(machine_preview::Error::ToolSegment))?;
        Ok(Self {
            job,
            stats,
            validated_bytes: position,
            source_bytes: input.script.len(),
            hz: clock.hz,
            last_clock: 0,
            reading,
            // The single-extruder method is intentionally unavailable here.
            extruder_lane: false,
            tools: Some(ToolState {
                readings,
                changes,
                planned_active: input.initial_tool,
            }),
            heat_wait: None,
        })
    }

    pub fn planned_tool(&self) -> Option<u8> {
        self.tools.as_ref().map(|s| s.planned_active)
    }

    pub fn snapshot(&self) -> virtual_job::Snapshot {
        self.job.snapshot()
    }

    pub fn preflight_boundary(&self) -> (usize, usize) {
        (self.validated_bytes, self.source_bytes)
    }

    pub fn start(&mut self, clock: u64) -> Result<(), Error> {
        let at = self.seconds(clock)?;
        self.job.start(clock).map_err(Error::Job)?;
        self.stats
            .note_start(at, self.reading)
            .map_err(Error::Status)?;
        self.last_clock = clock;
        Ok(())
    }

    pub fn advance(&mut self, clock: u64, budget: usize) -> Result<Vec<Issued>, Error> {
        let at = self.seconds(clock)?;
        let was = self.job.snapshot().state;
        let issued = self.job.poll(clock, budget).map_err(Error::Job)?;
        if let Some(tools) = &mut self.tools {
            let simulated_tick = self.job.snapshot().simulated_tick;
            while tools
                .changes
                .front()
                .is_some_and(|(when, _)| *when <= simulated_tick)
            {
                let (_, id) = tools.changes.pop_front().expect("scheduled transition");
                let reading = tools
                    .readings
                    .iter()
                    .find(|(key, _)| *key == id)
                    .map(|(_, observed)| *observed)
                    .expect("tool references preflighted");
                self.stats
                    .switch_extruder(at, reading)
                    .map_err(Error::Status)?;
                tools.planned_active = id;
                self.reading = reading;
            }
        }
        if was == State::Running && self.job.snapshot().state == State::Finished {
            self.stats
                .note_finish(at, print_stats::State::Complete, "")
                .map_err(Error::Status)?;
        }
        self.last_clock = clock;
        Ok(issued)
    }

    pub fn pause(&mut self) -> Result<(), Error> {
        let at = self.seconds(self.last_clock)?;
        self.job.pause().map_err(Error::Job)?;
        self.stats
            .note_pause(at, self.reading)
            .map_err(Error::Status)
    }

    pub fn resume(&mut self, clock: u64) -> Result<(), Error> {
        if self.heat_wait.is_some() {
            return Err(Error::ThermalGate);
        }
        let at = self.seconds(clock)?;
        self.job.resume(clock).map_err(Error::Job)?;
        self.stats
            .note_start(at, self.reading)
            .map_err(Error::Status)?;
        self.last_clock = clock;
        Ok(())
    }

    /// Apply recognized heater text at an explicit virtual motion boundary.
    /// Non-heater G-code is left to the caller. If a wait cannot be armed,
    /// the heater target table is not updated. No hardware effects occur.
    pub fn dispatch_temperature<'a>(
        &mut self,
        bank: &mut thermal::Bank<'a>,
        line: &str,
        max_sample_age: f64,
    ) -> Result<Option<thermal::Output<'a>>, Error> {
        let Some(cmd) = thermal::parse_gcode(line).map_err(Error::Thermal)? else {
            return Ok(None);
        };
        if self.heat_wait.is_some() {
            return Err(Error::ThermalGate);
        }
        let mut staged = bank.clone();
        let result = staged.command(cmd).map_err(Error::Thermal)?;
        if let thermal::Output::Set {
            wait: Some(wait), ..
        }
        | thermal::Output::Range(wait) = result
        {
            self.begin_temperature_wait(wait, max_sample_age)?;
        }
        *bank = staged;
        Ok(Some(result))
    }

    /// Pause the virtual release cursor at an already planned command
    /// boundary. A generic resume cannot bypass this simulated wait.
    pub fn begin_temperature_wait(
        &mut self,
        wait: Wait<'_>,
        max_sample_age: f64,
    ) -> Result<(), Error> {
        if self.heat_wait.is_some() {
            return Err(Error::ThermalGate);
        }
        let pending = HeatWait::new(wait, max_sample_age)?;
        self.pause()?;
        self.heat_wait = Some(pending);
        Ok(())
    }

    /// Evaluate only a supplied inert observation, at a caller-selected host
    /// tick. Missing/stale data leave the virtual release cursor blocked.
    pub fn poll_temperature_wait(
        &mut self,
        clock: u64,
        reading: Option<Observation<'_>>,
        shutdown: bool,
    ) -> Result<Gate, Error> {
        let at = self.seconds(clock)?;
        let state = self.heat_wait.as_ref().ok_or(Error::ThermalGate)?;
        let gate = state
            .wait()
            .poll(at, state.max_sample_age, reading, shutdown)
            .map_err(Error::Thermal)?;
        // A paused job always emits no records. Moving this host clock does
        // not move the virtual job cursor.
        debug_assert_eq!(self.job.snapshot().state, State::Paused);
        let work = self.advance(clock, 1)?;
        debug_assert!(work.is_empty());
        match gate {
            Gate::Pending => (),
            Gate::Ready => {
                self.heat_wait = None;
                self.resume(clock)?;
            }
            Gate::Shutdown => self.fail("simulated heater wait shut down")?,
        }
        Ok(gate)
    }

    /// Attach an inert extrusion observation to print status. This is
    /// separate from planning and never controls heat or motion.
    pub fn extrusion_observation(&mut self, reading: Extrusion) -> Result<(), Error> {
        if !self.extruder_lane
            || !reading.position.is_finite()
            || !reading.factor.is_finite()
            || reading.factor <= 0.0
        {
            return Err(Error::Status(print_stats::Error::Extrusion));
        }
        self.stats
            .observe(self.seconds(self.last_clock)?, reading)
            .map_err(Error::Status)?;
        self.reading = reading;
        Ok(())
    }

    /// Record an observation only for the modeled active lane. Other tools'
    /// parked E and flow cannot be charged to this lane's filament total.
    pub fn tool_extrusion_observation(
        &mut self,
        tool: u8,
        reading: Extrusion,
    ) -> Result<(), Error> {
        let at = self.seconds(self.last_clock)?;
        let fault = || Error::Status(print_stats::Error::Extrusion);
        let tools = self
            .tools
            .as_mut()
            .filter(|s| s.planned_active == tool)
            .ok_or_else(fault)?;
        let position = tools
            .readings
            .iter_mut()
            .find(|(id, _)| *id == tool)
            .ok_or_else(fault)?;
        if !reading.position.is_finite() || !reading.factor.is_finite() || reading.factor <= 0.0 {
            return Err(fault());
        }
        self.stats.observe(at, reading).map_err(Error::Status)?;
        position.1 = reading;
        self.reading = reading;
        Ok(())
    }

    pub fn cancel(&mut self) -> Result<(), Error> {
        let at = self.seconds(self.last_clock)?;
        self.job.cancel().map_err(Error::Job)?;
        self.heat_wait = None;
        if let Some(tools) = &mut self.tools {
            tools.changes.clear();
        }
        self.stats
            .note_finish(at, print_stats::State::Cancelled, "")
            .map_err(Error::Status)
    }

    /// Inspect a simulated heater observation at the current host clock.
    /// A failed monitor latches the active virtual job before it can emit
    /// more commands. This does not run any physical heater shutdown path.
    pub fn heater_sample(
        &mut self,
        monitor: &mut Monitor,
        temperature: f64,
        target: f64,
    ) -> Result<HeaterDecision, Error> {
        let at = self.seconds(self.last_clock)?;
        let result = monitor.sample(at, temperature, target);
        if let HeaterDecision::Fault(reason) = result {
            if matches!(
                self.job.snapshot().state,
                State::Ready | State::Running | State::Paused
            ) {
                self.fail(match reason {
                    HeaterFault::NotHeating => "simulated heater not heating at expected rate",
                    HeaterFault::InvalidObservation => "invalid simulated heater observation",
                })?;
            }
        }
        Ok(result)
    }

    pub fn fail(&mut self, message: &str) -> Result<(), Error> {
        let at = self.seconds(self.last_clock)?;
        if message.len() > 4096 {
            return Err(Error::Status(print_stats::Error::Metadata));
        }
        self.job.fault().map_err(Error::Job)?;
        self.heat_wait = None;
        if let Some(tools) = &mut self.tools {
            tools.changes.clear();
        }
        self.stats
            .note_finish(at, print_stats::State::Error, message)
            .map_err(Error::Status)
    }

    /// Populate conventional status objects only from observations already
    /// supplied to the simulator. No WebHook command can alter this state.
    #[cfg(feature = "webhooks")]
    pub fn publish(&mut self, source: &mut crate::webhooks::Snapshot) -> Result<bool, Error> {
        let time = self.seconds(self.last_clock)?;
        let sd = serde_json::json!({
            "mode": "simulation",
            "preflight_boundary": self.validated_bytes,
            "file_size": self.source_bytes,
        });
        let stats = self
            .stats
            .observe(time, self.reading)
            .map_err(Error::Status)?;
        let base = stats.publish(source)
            && self.job.publish_status(source)
            && source.set_object("virtual_input".into(), sd.as_object().unwrap().clone());
        let tools = self.tools.as_ref().is_none_or(|state| {
            let object = serde_json::json!({
                "mode": "planned_virtual",
                "planned_active_tool": state.planned_active,
                "pending_tool_changes": state.changes.len(),
                "tool_ids": state.readings.iter().map(|(id, _)| id).collect::<Vec<_>>(),
            });
            source.set_object(
                "virtual_tool_lanes".into(),
                object.as_object().unwrap().clone(),
            )
        });
        let heat = serde_json::json!({ "mode":"simulation", "waiting":self.heat_wait.is_some() });
        Ok(base
            && tools
            && source.set_object(
                "virtual_heat_wait".into(),
                heat.as_object().unwrap().clone(),
            ))
    }

    fn seconds(&self, clock: u64) -> Result<f64, Error> {
        let at = clock as f64 / self.hz;
        if clock < self.last_clock || !(0.0..=1e12).contains(&at) {
            return Err(Error::Clock);
        }
        Ok(at)
    }
}
