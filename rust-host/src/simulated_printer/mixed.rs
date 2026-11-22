// SPDX-License-Identifier: GPL-3.0-only
//! Phase-isolated motion release for the offline heater/motion preview.
//! Per-phase ticks and generations are simulation records, not real MCU
//! reinitialization, transport or scheduling instructions.

use super::{job_end, Error, HeatWait};
use crate::{
    machine_preview::mixed::{MixedTrace, Stage},
    step_wire::StepMessages,
    thermal::commands::{Gate, Observation, Output, Source, Wait},
    virtual_job::{self, Issued, Job, State as JobState},
};
use std::collections::VecDeque;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Ready,
    Running,
    Waiting,
    AwaitingMotionCommit,
    Finished,
    Cancelled,
    Faulted,
}

/// Inert completion evidence supplied by a fixture, not proven by the
/// existence of queued messages or the wall clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotionProof {
    pub generation: u64,
    pub accepted_commands: usize,
    pub simulated_tick: u64,
}

enum Pending {
    Motion {
        line: usize,
        job: Job,
    },
    Heater {
        line: usize,
        wait: Option<HeatWait>,
        targets: Vec<f64>,
    },
}

pub struct Program {
    stages: VecDeque<Pending>,
    running: Option<Job>,
    waiting: Option<HeatWait>,
    state: State,
    line: Option<usize>,
    targets: Vec<f64>,
    hz: f64,
    clock: u64,
    require_commit: bool,
    pending_commit: Option<MotionProof>,
}

impl Program {
    /// Pack each phase's wire data into a distinct, initially inactive job.
    /// No input file or external device is opened. The target snapshots
    /// update only when the program reaches their corresponding lines.
    pub fn new(
        trace: MixedTrace<'_>,
        codec: &StepMessages,
        oids: &[u8],
        initial_targets: &[f64],
        max_sample_age: f64,
    ) -> Result<Self, Error> {
        if !(1.0..=1e9).contains(&trace.hz) {
            return Err(Error::Clock);
        }
        if !valid_targets(initial_targets) || trace.phases.len() > 1025 {
            return Err(Error::InputLimit);
        }
        let mut stages = VecDeque::with_capacity(trace.phases.len());
        let mut total = 0;
        let mut prior = 0;
        for (i, stage) in trace.phases.into_iter().enumerate() {
            let line = match &stage {
                Stage::Motion { first_line, .. } => *first_line,
                Stage::Heater { line, .. } => *line,
            };
            if line <= prior {
                return Err(Error::InputLimit);
            }
            prior = line;
            match stage {
                Stage::Heater {
                    line,
                    outcome,
                    targets,
                } => {
                    if targets.len() != initial_targets.len() || !valid_targets(&targets) {
                        return Err(Error::ThermalGate);
                    }
                    let wait = match outcome {
                        Output::Set { wait, .. } => wait,
                        Output::Range(wait) => Some(wait),
                        _ => None,
                    };
                    if wait.is_some_and(|w| match w {
                        Wait::Controller { index, target } => targets.get(index) != Some(&target),
                        Wait::Range {
                            source: Source::Heater(index),
                            ..
                        } => index >= targets.len(),
                        Wait::Range { .. } => false,
                    }) {
                        return Err(Error::ThermalGate);
                    }
                    let wait = wait.map(|w| HeatWait::new(w, max_sample_age)).transpose()?;
                    stages.push_back(Pending::Heater {
                        line,
                        wait,
                        targets,
                    });
                }
                Stage::Motion {
                    first_line,
                    trace: t,
                } => {
                    if !t.duration.is_finite() || t.duration < 0. {
                        return Err(Error::Clock);
                    }
                    let tracks = t
                        .motors
                        .iter()
                        .chain(t.extruder.iter())
                        .map(Vec::as_slice)
                        .collect::<Vec<_>>();
                    let count = tracks.iter().map(|lane| lane.len()).sum::<usize>();
                    total += count;
                    if total > 196_000 {
                        return Err(Error::InputLimit);
                    }
                    if count == 0 && t.duration == 0. {
                        continue;
                    }
                    let end = job_end(t.duration, trace.hz, &tracks)?;
                    let job = Job::for_lanes(codec, &tracks, oids, 0, end, i as u64 + 1)
                        .map_err(Error::Job)?;
                    stages.push_back(Pending::Motion {
                        line: first_line,
                        job,
                    });
                }
            }
        }
        Ok(Self {
            stages,
            running: None,
            waiting: None,
            state: State::Ready,
            line: None,
            targets: initial_targets.to_vec(),
            hz: trace.hz,
            clock: 0,
            require_commit: false,
            pending_commit: None,
        })
    }

    /// Require the caller to supply inert phase completion evidence before
    /// the next heater command or motion generation can be armed.
    pub fn require_motion_commit(mut self) -> Result<Self, Error> {
        if self.state != State::Ready {
            return Err(Error::MotionCommit);
        }
        self.require_commit = true;
        Ok(self)
    }

    pub fn pending_motion_commit(&self) -> Option<MotionProof> {
        self.pending_commit
    }

    pub fn state(&self) -> State {
        self.state
    }
    pub fn line(&self) -> Option<usize> {
        self.line
    }
    pub fn targets(&self) -> &[f64] {
        &self.targets
    }

    /// Report modeled state only; this status object has no mutation endpoint.
    #[cfg(feature = "webhooks")]
    pub fn publish_status(&self, snapshot: &mut crate::webhooks::Snapshot) -> bool {
        let state = match self.state {
            State::Ready => "ready",
            State::Running => "running",
            State::Waiting => "waiting_for_temperature",
            State::AwaitingMotionCommit => "waiting_for_motion_proof",
            State::Finished => "finished",
            State::Cancelled => "cancelled",
            State::Faulted => "faulted",
        };
        let pending = self.pending_commit.map(|p| {
            serde_json::json!({
                "generation": p.generation,
                "accepted_commands": p.accepted_commands,
                "simulated_tick": p.simulated_tick,
            })
        });
        let data = serde_json::json!({
            "mode": "simulation",
            "state": state,
            "line": self.line,
            "heater_targets_c": self.targets,
            "motion_proof": pending,
            "pending_stages": self.stages.len(),
        });
        snapshot.set_object(
            "virtual_mixed_job".into(),
            data.as_object().unwrap().clone(),
        )
    }

    pub fn start(&mut self, clock: u64) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::Job(virtual_job::Error::InvalidState));
        }
        self.time(clock)?;
        self.clock = clock;
        self.load(clock)
    }

    /// A single dispatch tick can drain only the currently armed phase.
    /// Even an arbitrarily late tick cannot issue data from the next phase.
    pub fn advance(&mut self, clock: u64, budget: usize) -> Result<Vec<Issued>, Error> {
        if !(1..=256).contains(&budget) {
            return Err(Error::Job(virtual_job::Error::Batch));
        }
        self.time(clock)?;
        self.clock = clock;
        let Some(running) = &mut self.running else {
            return Ok(Vec::new());
        };
        let out = running.poll(clock, budget).map_err(Error::Job)?;
        let stage = running.snapshot();
        if stage.state == JobState::Finished {
            self.running = None;
            if self.require_commit {
                self.pending_commit = Some(MotionProof {
                    generation: stage.generation,
                    accepted_commands: stage.total,
                    simulated_tick: stage.planned_end,
                });
                self.state = State::AwaitingMotionCommit;
            } else {
                self.load(clock)?;
            }
        }
        Ok(out)
    }

    /// Check the exact motion generation and accepted count, plus a tick
    /// at or beyond the virtual phase end. Invalid evidence leaves the gate
    /// closed. This caller-supplied evidence is simulation-only.
    pub fn commit_motion(&mut self, clock: u64, proof: MotionProof) -> Result<(), Error> {
        self.time(clock)?;
        let expected = self.pending_commit.ok_or(Error::MotionCommit)?;
        if proof.generation != expected.generation
            || proof.accepted_commands != expected.accepted_commands
            || proof.simulated_tick < expected.simulated_tick
        {
            return Err(Error::MotionCommit);
        }
        self.clock = clock;
        self.pending_commit = None;
        self.load(clock)
    }

    pub fn temperature(
        &mut self,
        clock: u64,
        sample: Option<Observation<'_>>,
        shutdown: bool,
    ) -> Result<Gate, Error> {
        let at = self.time(clock)?;
        let condition = self.waiting.as_ref().ok_or(Error::ThermalGate)?;
        let gate = condition
            .wait()
            .poll(at, condition.max_sample_age, sample, shutdown)
            .map_err(Error::Thermal)?;
        self.clock = clock;
        match gate {
            Gate::Pending => (),
            Gate::Ready => {
                self.waiting = None;
                self.load(clock)?;
            }
            Gate::Shutdown => self.clear(State::Faulted),
        }
        Ok(gate)
    }

    pub fn cancel(&mut self) -> Result<(), Error> {
        if !matches!(
            self.state,
            State::Ready | State::Running | State::Waiting | State::AwaitingMotionCommit
        ) {
            return Err(Error::Job(virtual_job::Error::InvalidState));
        }
        self.clear(State::Cancelled);
        Ok(())
    }

    fn load(&mut self, clock: u64) -> Result<(), Error> {
        while let Some(stage) = self.stages.pop_front() {
            match stage {
                Pending::Motion { line, mut job } => {
                    job.start(clock).map_err(Error::Job)?;
                    self.line = Some(line);
                    self.running = Some(job);
                    self.state = State::Running;
                    return Ok(());
                }
                Pending::Heater {
                    line,
                    wait,
                    targets,
                } => {
                    self.targets = targets;
                    if let Some(condition) = wait {
                        self.line = Some(line);
                        self.waiting = Some(condition);
                        self.state = State::Waiting;
                        return Ok(());
                    }
                }
            }
        }
        self.state = State::Finished;
        self.line = None;
        Ok(())
    }

    fn time(&self, clock: u64) -> Result<f64, Error> {
        let at = clock as f64 / self.hz;
        if clock < self.clock || !(0.0..=1e12).contains(&at) {
            return Err(Error::Clock);
        }
        Ok(at)
    }

    fn clear(&mut self, state: State) {
        self.stages.clear();
        self.pending_commit = None;
        self.running = None;
        self.waiting = None;
        self.state = state;
        self.line = None;
    }
}

fn valid_targets(targets: &[f64]) -> bool {
    targets.len() <= 32 && targets.iter().all(|x| x.is_finite())
}
