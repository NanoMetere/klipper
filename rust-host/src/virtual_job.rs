// SPDX-License-Identifier: GPL-3.0-only
//! Virtual job lifecycle for simulated inert motion payloads. No device I/O.

use crate::{
    machine_preview::Trace,
    step_clock::Command,
    step_wire::{self, Payload, StepMessages},
};
use std::collections::{BTreeSet, VecDeque};

const MAX_COMMANDS: usize = 196_000;
const MAX_BATCH: usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Ready,
    Running,
    Paused,
    Finished,
    Cancelled,
    Faulted,
}

#[derive(Debug)]
pub enum Error {
    InvalidState,
    InvalidTrace,
    InvalidClock,
    Batch,
    Wire(step_wire::Error),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    pub state: State,
    pub simulated_tick: u64,
    pub issued: usize,
    pub total: usize,
    pub planned_end: u64,
    pub generation: u64,
}

#[derive(Debug)]
pub struct Issued {
    pub lane: usize,
    pub generation: u64,
    pub command: Payload,
}

/// Holds a finite preflighted script. Issued outputs are inert records for
/// a fixture only. Pause freezes virtual time at the last clock observation.
/// Cancel/fault discards unissued commands; it is not a hardware estop.
pub struct Job {
    state: State,
    generation: u64,
    simulation_tick: u64,
    clock: u64,
    end: u64,
    issued: usize,
    total: usize,
    lanes: Vec<VecDeque<Payload>>,
}

impl Job {
    /// Translate a complete validated motion trace without touching external
    /// state. OIDs must be distinct. Caller supplies an end tick that covers
    /// the total modeled duration including any trailing dwell.
    pub fn for_trace(
        codec: &StepMessages,
        trace: &Trace,
        oids: &[u8],
        initial: u64,
        end: u64,
        generation: u64,
    ) -> Result<Self, Error> {
        let lanes = trace
            .motors
            .iter()
            .chain(trace.extruder.iter())
            .map(Vec::as_slice)
            .collect::<Vec<_>>();
        Self::for_lanes(codec, &lanes, oids, initial, end, generation)
    }

    /// Bind independent inert motor traces to a shared virtual schedule.
    pub fn for_lanes(
        codec: &StepMessages,
        traces: &[&[Command]],
        oids: &[u8],
        initial: u64,
        end: u64,
        generation: u64,
    ) -> Result<Self, Error> {
        let lane_count = traces.len();
        if !(1..=16).contains(&lane_count)
            || oids.len() != lane_count
            || oids.iter().copied().collect::<BTreeSet<_>>().len() != lane_count
            || end < initial
            || generation == 0
        {
            return Err(Error::InvalidTrace);
        }
        let mut total = 0usize;
        let mut lanes = Vec::with_capacity(lane_count);
        for (&lane, &oid) in traces.iter().zip(oids) {
            total += lane.len();
            if total > MAX_COMMANDS
                || lane.iter().any(|cmd| match cmd {
                    Command::Steps(b) => b.last_clock > end,
                    Command::SetDirection(_) => false,
                })
            {
                return Err(Error::InvalidTrace);
            }
            let payloads = codec.payloads(oid, initial, lane).map_err(Error::Wire)?;
            lanes.push(payloads.into());
        }
        Ok(Self {
            state: State::Ready,
            generation,
            simulation_tick: initial,
            clock: 0,
            end,
            issued: 0,
            total,
            lanes,
        })
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            state: self.state,
            simulated_tick: self.simulation_tick,
            issued: self.issued,
            total: self.total,
            planned_end: self.end,
            generation: self.generation,
        }
    }

    /// Publish explicitly virtual status without exposing command execution
    /// through the read-only Klippy protocol subset.
    #[cfg(feature = "webhooks")]
    pub fn publish_status(&self, source: &mut crate::webhooks::Snapshot) -> bool {
        let s = self.snapshot();
        let state = match s.state {
            State::Ready => "ready",
            State::Running => "running",
            State::Paused => "paused",
            State::Finished => "finished",
            State::Cancelled => "cancelled",
            State::Faulted => "faulted",
        };
        let body = serde_json::json!({
            "mode": "simulation", "state": state, "simulated_tick": s.simulated_tick,
            "issued": s.issued, "total": s.total, "planned_end": s.planned_end, "generation": s.generation,
        });
        source.set_object("virtual_job".into(), body.as_object().unwrap().clone())
    }

    pub fn start(&mut self, clock: u64) -> Result<(), Error> {
        if self.state != State::Ready {
            return Err(Error::InvalidState);
        }
        self.state = State::Running;
        self.clock = clock;
        Ok(())
    }

    /// Freezes at the most recently advanced simulated tick. Call poll first
    /// if the fixture needs to advance to the exact pause instant.
    pub fn pause(&mut self) -> Result<(), Error> {
        if self.state != State::Running {
            return Err(Error::InvalidState);
        }
        self.state = State::Paused;
        Ok(())
    }

    pub fn resume(&mut self, clock: u64) -> Result<(), Error> {
        if self.state != State::Paused {
            return Err(Error::InvalidState);
        }
        if clock < self.clock {
            return Err(Error::InvalidClock);
        }
        self.clock = clock;
        self.state = State::Running;
        Ok(())
    }

    /// Advance the simulated clock and emit a bounded amount of due work,
    /// interleaving by required tick while preserving per-lane order.
    pub fn poll(&mut self, clock: u64, budget: usize) -> Result<Vec<Issued>, Error> {
        if !(1..=MAX_BATCH).contains(&budget) {
            return Err(Error::Batch);
        }
        if clock < self.clock {
            return Err(Error::InvalidClock);
        }
        if self.state != State::Running {
            if self.state == State::Paused {
                self.clock = clock;
            }
            return Ok(Vec::new());
        }
        self.simulation_tick = self
            .simulation_tick
            .saturating_add(clock - self.clock)
            .min(self.end);
        self.clock = clock;
        let mut out = Vec::new();
        for _ in 0..budget {
            let Some(lane) = self
                .lanes
                .iter()
                .enumerate()
                .filter_map(|(i, q)| q.front().map(|p| (i, p)))
                .filter(|(_, p)| p.required_tick <= self.simulation_tick)
                .min_by_key(|&(i, p)| (p.required_tick, i))
                .map(|(lane, _)| lane)
            else {
                break;
            };
            let command = self.lanes[lane].pop_front().expect("nonempty lane");
            out.push(Issued {
                lane,
                generation: self.generation,
                command,
            });
            self.issued += 1;
        }
        if self.issued == self.total && self.simulation_tick == self.end {
            self.state = State::Finished;
        }
        Ok(out)
    }

    pub fn cancel(&mut self) -> Result<(), Error> {
        if !matches!(self.state, State::Ready | State::Running | State::Paused) {
            return Err(Error::InvalidState);
        }
        self.finish(State::Cancelled);
        Ok(())
    }

    pub fn fault(&mut self) -> Result<(), Error> {
        if !matches!(self.state, State::Ready | State::Running | State::Paused) {
            return Err(Error::InvalidState);
        }
        self.finish(State::Faulted);
        Ok(())
    }

    fn finish(&mut self, state: State) {
        self.lanes.clear();
        self.state = state;
    }
}
