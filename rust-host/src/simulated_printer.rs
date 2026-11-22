// SPDX-License-Identifier: GPL-3.0-only
//! One bounded offline execution path for SD input, virtual homing, motion, and status.
//! This is not a Klippy daemon and cannot open or drive a printer.

use crate::{
    gcode_macros::Registry,
    homing::VirtualHoming,
    machine_preview::{self, Machine},
    print_stats::{self, Extrusion, Stats},
    step_clock::Clock,
    step_wire::StepMessages,
    virtual_job::{self, Issued, Job, State},
    virtual_sd,
};

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
}

pub struct Simulation {
    job: Job,
    stats: Stats,
    validated_bytes: usize,
    source_bytes: usize,
    hz: f64,
    last_clock: u64,
}

/// Inputs bound to the explicitly virtual run; no external paths are opened.
pub struct Input<'a> {
    pub name: &'a str,
    pub script: &'a str,
    pub macros: Option<&'a Registry>,
    pub discovered: &'a StepMessages,
    pub oids: [u8; 3],
    pub hz: f64,
}

impl Simulation {
    pub fn prepare(
        machine: &Machine,
        home: &VirtualHoming,
        input: Input<'_>,
    ) -> Result<Self, Error> {
        if input.script.len() > MAX_SCRIPT_BYTES {
            return Err(Error::InputLimit);
        }
        if !(1.0..=1e9).contains(&input.hz) {
            return Err(Error::Clock);
        }
        let mut sd = virtual_sd::Job::select(input.script.as_bytes());
        sd.resume().map_err(Error::Source)?;
        while let Some(line) = sd.issue().map_err(Error::Source)? {
            sd.commit(line.token, None).map_err(Error::Source)?;
        }
        let position = sd.snapshot().position;
        let source = &input.script[..position];
        let expanded;
        let script = if let Some(macros) = input.macros {
            expanded = macros.expand(source).map_err(Error::Macro)?;
            expanded.as_str()
        } else {
            source
        };
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: input.hz,
        };
        let trace = machine
            .trace_homed(script, home, START_TIME, clock, 20, [false; 3])
            .map_err(Error::Machine)?;
        let ticks = ((START_TIME + trace.duration) * clock.hz).ceil();
        if !(0.0..u64::MAX as f64).contains(&ticks) {
            return Err(Error::Clock);
        }
        let end = (ticks as u64).checked_add(1).ok_or(Error::Clock)?;
        let job =
            Job::for_trace(input.discovered, &trace, &input.oids, 0, end, 1).map_err(Error::Job)?;
        let mut stats = Stats::default();
        stats.select(input.name).map_err(Error::Status)?;
        Ok(Self {
            job,
            stats,
            validated_bytes: position,
            source_bytes: input.script.len(),
            hz: input.hz,
            last_clock: 0,
        })
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
            .note_start(at, NO_EXTRUDER)
            .map_err(Error::Status)?;
        self.last_clock = clock;
        Ok(())
    }

    pub fn advance(&mut self, clock: u64, budget: usize) -> Result<Vec<Issued>, Error> {
        let at = self.seconds(clock)?;
        let was = self.job.snapshot().state;
        let issued = self.job.poll(clock, budget).map_err(Error::Job)?;
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
            .note_pause(at, NO_EXTRUDER)
            .map_err(Error::Status)
    }

    pub fn resume(&mut self, clock: u64) -> Result<(), Error> {
        let at = self.seconds(clock)?;
        self.job.resume(clock).map_err(Error::Job)?;
        self.stats
            .note_start(at, NO_EXTRUDER)
            .map_err(Error::Status)?;
        self.last_clock = clock;
        Ok(())
    }

    pub fn cancel(&mut self) -> Result<(), Error> {
        let at = self.seconds(self.last_clock)?;
        self.job.cancel().map_err(Error::Job)?;
        self.stats
            .note_finish(at, print_stats::State::Cancelled, "")
            .map_err(Error::Status)
    }

    pub fn fail(&mut self, message: &str) -> Result<(), Error> {
        let at = self.seconds(self.last_clock)?;
        if message.len() > 4096 {
            return Err(Error::Status(print_stats::Error::Metadata));
        }
        self.job.fault().map_err(Error::Job)?;
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
            .observe(time, NO_EXTRUDER)
            .map_err(Error::Status)?;
        Ok(stats.publish(source)
            && self.job.publish_status(source)
            && source.set_object("virtual_input".into(), sd.as_object().unwrap().clone()))
    }

    fn seconds(&self, clock: u64) -> Result<f64, Error> {
        let at = clock as f64 / self.hz;
        if clock < self.last_clock || !(0.0..=1e12).contains(&at) {
            return Err(Error::Clock);
        }
        Ok(at)
    }
}
