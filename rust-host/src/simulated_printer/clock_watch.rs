// SPDX-License-Identifier: GPL-3.0-only
//! An optional liveness gate for inert simulation only.

use super::{Error, Simulation};
use crate::{
    clock_liveness::{ClockLiveness, InvalidTime, TimeoutAction},
    virtual_job::{Issued, Snapshot, State},
};

pub enum Advance {
    Output(Vec<Issued>),
    /// The observed timeout latched before any further fixture output.
    LostCommunication,
}

pub struct ClockWatchedSimulation {
    simulation: Simulation,
    watch: ClockLiveness,
    file_output: bool,
}

impl ClockWatchedSimulation {
    /// Wrap a prepared or running simulator, never a device or transport.
    pub fn new(simulation: Simulation, file_output: bool) -> Self {
        Self {
            simulation,
            watch: ClockLiveness::default(),
            file_output,
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        self.simulation.snapshot()
    }

    pub fn query_sent(&mut self, at: f64) -> Result<f64, InvalidTime> {
        self.watch.query_sent(at)
    }

    pub fn reply_received(&mut self) {
        self.watch.reply_received();
    }

    pub fn advance(&mut self, clock: u64, budget: usize) -> Result<Advance, Error> {
        if self.watch.check_timeout(self.file_output) == TimeoutAction::LostCommunication {
            if matches!(
                self.snapshot().state,
                State::Ready | State::Running | State::Paused
            ) {
                self.simulation
                    .fail("Lost communication with simulated MCU")?;
            }
            return Ok(Advance::LostCommunication);
        }
        self.simulation.advance(clock, budget).map(Advance::Output)
    }
}

#[cfg(feature = "clock-watched-status-preview")]
mod status;
