// SPDX-License-Identifier: GPL-3.0-only
//! Read-only status of the simulated clock guard. No transport or commands.

use super::{ClockWatchedSimulation, Error};
use crate::webhooks::Snapshot;
use serde_json::json;

impl ClockWatchedSimulation {
    /// Publish both current clock response health and the separately latched
    /// timeout. A late reply can update the former without undoing the latter
    /// or changing the already faulted virtual job.
    pub fn publish_watched(&mut self, source: &mut Snapshot) -> Result<bool, Error> {
        if !self.simulation.publish(source)? {
            return Ok(false);
        }
        let value = json!({
            "mode": "simulation",
            "active": self.watch.is_active(),
            "timeout_reported": self.watch.timeout_reported(),
            "file_output": self.file_output,
        });
        Ok(source.set_object(
            "virtual_clock_liveness".into(),
            value.as_object().unwrap().clone(),
        ))
    }
}
