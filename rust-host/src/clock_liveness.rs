// SPDX-License-Identifier: GPL-3.0-only
//! Side-effect-free clock-query liveness and timeout indication.
//! Based on Klipper's ClockSync._get_clock_event and MCUConnectHelper
//! check_timeout behavior. This module does not send or abort any commands.

const MAX_ACTIVE_PENDING: u8 = 4;
const CLOCK_QUERY_INTERVAL: f64 = 0.9839;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeoutAction {
    None,
    /// The caller can report lost communication and abort pending waits.
    /// This option is emitted once per session and does not perform I/O.
    LostCommunication,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidTime;

#[derive(Debug, Clone, Default)]
pub struct ClockLiveness {
    pending: u8,
    timeout_reported: bool,
}

impl ClockLiveness {
    /// Record a successful submission, not a proposed query or a frame retry.
    /// Returns the next stock timer deadline in host seconds.
    pub fn query_sent(&mut self, at: f64) -> Result<f64, InvalidTime> {
        let next = at + CLOCK_QUERY_INTERVAL;
        if !at.is_finite() || at < 0. || !next.is_finite() || next <= at {
            return Err(InvalidTime);
        }
        if self.pending <= MAX_ACTIVE_PENDING {
            self.pending += 1;
        }
        Ok(next)
    }

    /// A decoded clock reply clears pending liveness queries even when its
    /// timing sample is unsuitable for regression. It never clears a report.
    pub fn reply_received(&mut self) {
        self.pending = 0;
    }

    pub fn is_active(&self) -> bool {
        self.pending <= MAX_ACTIVE_PENDING
    }

    pub fn timeout_reported(&self) -> bool {
        self.timeout_reported
    }

    /// File-output previews do not report timeouts. Only constructing a
    /// fresh session clears the one-shot report.
    pub fn check_timeout(&mut self, file_output: bool) -> TimeoutAction {
        if self.is_active() || file_output || self.timeout_reported {
            return TimeoutAction::None;
        }
        self.timeout_reported = true;
        TimeoutAction::LostCommunication
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifth_query_demotes_liveness_and_late_reply_never_rearms_report() {
        let mut watch = ClockLiveness::default();
        for i in 0..4 {
            let now = f64::from(i);
            assert_eq!(watch.query_sent(now), Ok(now + 0.9839));
            assert!(watch.is_active());
        }
        watch.query_sent(4.).unwrap();
        assert!(!watch.is_active());
        assert_eq!(watch.check_timeout(true), TimeoutAction::None);
        assert_eq!(watch.check_timeout(false), TimeoutAction::LostCommunication);
        assert_eq!(watch.check_timeout(false), TimeoutAction::None);
        watch.reply_received();
        assert!(watch.is_active());
        for i in 5..10 {
            watch.query_sent(f64::from(i)).unwrap();
        }
        assert!(watch.timeout_reported());
        assert_eq!(watch.check_timeout(false), TimeoutAction::None);
    }

    #[test]
    fn bad_timing_and_large_query_counts_are_bounded() {
        let mut watch = ClockLiveness::default();
        for time in [-1., f64::NAN, f64::INFINITY, f64::MAX] {
            assert_eq!(watch.query_sent(time), Err(InvalidTime));
        }
        assert!(watch.is_active());
        for i in 0..1000 {
            watch.query_sent(f64::from(i)).unwrap();
        }
        assert_eq!(watch.pending, MAX_ACTIVE_PENDING + 1);
    }
}
