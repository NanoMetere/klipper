// SPDX-License-Identifier: GPL-3.0-only
//! Offline adaptive transport timing from Klipper chelper/serialqueue.c.
//! Upstream copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>.
use crate::flight::Retry;

const MIN_RTO: f64 = 0.025;
const MAX_RTO: f64 = 5.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Timer {
    Idle,
    At(f64),
    Fast,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidTime,
    NotArmed,
}

pub struct Timing {
    timeout: f64,
    mean: f64,
    variation: f64,
    sample_seq: Option<u64>,
    timer: Timer,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            timeout: MIN_RTO,
            mean: 0.0,
            variation: 0.0,
            sample_seq: None,
            timer: Timer::Idle,
        }
    }
}

impl Timing {
    pub fn timeout(&self) -> f64 {
        self.timeout
    }
    pub fn mean(&self) -> f64 {
        self.mean
    }
    pub fn variation(&self) -> f64 {
        self.variation
    }
    pub fn timer(&self) -> Timer {
        self.timer
    }

    /// Notify the estimator after a successful fresh frame write. Time is
    /// the predicted end of that frame on the chosen wire.
    pub fn sent(&mut self, sequence: u64, end_at: f64, first: bool) -> Result<(), Error> {
        if !valid(end_at) {
            return Err(Error::InvalidTime);
        }
        if first {
            self.timer = Timer::At(end_at + self.timeout);
        }
        if self.sample_seq.is_none() && sequence != 0 {
            self.sample_seq = Some(sequence);
        }
        Ok(())
    }

    /// Report strictly advancing cumulative ACKs only. The predicted end
    /// time belongs to the last newly retired frame, as in stock. The
    /// first pending frame's wire time is supplied if one remains.
    pub fn acknowledged(
        &mut self,
        sequence: u64,
        event_at: f64,
        last_retired_end: Option<f64>,
        next_wire: Option<f64>,
    ) -> Result<(), Error> {
        if !valid(event_at)
            || last_retired_end.is_some_and(|t| !valid(t))
            || next_wire.is_some_and(|t| !valid(t))
        {
            return Err(Error::InvalidTime);
        }
        if let Some(end) = last_retired_end.filter(|t| *t != 0.0) {
            if self.sample_seq.is_some_and(|seq| sequence > seq) {
                let delta = event_at - end;
                if self.mean == 0.0 {
                    self.variation = delta / 2.0;
                    self.mean = delta * 10.0;
                } else {
                    self.variation = (3.0 * self.variation + (self.mean - delta).abs()) / 4.0;
                    self.mean = (7.0 * self.mean + delta) / 8.0;
                }
                self.timeout =
                    (self.mean + (4.0 * self.variation).max(0.001)).clamp(MIN_RTO, MAX_RTO);
                self.sample_seq = None;
            }
        }
        self.timer = next_wire.map_or(Timer::Idle, |wire| {
            Timer::At(event_at + self.timeout + wire)
        });
        Ok(())
    }

    pub fn arm_fast_retry(&mut self) -> Result<(), Error> {
        if self.timer == Timer::Idle {
            return Err(Error::NotArmed);
        }
        self.timer = Timer::Fast;
        Ok(())
    }

    /// Notify after a successful cached-frame replay. Stock times the next
    /// retry from the sync byte and first cached frame, not the full burst.
    pub fn retried(&mut self, cause: Retry, event_at: f64, first_wire: f64) -> Result<(), Error> {
        if !valid(event_at) || !valid(first_wire) {
            return Err(Error::InvalidTime);
        }
        if self.timer == Timer::Idle {
            return Err(Error::NotArmed);
        }
        if cause == Retry::Timeout {
            self.timeout = (self.timeout * 2.0).min(MAX_RTO);
        }
        self.sample_seq = None;
        self.timer = Timer::At(event_at + self.timeout + first_wire);
        Ok(())
    }
}

fn valid(t: f64) -> bool {
    (0.0..=1e12).contains(&t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_once_clears_on_retry_and_obeys_timeout_bounds() {
        let mut t = Timing::default();
        t.sent(17, 1.004, true).unwrap();
        assert_eq!(t.timer(), Timer::At(1.004 + MIN_RTO));
        t.acknowledged(18, 1.104, Some(1.004), None).unwrap();
        assert!((t.mean() - 1.0).abs() < 1e-12);
        assert!((t.timeout() - 1.2).abs() < 1e-12);
        t.sent(18, 2.0, true).unwrap();
        t.arm_fast_retry().unwrap();
        t.retried(Retry::DuplicateAck, 2.001, 0.1).unwrap();
        let rto = t.timeout();
        t.acknowledged(19, 2.6, Some(2.0), Some(0.05_f64)).unwrap();
        assert_eq!(t.timeout(), rto);
        for i in 0..5 {
            t.retried(Retry::Timeout, 4.0 + f64::from(i), 0.01).unwrap();
        }
        assert_eq!(t.timeout(), 5.0);
    }
}
