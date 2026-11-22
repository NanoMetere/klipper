// SPDX-License-Identifier: GPL-3.0-only
//! Primary MCU clock estimation, without an I/O or print scheduler.
//! Adapted from Klipper's klippy/clocksync.py.
//! Upstream copyright (C) 2016-2018 Kevin O'Connor <kevin@koconnor.net>.

const RTT_AGE: f64 = 0.000010 / (60.0 * 60.0);
const DECAY: f64 = 1.0 / 30.0;
const TRANSMIT_EXTRA: f64 = 0.001;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    pub at: f64,
    pub ticks: f64,
    pub hz: f64,
}

impl Estimate {
    /// Float tick estimate, before applying the caller's integer rounding.
    pub fn at_time(self, time: f64) -> f64 {
        self.ticks + (time - self.at) * self.hz
    }

    pub fn time_for_tick(self, tick: u64) -> f64 {
        (tick as f64 - self.ticks) / self.hz + self.at
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReleaseEstimate {
    pub at: f64,
    pub ticks: i64,
    pub hz: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sample {
    /// Retransmitted reply without a valid send timestamp. Only extend ticks.
    Untimed,
    Timed {
        sent: f64,
        received: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Update {
    Untimed,
    Outlier,
    Updated(ReleaseEstimate),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidFrequency,
    InvalidTime,
    ClockOverflow,
    DegenerateEstimate,
}

pub struct PrimaryClock {
    nominal: f64,
    estimate: Estimate,
    clock64: u64,
    time_avg: f64,
    time_var: f64,
    clock_avg: f64,
    covariance: f64,
    pred_var: f64,
    last_prediction: f64,
    min_half_rtt: f64,
    min_rtt_time: f64,
}

impl PrimaryClock {
    /// Initialize from an explicit firmware uptime and its host send time.
    /// Reinitialize on restart/disconnect or any gap spanning a full 32-bit
    /// counter period; wrap and a reboot cannot be distinguished using low bits.
    pub fn from_uptime(nominal: f64, ticks: u64, sent: f64) -> Result<Self, Error> {
        if !nominal.is_finite() || nominal <= 0.0 || nominal > 1e12 {
            return Err(Error::InvalidFrequency);
        }
        if !sent.is_finite() || sent < 0.0 {
            return Err(Error::InvalidTime);
        }
        let clock = ticks as f64;
        Ok(Self {
            nominal,
            estimate: Estimate {
                at: sent,
                ticks: clock,
                hz: nominal,
            },
            clock64: ticks,
            time_avg: sent,
            time_var: 0.0,
            clock_avg: clock,
            covariance: 0.0,
            pred_var: variance_seed(nominal),
            last_prediction: 0.0,
            min_half_rtt: 999_999_999.9,
            min_rtt_time: 0.0,
        })
    }

    pub fn estimate(&self) -> Estimate {
        self.estimate
    }

    pub fn last_clock(&self) -> u64 {
        self.clock64
    }

    /// Extend an asynchronous report to the nearest tick epoch, not
    /// necessarily after the last clock sample.
    pub fn nearest_tick(&self, low: u32) -> Option<u64> {
        let delta = low.wrapping_sub(self.clock64 as u32) as i32;
        self.clock64.checked_add_signed(i64::from(delta))
    }

    /// Update using an in-order measurement from this MCU. Pass `warmup`
    /// for each of stock Klippy's eight initial get_clock reads.
    pub fn observe(&mut self, low: u32, sample: Sample, warmup: bool) -> Result<Update, Error> {
        let delta = low.wrapping_sub(self.clock64 as u32);
        let tick = self
            .clock64
            .checked_add(u64::from(delta))
            .ok_or(Error::ClockOverflow)?;
        let Sample::Timed { sent, received } = sample else {
            self.clock64 = tick;
            return Ok(Update::Untimed);
        };
        if !sent.is_finite() || !received.is_finite() || received < sent || sent < 0.0 {
            return Err(Error::InvalidTime);
        }
        let mut last_pred = if warmup {
            -9999.0
        } else {
            self.last_prediction
        };
        let diff_time = sent - self.time_avg;
        let mut pred_var = self.pred_var;
        let predicted = diff_time * self.estimate.hz + self.clock_avg;
        let clock = tick as f64;
        let diff2 = (clock - predicted) * (clock - predicted);
        let threshold = 0.000500 * self.nominal;
        if diff2 > 25.0 * pred_var && diff2 > threshold * threshold {
            if clock > predicted && sent < last_pred + 10.0 {
                self.clock64 = tick;
                return Ok(Update::Outlier);
            }
            pred_var = variance_seed(self.nominal);
        } else {
            last_pred = sent;
            pred_var = (1.0 - DECAY) * (pred_var + diff2 * DECAY);
        }
        let time_avg = self.time_avg + DECAY * diff_time;
        let time_var = (1.0 - DECAY) * (self.time_var + (diff_time * diff_time) * DECAY);
        let diff_clock = clock - self.clock_avg;
        let clock_avg = self.clock_avg + DECAY * diff_clock;
        let covariance = (1.0 - DECAY) * (self.covariance + diff_time * diff_clock * DECAY);
        let hz = covariance / time_var;
        let release = clock_avg - 3.0 * pred_var.sqrt();
        if !hz.is_finite()
            || hz <= 0.0
            || !release.is_finite()
            || release < i64::MIN as f64
            || release >= -(i64::MIN as f64)
        {
            return Err(Error::DegenerateEstimate);
        }

        let half_rtt = 0.5 * (received - sent);
        if half_rtt < self.min_half_rtt + (sent - self.min_rtt_time) * RTT_AGE {
            self.min_half_rtt = half_rtt;
            self.min_rtt_time = sent;
        }
        self.clock64 = tick;
        self.time_avg = time_avg;
        self.time_var = time_var;
        self.clock_avg = clock_avg;
        self.covariance = covariance;
        self.pred_var = pred_var;
        self.last_prediction = last_pred;
        self.estimate = Estimate {
            at: time_avg + self.min_half_rtt,
            ticks: clock_avg,
            hz,
        };
        Ok(Update::Updated(ReleaseEstimate {
            at: time_avg + TRANSMIT_EXTRA,
            ticks: release as i64,
            hz,
        }))
    }
}

fn variance_seed(nominal: f64) -> f64 {
    let jitter = 0.001 * nominal;
    jitter * jitter
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wraps_and_handles_missing_send_time() {
        let base = u64::from(u32::MAX) - 10;
        let mut c = PrimaryClock::from_uptime(1_000_000.0, base, 50.0).unwrap();
        let previous = c.estimate();
        assert_eq!(c.observe(8, Sample::Untimed, false), Ok(Update::Untimed));
        assert_eq!(c.last_clock(), (1 << 32) + 8);
        assert_eq!(c.nearest_tick(u32::MAX), Some(u64::from(u32::MAX)));
        assert_eq!(c.estimate(), previous);
    }

    #[test]
    fn invalid_sample_does_not_change_state_and_reboot_is_explicit() {
        let mut c = PrimaryClock::from_uptime(1e6, 10, 20.0).unwrap();
        let prev = c.estimate();
        assert_eq!(
            c.observe(
                11,
                Sample::Timed {
                    sent: 25.0,
                    received: 24.0
                },
                true
            ),
            Err(Error::InvalidTime)
        );
        assert_eq!(
            c.observe(
                11,
                Sample::Timed {
                    sent: 20.0,
                    received: 20.0
                },
                true
            ),
            Err(Error::DegenerateEstimate)
        );
        assert_eq!(c.last_clock(), 10);
        assert_eq!(c.estimate(), prev);
        assert_eq!(
            PrimaryClock::from_uptime(1e6, 2, 30.0)
                .unwrap()
                .last_clock(),
            2
        );
        let mut c = PrimaryClock::from_uptime(1e6, u64::MAX, 0.0).unwrap();
        assert_eq!(
            c.observe(0, Sample::Untimed, false),
            Err(Error::ClockOverflow)
        );
    }
}

/// Translation between a primary MCU's print timeline and a secondary MCU.
/// An independent PrimaryClock must track each MCU's own host/tick relation.
pub struct SecondaryClock {
    adjustment: PrintClock,
    last_sync_time: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrintClock {
    pub offset: f64,
    pub hz: f64,
}

impl PrintClock {
    pub fn tick_at(self, print_time: f64) -> Option<i64> {
        tick_i64((print_time - self.offset) * self.hz)
    }

    pub fn print_time_at(self, ticks: i64) -> f64 {
        ticks as f64 / self.hz + self.offset
    }
}

fn tick_i64(value: f64) -> Option<i64> {
    (value.is_finite() && value >= i64::MIN as f64 && value < -(i64::MIN as f64))
        .then_some(value as i64)
}

impl SecondaryClock {
    pub fn connect(main: &PrimaryClock, local: &PrimaryClock, at: f64) -> Result<Self, Error> {
        let main_clock = tick_i64(main.estimate.at_time(at)).ok_or(Error::DegenerateEstimate)?;
        let local_clock = tick_i64(local.estimate.at_time(at)).ok_or(Error::DegenerateEstimate)?;
        let mut this = Self {
            adjustment: PrintClock {
                offset: main_clock as f64 / main.nominal - local_clock as f64 / local.nominal,
                hz: local.nominal,
            },
            last_sync_time: 0.0,
        };
        this.calibrate(main, local, 0.0, at)?;
        Ok(this)
    }

    pub fn adjustment(&self) -> PrintClock {
        self.adjustment
    }

    pub fn horizon(&self) -> f64 {
        self.last_sync_time
    }

    /// Keep the existing translation at the nearer anchor and match the
    /// primary's print time at a later anchor (at least four seconds out).
    pub fn calibrate(
        &mut self,
        main: &PrimaryClock,
        local: &PrimaryClock,
        print_time: f64,
        at: f64,
    ) -> Result<PrintClock, Error> {
        if !print_time.is_finite() || !at.is_finite() || at < 0.0 {
            return Err(Error::InvalidTime);
        }
        let estimated_print_time = main.estimate.at_time(at) / main.nominal;
        let near = print_time.max(estimated_print_time);
        let far = (near + 4.0)
            .max(self.last_sync_time)
            .max(print_time + 2.5 * (print_time - estimated_print_time));
        let far_sys =
            main.estimate.at + (far * main.nominal - main.estimate.ticks) / main.estimate.hz;
        let near_tick = self
            .adjustment
            .tick_at(near)
            .ok_or(Error::DegenerateEstimate)?;
        let far_tick =
            tick_i64(local.estimate.at_time(far_sys)).ok_or(Error::DegenerateEstimate)?;
        let difference = i128::from(far_tick) - i128::from(near_tick);
        let hz = difference as f64 / (far - near);
        let offset = near - near_tick as f64 / hz;
        if !hz.is_finite() || hz <= 0.0 || !offset.is_finite() {
            return Err(Error::DegenerateEstimate);
        }
        self.adjustment = PrintClock { offset, hz };
        self.last_sync_time = far;
        Ok(self.adjustment)
    }
}

#[cfg(test)]
mod secondary_tests {
    use super::*;

    #[test]
    fn keeps_near_anchor_and_does_not_move_horizon_backwards() {
        let main = PrimaryClock::from_uptime(1e6, 2_000_000, 20.0).unwrap();
        let local = PrimaryClock::from_uptime(2e6, 9_000_000, 20.0).unwrap();
        let mut adjustment = SecondaryClock::connect(&main, &local, 20.0).unwrap();
        assert_eq!(adjustment.adjustment().tick_at(2.0), Some(9_000_000));
        let old = adjustment.adjustment();
        let horizon = adjustment.horizon();
        let new = adjustment.calibrate(&main, &local, 3.0, 20.5).unwrap();
        assert_eq!(new.tick_at(3.0), old.tick_at(3.0));
        assert!(adjustment.horizon() >= horizon);
        let prev = adjustment.adjustment();
        assert_eq!(
            adjustment.calibrate(&main, &local, f64::NAN, 21.0),
            Err(Error::InvalidTime)
        );
        assert_eq!(adjustment.adjustment(), prev);
    }
}
