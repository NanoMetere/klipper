// SPDX-License-Identifier: GPL-3.0-only
//! Offline heater verification adapted from Klipper extras/verify_heater.py.
//! Upstream copyright (C) 2018 Kevin O'Connor <kevin@koconnor.net>.
//! No sensor, PWM, heater, or watchdog device is opened or updated.

#[derive(Debug, Clone, Copy)]
pub struct Settings {
    pub hysteresis: f64,
    pub max_error: f64,
    pub heating_gain: f64,
    pub check_gain_time: f64,
}

impl Settings {
    pub fn standard(bed: bool) -> Self {
        Self {
            hysteresis: 5.0,
            max_error: 120.0,
            heating_gain: 2.0,
            check_gain_time: if bed { 60.0 } else { 20.0 },
        }
    }

    fn valid(self) -> bool {
        (0.0..=2_000.0).contains(&self.hysteresis)
            && (0.0..=100_000.0).contains(&self.max_error)
            && (0.0..=2_000.0).contains(&self.heating_gain)
            && self.heating_gain > 0.0
            && (1.0..=3600.0).contains(&self.check_gain_time)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Settings,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    NotHeating,
    InvalidObservation,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Decision {
    Continue { next: f64 },
    Fault(Fault),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Snapshot {
    pub approaching: bool,
    pub starting: bool,
    pub target: f64,
    pub goal: f64,
    pub error: f64,
    pub at_goal: Option<f64>,
    pub fault: Option<Fault>,
}

pub struct Monitor {
    settings: Settings,
    state: Snapshot,
    last: Option<f64>,
}

impl Monitor {
    pub fn new(settings: Settings) -> Result<Self, Error> {
        if !settings.valid() {
            return Err(Error::Settings);
        }
        Ok(Self {
            settings,
            state: Snapshot {
                approaching: false,
                starting: false,
                target: 0.0,
                goal: 0.0,
                error: 0.0,
                at_goal: None,
                fault: None,
            },
            last: None,
        })
    }

    pub fn snapshot(&self) -> Snapshot {
        self.state
    }

    /// Consume one synthetic, stock-cadence reading. Invalid or reordered
    /// telemetry is a distinct sticky failure; no heater action is taken.
    pub fn sample(&mut self, at: f64, temperature: f64, target: f64) -> Decision {
        if let Some(fault) = self.state.fault {
            return Decision::Fault(fault);
        }
        if !(0.0..=1e12).contains(&at)
            || self.last.is_some_and(|last| at <= last)
            || !(-273.15..=2000.0).contains(&temperature)
            || !(-273.15..=2000.0).contains(&target)
        {
            return self.fault(Fault::InvalidObservation);
        }
        self.last = Some(at);
        let cfg = self.settings;
        let state = &mut self.state;
        if target <= 0.0 || temperature >= target - cfg.hysteresis {
            state.approaching = false;
            state.starting = false;
            if temperature <= target + cfg.hysteresis {
                state.error = 0.0;
            }
        } else {
            state.error += target - cfg.hysteresis - temperature;
            if !state.approaching {
                if target != state.target {
                    state.approaching = true;
                    state.starting = true;
                    state.goal = temperature + cfg.heating_gain;
                    state.at_goal = Some(at + cfg.check_gain_time);
                } else if state.error >= cfg.max_error {
                    return self.fault(Fault::NotHeating);
                }
            } else if temperature >= state.goal {
                state.starting = false;
                state.error = 0.0;
                state.goal = temperature + cfg.heating_gain;
                state.at_goal = Some(at + cfg.check_gain_time);
            } else if state.at_goal.is_some_and(|deadline| at >= deadline) {
                state.approaching = false;
            } else if state.starting {
                state.goal = state.goal.min(temperature + cfg.heating_gain);
            }
        }
        state.target = target;
        Decision::Continue { next: at + 1.0 }
    }

    fn fault(&mut self, fault: Fault) -> Decision {
        self.state.fault = Some(fault);
        Decision::Fault(fault)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heating_failure_and_stale_telemetry_are_sticky() {
        let mut bed = Monitor::new(Settings::standard(true)).unwrap();
        for at in 0..=60 {
            assert_eq!(
                bed.sample(f64::from(at), 20.0, 100.0),
                Decision::Continue {
                    next: f64::from(at) + 1.0
                }
            );
        }
        assert_eq!(
            bed.sample(61., 20., 100.),
            Decision::Fault(Fault::NotHeating)
        );
        assert_eq!(
            bed.sample(62., 100., 100.),
            Decision::Fault(Fault::NotHeating)
        );

        let mut extruder = Monitor::new(Settings::standard(false)).unwrap();
        assert_eq!(
            extruder.sample(10., 190., 200.),
            Decision::Continue { next: 11. }
        );
        assert_eq!(
            extruder.sample(10., 200., 200.),
            Decision::Fault(Fault::InvalidObservation)
        );
        assert_eq!(
            extruder.sample(12., 200., 200.),
            Decision::Fault(Fault::InvalidObservation)
        );
    }
}
