// SPDX-License-Identifier: GPL-3.0-only
//! Offline thermal controller math, adapted from Klipper extras/heaters.py.
//! Upstream copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>.
//! No sensor, PWM or transport integration. Not suitable for driving a heater.
pub mod commands;
mod commands_parse;
pub mod script;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    Parameters,
    Sensor,
    Clock,
    Latched,
}

#[derive(Debug, Clone, Copy)]
pub enum Mode {
    Watermark { delta: f64 },
    Pid { kp: f64, ki: f64, kd: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    pub modeled_duty: f64,
    pub smoothed_temp: f64,
    pub busy: bool,
}

pub struct Model {
    mode: Mode,
    min: f64,
    max: f64,
    power: f64,
    smoothing: f64,
    last: f64,
    at: f64,
    derivative: f64,
    integral: f64,
    smoothed: f64,
    heating: bool,
    failed: bool,
}

impl Model {
    pub fn new(mode: Mode, min: f64, max: f64, power: f64, smoothing: f64) -> Result<Self, Fault> {
        if !min.is_finite()
            || !max.is_finite()
            || min < -273.15
            || max <= min
            || !power.is_finite()
            || !(0.0 < power && power <= 1.0)
            || !smoothing.is_finite()
            || smoothing <= 0.0
        {
            return Err(Fault::Parameters);
        }
        match mode {
            Mode::Watermark { delta } if !delta.is_finite() || delta <= 0.0 => {
                return Err(Fault::Parameters)
            }
            Mode::Pid { kp, ki, kd } if [kp, ki, kd].iter().any(|v| !v.is_finite() || *v < 0.0) => {
                return Err(Fault::Parameters)
            }
            _ => {}
        }
        Ok(Self {
            mode,
            min,
            max,
            power,
            smoothing,
            last: 25.0,
            at: 0.0,
            derivative: 0.0,
            integral: 0.0,
            smoothed: 0.0,
            heating: false,
            failed: false,
        })
    }

    /// Evaluate one sample in the simulation. A faulty sensor or clock latches
    /// failure; invalid target requests are rejected without consuming input.
    pub fn update(
        &mut self,
        at: f64,
        temp: f64,
        target: f64,
        host_live: bool,
    ) -> Result<Sample, Fault> {
        if self.failed {
            return Err(Fault::Latched);
        }
        if !target.is_finite() || (target != 0.0 && !(self.min..=self.max).contains(&target)) {
            return Err(Fault::Parameters);
        }
        if !temp.is_finite() || !(self.min..=self.max).contains(&temp) {
            self.failed = true;
            return Err(Fault::Sensor);
        }
        if !at.is_finite() || at < self.at {
            self.failed = true;
            return Err(Fault::Clock);
        }
        let dt = at - self.at;
        let modeled = match self.mode {
            Mode::Watermark { delta } => {
                if self.heating && temp >= target + delta {
                    self.heating = false;
                } else if !self.heating && temp <= target - delta {
                    self.heating = true;
                }
                if self.heating {
                    self.power
                } else {
                    0.0
                }
            }
            Mode::Pid { kp, ki, kd } => {
                let (kp, ki, kd) = (kp / 255.0, ki / 255.0, kd / 255.0);
                let diff = temp - self.last;
                self.derivative = if dt >= self.smoothing {
                    diff / dt
                } else {
                    (self.derivative * (self.smoothing - dt) + diff) / self.smoothing
                };
                let error = target - temp;
                let max_i = if ki > 0.0 { self.power / ki } else { 0.0 };
                let integral = (self.integral + error * dt).max(0.0).min(max_i);
                let duty = kp * error + ki * integral - kd * self.derivative;
                if !duty.is_finite() || !self.derivative.is_finite() {
                    self.failed = true;
                    return Err(Fault::Sensor);
                }
                let bounded = duty.clamp(0.0, self.power);
                if duty == bounded {
                    self.integral = integral;
                }
                bounded
            }
        };
        self.last = temp;
        self.at = at;
        self.smoothed += (temp - self.smoothed) * (dt * (1.0 / self.smoothing)).min(1.0);
        let busy = match self.mode {
            Mode::Watermark { delta } => self.smoothed < target - delta,
            Mode::Pid { .. } => (target - self.smoothed).abs() > 1.0 || self.derivative.abs() > 0.1,
        };
        Ok(Sample {
            modeled_duty: if host_live && target > 0.0 {
                modeled
            } else {
                0.0
            },
            smoothed_temp: self.smoothed,
            busy,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zeroes_simulated_request_when_host_is_stale_or_target_off() {
        let mut m = Model::new(Mode::Watermark { delta: 2.0 }, -20., 325., 0.7, 0.5).unwrap();
        assert_eq!(m.update(1., 25., 100., true).unwrap().modeled_duty, 0.7);
        assert_eq!(m.update(2., 25., 100., false).unwrap().modeled_duty, 0.0);
        assert_eq!(m.update(3., 25., 0., true).unwrap().modeled_duty, 0.0);
    }

    #[test]
    fn sensor_and_clock_faults_latch_and_bad_targets_do_not_mutate() {
        let pid = Mode::Pid {
            kp: 20.,
            ki: 1.,
            kd: 70.,
        };
        let mut m = Model::new(pid, -20., 325., 0.7, 0.5).unwrap();
        assert_eq!(m.update(4., 25., 999., true), Err(Fault::Parameters));
        m.update(2., 25., 55., true).unwrap();
        assert_eq!(m.update(3., f64::NAN, 55., true), Err(Fault::Sensor));
        assert_eq!(m.update(4., 25., 55., true), Err(Fault::Latched));
        let mut m = Model::new(pid, -20., 325., 0.7, 0.5).unwrap();
        m.update(2., 25., 55., true).unwrap();
        assert_eq!(m.update(1., 25., 55., true), Err(Fault::Clock));
        assert_eq!(m.update(3., 25., 0., false), Err(Fault::Latched));
    }
}
