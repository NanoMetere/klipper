// SPDX-License-Identifier: GPL-3.0-only
//! Offline fan duty/kick and coalescing request queue.
//! Adapted from Klipper extras/fan.py and extras/output_pin.py.
//! Upstream copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>.
//! Outputs are inert data; no pins, PWM, or firmware access.

use std::collections::VecDeque;

const MAX_PENDING: usize = 4096;

#[derive(Debug, Clone, Copy)]
pub struct Settings {
    pub max_power: f64,
    pub off_below: f64,
    pub kick_seconds: f64,
    pub min_schedule_seconds: f64,
    pub enable_present: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Output {
    Pwm { clock: f64, duty: f64 },
    Enable { clock: f64, on: bool },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Settings,
    Request,
    Limit,
}

#[derive(Clone, Copy)]
struct Request {
    at: f64,
    requested: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Apply {
    Discard,
    Repeat(f64),
    Scheduled,
}

pub struct Fan {
    settings: Settings,
    queue: VecDeque<Request>,
    earliest: f64,
    requested: f64,
    scheduled: f64,
}

impl Fan {
    pub fn new(settings: Settings) -> Result<Self, Error> {
        if !(0.0..=1.0).contains(&settings.max_power)
            || settings.max_power == 0.0
            || !(0.0..=1.0).contains(&settings.off_below)
            || !(0.0..=300.0).contains(&settings.kick_seconds)
            || !(1e-6..=1.0).contains(&settings.min_schedule_seconds)
        {
            return Err(Error::Settings);
        }
        Ok(Self {
            settings,
            queue: VecDeque::new(),
            earliest: 0.,
            requested: 0.,
            scheduled: 0.,
        })
    }

    pub fn speed(&self) -> f64 {
        self.requested
    }

    pub fn next_min_flush_time(&self) -> f64 {
        self.earliest
    }

    pub fn queued(&self) -> usize {
        self.queue.len()
    }

    pub fn queue(&mut self, at: f64, requested: f64) -> Result<(), Error> {
        validate(at, requested)?;
        if self.queue.len() >= MAX_PENDING {
            return Err(Error::Limit);
        }
        if self.queue.back().is_some_and(|last| last.at > at) {
            return Err(Error::Request);
        }
        self.queue.push_back(Request { at, requested });
        Ok(())
    }

    /// Apply all available requests, using the latest request when several
    /// target the same permitted clock. A kick keeps the demand pending.
    pub fn flush(&mut self, until: f64) -> Result<Vec<Output>, Error> {
        validate(until, 0.0)?;
        let mut writes = Vec::new();
        while let Some(first) = self.queue.front() {
            let next = first.at.max(self.earliest);
            if next > until {
                break;
            }
            let covered = self.queue.iter().take_while(|r| r.at <= next).count();
            let newest = self.queue[covered - 1].requested;
            let keep = match self.apply(next, newest, &mut writes) {
                Apply::Discard => {
                    self.queue.drain(..covered);
                    continue;
                }
                Apply::Repeat(time) => {
                    self.earliest = self.earliest.max(time);
                    1
                }
                Apply::Scheduled => 0,
            };
            self.queue.drain(..covered - keep);
            self.earliest = self.earliest.max(next + self.settings.min_schedule_seconds);
        }
        Ok(writes)
    }

    /// Immediate caller-timed request, including any scheduled end of kick.
    pub fn asynchronous(&mut self, at: f64, requested: f64) -> Result<Vec<Output>, Error> {
        validate(at, requested)?;
        let mut writes = Vec::new();
        let next = at.max(self.earliest);
        match self.apply(next, requested, &mut writes) {
            Apply::Discard => return Ok(writes),
            Apply::Scheduled => {
                self.earliest = next + self.settings.min_schedule_seconds;
            }
            Apply::Repeat(time) => {
                self.earliest = time.max(next + self.settings.min_schedule_seconds);
                if self.apply(self.earliest, requested, &mut writes) == Apply::Scheduled {
                    self.earliest += self.settings.min_schedule_seconds;
                }
            }
        }
        Ok(writes)
    }

    fn apply(&mut self, clock: f64, mut value: f64, writes: &mut Vec<Output>) -> Apply {
        if value < self.settings.off_below {
            value = 0.0;
        }
        let duty = (value * self.settings.max_power).clamp(0.0, self.settings.max_power);
        if duty == self.scheduled {
            return Apply::Discard;
        }
        if self.settings.enable_present && (duty > 0.0) != (self.scheduled > 0.0) {
            writes.push(Output::Enable {
                clock,
                on: duty > 0.0,
            });
        }
        self.requested = duty;
        if duty > 0.0
            && self.settings.kick_seconds != 0.0
            && (self.scheduled == 0.0 || duty - self.scheduled > 0.5)
        {
            self.scheduled = self.settings.max_power;
            writes.push(Output::Pwm {
                clock,
                duty: self.scheduled,
            });
            Apply::Repeat(clock + self.settings.kick_seconds)
        } else {
            self.scheduled = duty;
            writes.push(Output::Pwm { clock, duty });
            Apply::Scheduled
        }
    }
}

fn validate(clock: f64, requested: f64) -> Result<(), Error> {
    if (0.0..=1e8).contains(&clock) && requested.is_finite() {
        Ok(())
    } else {
        Err(Error::Request)
    }
}

pub fn tachometer_rpm(frequency: f64, pulses_per_revolution: u32) -> Result<f64, Error> {
    if pulses_per_revolution == 0 || !frequency.is_finite() || frequency < 0.0 {
        return Err(Error::Request);
    }
    let rpm = frequency * 30.0 / f64::from(pulses_per_revolution);
    rpm.is_finite().then_some(rpm).ok_or(Error::Request)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coalesces_superseded_demands_and_honors_enable_kick_and_repeat() {
        let mut fan = Fan::new(Settings {
            max_power: 0.8,
            off_below: 0.1,
            kick_seconds: 0.25,
            min_schedule_seconds: 0.001,
            enable_present: true,
        })
        .unwrap();
        fan.queue(1., 0.3).unwrap();
        fan.queue(1., 0.5).unwrap();
        assert_eq!(
            fan.flush(1.).unwrap(),
            [
                Output::Enable {
                    clock: 1.,
                    on: true
                },
                Output::Pwm {
                    clock: 1.,
                    duty: 0.8
                }
            ]
        );
        assert_eq!(fan.speed(), 0.4);
        assert_eq!(fan.queued(), 1);
        assert!(fan.flush(1.24).unwrap().is_empty());
        assert_eq!(
            fan.flush(1.25).unwrap(),
            [Output::Pwm {
                clock: 1.25,
                duty: 0.4
            }]
        );
        assert_eq!(
            fan.asynchronous(3., 0.).unwrap(),
            [
                Output::Enable {
                    clock: 3.,
                    on: false
                },
                Output::Pwm {
                    clock: 3.,
                    duty: 0.
                }
            ]
        );
    }
}
