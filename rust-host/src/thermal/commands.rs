// SPDX-License-Identifier: GPL-3.0-only
//! Inert host heater routing. Store targets and assess supplied observations;
//! do not open a sensor, schedule hardware or emit a PWM request.

pub use super::commands_parse::parse_gcode;

const MAX_HEATERS: usize = 32;
const MAX_SENSORS: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Config,
    UnknownHeater,
    UnknownSensor,
    Target,
    Range,
    Observation,
    Syntax,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Heater<'a> {
    /// None selects the active extruder. Numbers are stock heater indices,
    /// not physical Toolchanger IDs.
    Extruder(Option<u32>),
    Bed,
    Named(&'a str),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command<'a> {
    Set {
        heater: Heater<'a>,
        target: f64,
        wait: bool,
    },
    Off,
    WaitSensor {
        name: &'a str,
        minimum: Option<f64>,
        maximum: Option<f64>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Extruder(u32),
    Bed,
    Named,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Profile<'a> {
    pub name: &'a str,
    pub role: Role,
    pub minimum: f64,
    pub maximum: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Output<'a> {
    Set {
        index: usize,
        wait: Option<Wait<'a>>,
    },
    AllOff,
    Ignored,
    Range(Wait<'a>),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Wait<'a> {
    Controller {
        index: usize,
        target: f64,
    },
    Range {
        source: Source<'a>,
        minimum: f64,
        maximum: f64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source<'a> {
    Heater(usize),
    Sensor(&'a str),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation<'a> {
    pub source: Source<'a>,
    pub sampled_at: f64,
    pub temperature: f64,
    pub controller_busy: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gate {
    Pending,
    Ready,
    Shutdown,
}

impl Wait<'_> {
    /// The caller supplies simulation observations. Unlike stock's sensor
    /// reporting fallback, missing or stale data cannot satisfy a wait.
    pub fn poll(
        self,
        now: f64,
        maximum_age: f64,
        observation: Option<Observation<'_>>,
        shutdown: bool,
    ) -> Result<Gate, Error> {
        if shutdown {
            return Ok(Gate::Shutdown);
        }
        if !now.is_finite() || !maximum_age.is_finite() || maximum_age < 0.0 {
            return Err(Error::Observation);
        }
        let Some(sample) = observation else {
            return Ok(Gate::Pending);
        };
        let source = match self {
            Self::Controller { index, .. } => Source::Heater(index),
            Self::Range { source, .. } => source,
        };
        if source != sample.source
            || !sample.sampled_at.is_finite()
            || !sample.temperature.is_finite()
            || sample.sampled_at > now
        {
            return Err(Error::Observation);
        }
        if now - sample.sampled_at > maximum_age {
            return Ok(Gate::Pending);
        }
        let ready = match self {
            Self::Controller { .. } => !sample.controller_busy,
            Self::Range {
                minimum, maximum, ..
            } => (minimum..=maximum).contains(&sample.temperature),
        };
        Ok(if ready { Gate::Ready } else { Gate::Pending })
    }
}

#[derive(Clone)]
pub struct Bank<'a> {
    profiles: &'a [Profile<'a>],
    sensors: &'a [&'a str],
    targets: [f64; MAX_HEATERS],
    active: Option<u32>,
    debug: bool,
}

impl<'a> Bank<'a> {
    pub fn new(
        profiles: &'a [Profile<'a>],
        sensors: &'a [&'a str],
        active: Option<u32>,
        debug: bool,
    ) -> Result<Self, Error> {
        if profiles.len() > MAX_HEATERS
            || sensors.len() > MAX_SENSORS
            || profiles.iter().enumerate().any(|(i, p)| {
                p.name.trim().is_empty()
                    || !p.minimum.is_finite()
                    || !p.maximum.is_finite()
                    || p.maximum <= p.minimum
                    || profiles[..i]
                        .iter()
                        .any(|q| q.name == p.name || (p.role != Role::Named && q.role == p.role))
            })
            || sensors.iter().enumerate().any(|(i, s)| {
                s.trim().is_empty()
                    || sensors[..i].contains(s)
                    || profiles.iter().any(|p| &p.name == s)
            })
        {
            return Err(Error::Config);
        }
        let count = profiles
            .iter()
            .filter(|p| matches!(p.role, Role::Extruder(_)))
            .count() as u32;
        if profiles
            .iter()
            .any(|p| matches!(p.role, Role::Extruder(n) if n >= count))
        {
            return Err(Error::Config);
        }
        let mut result = Self {
            profiles,
            sensors,
            targets: [0.; MAX_HEATERS],
            active: None,
            debug,
        };
        if let Some(index) = active {
            result.activate(index)?;
        }
        Ok(result)
    }

    pub fn activate(&mut self, index: u32) -> Result<(), Error> {
        if !self
            .profiles
            .iter()
            .any(|p| p.role == Role::Extruder(index))
        {
            return Err(Error::UnknownHeater);
        }
        self.active = Some(index);
        Ok(())
    }

    pub fn targets(&self) -> &[f64] {
        &self.targets[..self.profiles.len()]
    }

    pub fn command(&mut self, command: Command<'_>) -> Result<Output<'a>, Error> {
        match command {
            Command::Set {
                heater,
                target,
                wait,
            } => {
                if !target.is_finite() {
                    return Err(Error::Target);
                }
                let selected = match heater {
                    Heater::Extruder(index) => index.or(self.active).and_then(|n| {
                        self.profiles
                            .iter()
                            .position(|p| p.role == Role::Extruder(n))
                    }),
                    Heater::Bed => self.profiles.iter().position(|p| p.role == Role::Bed),
                    Heater::Named(name) => self.profiles.iter().position(|p| p.name == name),
                };
                let Some(index) = selected else {
                    if matches!(heater, Heater::Extruder(Some(_))) && target <= 0. {
                        return Ok(Output::Ignored);
                    }
                    return Err(Error::UnknownHeater);
                };
                let p = &self.profiles[index];
                if target != 0. && !(p.minimum..=p.maximum).contains(&target) {
                    return Err(Error::Target);
                }
                self.targets[index] = target;
                Ok(Output::Set {
                    index,
                    wait: (wait && target != 0. && !self.debug)
                        .then_some(Wait::Controller { index, target }),
                })
            }
            Command::Off => {
                self.targets.fill(0.);
                Ok(Output::AllOff)
            }
            Command::WaitSensor {
                name,
                minimum,
                maximum,
            } => {
                if minimum.is_some_and(|v| !v.is_finite())
                    || maximum.is_some_and(|v| !v.is_finite())
                    || (minimum.is_none() && maximum.is_none())
                {
                    return Err(Error::Range);
                }
                let minimum = minimum.unwrap_or(f64::NEG_INFINITY);
                let maximum = maximum.unwrap_or(f64::INFINITY);
                if minimum >= maximum {
                    return Err(Error::Range);
                }
                let source = if let Some(index) = self.profiles.iter().position(|p| p.name == name)
                {
                    Source::Heater(index)
                } else if let Some(&sensor) = self.sensors.iter().find(|&&s| s == name) {
                    Source::Sensor(sensor)
                } else {
                    return Err(Error::UnknownSensor);
                };
                Ok(if self.debug {
                    Output::Ignored
                } else {
                    Output::Range(Wait::Range {
                        source,
                        minimum,
                        maximum,
                    })
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_observation_must_match_the_wait_source_and_be_fresh() {
        let mut bank = Bank::new(&[], &["ambient", "other"], None, false).unwrap();
        let request = Command::WaitSensor {
            name: "ambient",
            minimum: Some(20.),
            maximum: Some(25.),
        };
        let Output::Range(wait) = bank.command(request).unwrap() else {
            panic!("sensor pending");
        };
        let reading = Observation {
            source: Source::Sensor("ambient"),
            sampled_at: 2.,
            temperature: 25.,
            controller_busy: true,
        };
        assert_eq!(wait.poll(2.5, 1., Some(reading), false), Ok(Gate::Ready));
        assert_eq!(wait.poll(4., 1., Some(reading), false), Ok(Gate::Pending));
        assert_eq!(wait.poll(4., 1., None, false), Ok(Gate::Pending));
        assert_eq!(wait.poll(4., 1., None, true), Ok(Gate::Shutdown));
        assert_eq!(
            wait.poll(1., 1., Some(reading), false),
            Err(Error::Observation)
        );
        assert_eq!(
            wait.poll(
                2.5,
                1.,
                Some(Observation {
                    source: Source::Sensor("other"),
                    ..reading
                }),
                false
            ),
            Err(Error::Observation)
        );
        assert!(matches!(
            Bank::new(&[], &["ambient"], None, true)
                .unwrap()
                .command(request),
            Ok(Output::Ignored)
        ));
    }

    #[test]
    fn nonfinite_targets_and_index_gaps_are_rejected() {
        let mut one_heater = [Profile {
            name: "extruder",
            role: Role::Extruder(0),
            minimum: 0.,
            maximum: 300.,
        }];
        let mut bank = Bank::new(&one_heater, &[], Some(0), false).unwrap();
        assert_eq!(
            bank.command(Command::Set {
                heater: Heater::Extruder(None),
                target: f64::NAN,
                wait: false
            }),
            Err(Error::Target)
        );
        assert_eq!(bank.targets(), [0.]);
        one_heater[0].role = Role::Extruder(1);
        assert!(matches!(
            Bank::new(&one_heater, &[], None, false),
            Err(Error::Config)
        ));
    }
}
