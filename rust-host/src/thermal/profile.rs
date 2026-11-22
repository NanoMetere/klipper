// SPDX-License-Identifier: GPL-3.0-only
//! Read stock heater/sensor names and limits for the offline mixed preview.

use super::commands::{Bank, Error, Profile, Role};
use crate::config::Config;

pub struct Catalog<'a> {
    heaters: Vec<Profile<'a>>,
    sensors: Vec<&'a str>,
}

impl<'a> Catalog<'a> {
    pub fn read(config: &'a Config) -> Result<Self, Error> {
        let mut heaters = Vec::new();
        let mut sensors = Vec::new();
        for section in config.sections() {
            let role = if section == "heater_bed" {
                Role::Bed
            } else if let Some(index) = section.strip_prefix("extruder") {
                if index.is_empty() {
                    Role::Extruder(0)
                } else if index.bytes().all(|v| v.is_ascii_digit()) && !index.starts_with('0') {
                    Role::Extruder(index.parse().map_err(|_| Error::Config)?)
                } else {
                    continue;
                }
            } else if section.starts_with("temperature_sensor ") {
                sensors.push(section);
                continue;
            } else {
                continue;
            };
            let limit = |key| {
                config
                    .get(section, key)
                    .and_then(|v| v.parse::<f64>().ok())
                    .filter(|x| x.is_finite())
                    .ok_or(Error::Config)
            };
            let minimum = limit("min_temp")?;
            let maximum = limit("max_temp")?;
            if minimum < -273.15 || maximum <= minimum {
                return Err(Error::Config);
            }
            heaters.push(Profile {
                name: section,
                role,
                minimum,
                maximum,
            });
        }
        let result = Self { heaters, sensors };
        result.bank(false)?;
        Ok(result)
    }

    pub fn names(&self) -> impl Iterator<Item = &'a str> + '_ {
        self.heaters.iter().map(|p| p.name)
    }

    pub fn bank(&self, debug: bool) -> Result<Bank<'_>, Error> {
        let active = self
            .heaters
            .iter()
            .any(|p| p.role == Role::Extruder(0))
            .then_some(0);
        Bank::new(&self.heaters, &self.sensors, active, debug)
    }
}
