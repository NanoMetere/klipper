// SPDX-License-Identifier: GPL-3.0-only
//! Static offline tool/extruder profile binding from Klipper-style config.
//! Heater ranges constrain caller-supplied modeled temperatures only;
//! pin strings are not used to configure hardware.

use super::{gearing, SimulatedExtruder, ToolLane};
use crate::{
    config::Config,
    extrusion::{Extruder, Settings},
    pressure_advance::Advance,
    toolchange::Catalog,
};

const MAX_PROFILES: usize = 8;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    Unsupported,
    MissingExtruder,
    ExtruderGap,
    ToolCatalog,
    SharedExtruder,
    Field(String, &'static str),
    Temperature(u8),
}

pub struct Profile {
    pub id: u8,
    pub section: String,
    pub step_distance: f64,
    geometry: Extruder,
    temperature: (f64, f64),
    direction: bool,
    advance: Option<Advance>,
}

impl Profile {
    pub fn lane(&self, simulated_temperature: f64, parked_e: f64) -> Result<ToolLane<'_>, Error> {
        if !(self.temperature.0..=self.temperature.1).contains(&simulated_temperature)
            || !parked_e.is_finite()
        {
            return Err(Error::Temperature(self.id));
        }
        Ok(ToolLane {
            id: self.id,
            parked_e,
            extruder: SimulatedExtruder {
                geometry: &self.geometry,
                temperature: simulated_temperature,
                step_distance: self.step_distance,
                inverse_direction: self.direction,
                pressure_advance: self.advance.as_ref(),
            },
        })
    }
}

/// Bind independently stepped extruders to physical tool numbers declared
/// in a single root toolchanger, or use contiguous stock indices without
/// a toolchanger. Nonzero nozzle offsets and per-tool input shaping are
/// rejected because this preview doesn't apply those configuration changes.
pub fn read(cfg: &Config) -> Result<Vec<Profile>, Error> {
    if cfg
        .sections()
        .any(|s| s.starts_with("extruder_stepper ") || s.starts_with("toolchanger "))
    {
        return Err(Error::Unsupported);
    }
    let mut extruders = cfg
        .sections()
        .filter_map(|s| {
            let index = s.strip_prefix("extruder")?;
            let id = if index.is_empty() {
                0
            } else if index.starts_with('0') || !index.bytes().all(|v| v.is_ascii_digit()) {
                return None;
            } else {
                index.parse::<u32>().unwrap_or(u32::MAX)
            };
            Some((id, s))
        })
        .collect::<Vec<_>>();
    extruders.sort_unstable_by_key(|p| p.0);
    if extruders.is_empty() {
        return Err(Error::MissingExtruder);
    }
    if extruders.len() > MAX_PROFILES {
        return Err(Error::Unsupported);
    }
    if extruders
        .iter()
        .enumerate()
        .any(|(n, (i, _))| n as u32 != *i)
    {
        return Err(Error::ExtruderGap);
    }
    if cfg.values("toolchanger").is_none() {
        if cfg.sections().any(|s| s.starts_with("tool ")) {
            return Err(Error::ToolCatalog);
        }
        return extruders
            .into_iter()
            .map(|(id, section)| parse(cfg, id as u8, section, None))
            .collect();
    }
    let catalog = Catalog::from_config(cfg).map_err(|_| Error::ToolCatalog)?;
    if catalog.tools().len() > MAX_PROFILES {
        return Err(Error::Unsupported);
    }
    let global_shaper = ["shaper_freq_x", "shaper_freq_y", "shaper_freq_z"]
        .iter()
        .any(|key| {
            cfg.get("input_shaper", key)
                .is_some_and(|v| v.parse::<f64>().ok() != Some(0.))
        });
    let mut names = Vec::new();
    let mut profiles = Vec::new();
    for tool in catalog.tools() {
        if tool.nozzle_offset != [0.; 3]
            || [
                "params_input_shaper_freq_x",
                "params_input_shaper_freq_y",
                "params_input_shaper_freq_z",
            ]
            .iter()
            .any(|key| {
                cfg.get(&tool.section, key)
                    .or_else(|| cfg.get("toolchanger", key))
                    .is_some_and(|x| global_shaper || x.parse::<f64>().ok() != Some(0.))
            })
        {
            return Err(Error::Unsupported);
        }
        let section = cfg
            .get(&tool.section, "extruder")
            .ok_or(Error::ToolCatalog)?;
        if !extruders.iter().any(|(_, name)| *name == section) {
            return Err(Error::ToolCatalog);
        }
        if names.contains(&section) {
            return Err(Error::SharedExtruder);
        }
        names.push(section);
        profiles.push(parse(cfg, tool.number, section, Some(&tool.section))?);
    }
    Ok(profiles)
}

fn parse(cfg: &Config, id: u8, section: &str, tool: Option<&str>) -> Result<Profile, Error> {
    let field = |section: &str, key: &'static str, default: Option<f64>| {
        cfg.get(section, key)
            .map_or(default, |s| s.parse().ok())
            .filter(|v| v.is_finite())
            .ok_or_else(|| Error::Field(section.to_owned(), key))
    };
    let get = |key, default| field(section, key, default);
    let integer = |key, default| {
        cfg.get(section, key)
            .map_or(default, |s| s.parse::<u32>().ok())
            .filter(|v| *v > 0)
            .ok_or_else(|| Error::Field(section.to_owned(), key))
    };
    let nozzle = get("nozzle_diameter", None)?;
    let filament = get("filament_diameter", None)?;
    let area = 4. * nozzle.powi(2);
    let default_ratio = area / (std::f64::consts::PI * (filament * 0.5).powi(2));
    let minimum = get("min_temp", None)?;
    let maximum = get("max_temp", None)?;
    let min_extrude = get("min_extrude_temp", Some(170.))?;
    if minimum < -273.15 || minimum >= maximum || !(minimum..=maximum).contains(&min_extrude) {
        return Err(Error::Field(section.into(), "heater bounds"));
    }
    let geometry = Extruder::new(Settings {
        nozzle,
        filament,
        max_area: get("max_extrude_cross_section", Some(area))?,
        retraction_distance: get("max_extrude_only_distance", Some(50.))?,
        retraction_velocity: get(
            "max_extrude_only_velocity",
            Some(field("printer", "max_velocity", None)? * default_ratio),
        )?,
        retraction_accel: get(
            "max_extrude_only_accel",
            Some(field("printer", "max_accel", None)? * default_ratio),
        )?,
        corner_velocity: get("instantaneous_corner_velocity", Some(1.))?,
        min_simulated_temperature: min_extrude,
    })
    .map_err(|_| Error::Field(section.into(), "extrusion geometry"))?;
    let rotation = get("rotation_distance", None)?;
    let micro = integer("microsteps", None)?;
    let full = integer("full_steps_per_rotation", Some(200))?;
    if full % 4 != 0 {
        return Err(Error::Field(section.into(), "full_steps_per_rotation"));
    }
    let gear = gearing(cfg.get(section, "gear_ratio"))
        .ok_or_else(|| Error::Field(section.into(), "gear_ratio"))?;
    // Stock multiplies integer full steps by microsteps before applying gearing.
    let step_distance = rotation / ((u64::from(full) * u64::from(micro)) as f64 * gear);
    if !(1e-9..=1000.).contains(&step_distance) {
        return Err(Error::Field(section.into(), "rotation_distance"));
    }
    if cfg.get(section, "step_pin").is_none_or(str::is_empty) {
        return Err(Error::Field(section.into(), "step_pin"));
    }
    let dir = cfg
        .get(section, "dir_pin")
        .filter(|v| {
            let pin = v.strip_prefix('!').unwrap_or(v);
            !pin.is_empty()
                && !pin.starts_with(['!', '^', '~'])
                && !pin.contains(char::is_whitespace)
        })
        .ok_or_else(|| Error::Field(section.into(), "dir_pin"))?;
    let override_number = |key: &'static str, default: f64| {
        tool.and_then(|section| {
            cfg.get(section, key)
                .map(|v| (section, v))
                .or_else(|| cfg.get("toolchanger", key).map(|v| ("toolchanger", v)))
        })
        .map_or(Ok(default), |(section, value)| {
            value
                .parse::<f64>()
                .ok()
                .filter(|v| v.is_finite())
                .ok_or_else(|| Error::Field(section.into(), key))
        })
    };
    let advance = override_number(
        "params_pressure_advance",
        get("pressure_advance", Some(0.))?,
    )?;
    let smooth = override_number(
        "params_pressure_advance_smooth_time",
        get("pressure_advance_smooth_time", Some(0.04))?,
    )?;
    if !((0.0..=0.200).contains(&smooth)) || advance < 0. {
        return Err(Error::Field(section.into(), "pressure advance"));
    }
    let advance = if advance == 0. || smooth == 0. {
        None
    } else {
        Some(
            Advance::new(smooth, &[(0., advance)])
                .map_err(|_| Error::Field(section.into(), "pressure advance"))?,
        )
    };
    Ok(Profile {
        id,
        section: section.into(),
        step_distance,
        geometry,
        temperature: (minimum, maximum),
        direction: dir.starts_with('!'),
        advance,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn fixture(f: impl FnOnce(&str) -> String) -> Config {
        let source = include_str!("../../examples/preview/sparse-tool.cfg");
        crate::config::parse(
            &BTreeMap::from([
                ("sparse.cfg".into(), f(source)),
                (
                    "printer.cfg".into(),
                    include_str!("../../examples/preview/printer.cfg").into(),
                ),
                (
                    "rails.cfg".into(),
                    include_str!("../../examples/preview/rails.cfg").into(),
                ),
            ]),
            "sparse.cfg",
        )
        .unwrap()
    }

    #[test]
    fn rejects_index_gaps_shared_extruders_and_unsupported_transforms() {
        let changed = fixture(|s| s.replace("extruder1", "extruder3"));
        assert!(matches!(read(&changed), Err(Error::ExtruderGap)));
        let changed = fixture(|s| s.replace("extruder: extruder2", "extruder: extruder1"));
        assert!(matches!(read(&changed), Err(Error::SharedExtruder)));
        let changed = fixture(|s| {
            s.replace(
                "extruder: extruder2",
                "extruder: extruder2\ngcode_z_offset: 0.1",
            )
        });
        assert!(matches!(read(&changed), Err(Error::Unsupported)));
        let changed = fixture(|s| {
            s.replace(
                "extruder: extruder2",
                "extruder: extruder2\nparams_input_shaper_freq_x: 0",
            )
        });
        assert!(matches!(read(&changed), Err(Error::Unsupported)));
        let changed = fixture(|s| s.replace("max_temp: 300", "max_temp: 160"));
        assert!(matches!(
            read(&changed),
            Err(Error::Field(_, "heater bounds"))
        ));
    }
}
