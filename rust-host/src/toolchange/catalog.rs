// SPDX-License-Identifier: GPL-3.0-only
//! Read-only metadata for the numbered tools in one root toolchanger.
//! Adapted from viesturz/klipper-toolchanger extras/tool.py and toolchanger.py.
//! Upstream copyright (C) 2023 Viesturs Zarins.

use super::{Changer, Error, Policy, MAX_TOOLS};
use crate::config::Config;

const ROOT: &str = "toolchanger";

#[derive(Debug, Clone, PartialEq)]
pub struct Tool {
    pub number: u8,
    pub section: String,
    pub nozzle_offset: [f64; 3],
    pub restore_axes: u8,
    pub has_detection: bool,
}

pub struct Catalog {
    tools: Vec<Tool>,
    policy: Policy,
}

impl Catalog {
    /// Parse the numbered tools only. This never loads extruder, heater,
    /// probe, fan or pin objects. Multi-toolchanger and dynamic assignment
    /// need a wider runtime than this static metadata view.
    pub fn from_config(config: &Config) -> Result<Self, Error> {
        if config.values(ROOT).is_none() {
            return Err(Error::Config);
        }
        let used = axes(config.get(ROOT, "uses_axis").unwrap_or("xyz"))?;
        if used == 0 {
            return Err(Error::Config);
        }
        let verify = boolean(config.get(ROOT, "verify_tool_pickup").unwrap_or("true"))?;
        let require = boolean(config.get(ROOT, "require_tool_present").unwrap_or("false"))?;
        let mut tools = Vec::new();
        for section in config.sections().filter(|s| s.starts_with("tool ")) {
            let Some(number) = config.get(section, "tool_number") else {
                // The plugin allows unnamed numerical tools to be assigned later.
                continue;
            };
            if tools.len() == MAX_TOOLS
                || section.len() > 128
                || config
                    .get(section, "toolchanger")
                    .is_some_and(|c| c != ROOT)
            {
                return Err(Error::Config);
            }
            let number = number.parse::<u8>().map_err(|_| Error::Config)?;
            let inherit = |key| config.get(section, key).or_else(|| config.get(ROOT, key));
            let mut nozzle_offset = [0.0; 3];
            for (v, key) in
                nozzle_offset
                    .iter_mut()
                    .zip(["gcode_x_offset", "gcode_y_offset", "gcode_z_offset"])
            {
                if let Some(txt) = inherit(key) {
                    *v = txt
                        .parse::<f64>()
                        .ok()
                        .filter(|f| f.is_finite())
                        .ok_or(Error::Config)?;
                }
            }
            tools.push(Tool {
                section: section.to_owned(),
                number,
                nozzle_offset,
                restore_axes: axes(inherit("t_command_restore_axis").unwrap_or("XYZ"))?,
                has_detection: config
                    .get(section, "detection_pin")
                    .is_some_and(|x| !x.is_empty()),
            });
        }
        let any_detection = tools.iter().any(|t| t.has_detection);
        if tools.is_empty() || (any_detection && tools.iter().any(|t| !t.has_detection)) {
            return Err(Error::Config);
        }
        tools.sort_unstable_by_key(|t| t.number);
        if tools
            .windows(2)
            .any(|pair| pair[0].number == pair[1].number)
        {
            return Err(Error::Config);
        }
        let policy = Policy {
            axes: used,
            allow_empty: !require,
            verify_pickup: verify && any_detection,
        };
        Ok(Self { tools, policy })
    }

    pub fn tools(&self) -> &[Tool] {
        &self.tools
    }

    pub fn policy(&self) -> Policy {
        self.policy
    }

    pub fn numbered(&self, number: u8) -> Option<&Tool> {
        self.tools
            .binary_search_by_key(&number, |t| t.number)
            .ok()
            .map(|i| &self.tools[i])
    }

    pub fn build(&self) -> Result<Changer, Error> {
        Changer::new(self.tools.iter().map(|t| t.number).collect(), self.policy)
    }
}

fn axes(text: &str) -> Result<u8, Error> {
    text.bytes().try_fold(0, |mask, c| {
        Ok(mask
            | match c.to_ascii_lowercase() {
                b'x' => 1,
                b'y' => 2,
                b'z' => 4,
                _ => return Err(Error::Config),
            })
    })
}

fn boolean(raw: &str) -> Result<bool, Error> {
    match raw.to_ascii_lowercase().as_str() {
        "yes" | "true" | "1" | "on" => Ok(true),
        "no" | "false" | "0" | "off" => Ok(false),
        _ => Err(Error::Config),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[test]
    fn preserves_root_defaults_and_uses_real_detection_only_for_all_tools() {
        let base = "[toolchanger]\nuses_axis: xY\ngcode_y_offset: .5\nt_command_restore_axis: Xz\nrequire_tool_present: true\n";
        let tools = "[tool T2]\ntool_number: 2\ngcode_x_offset: -.33\ngcode_y_offset: 0\ndetection_pin: ignored_a\n[tool T0]\ntool_number: 0\ndetection_pin: ignored_b\n";
        let parse = |s: String| {
            crate::config::parse(&BTreeMap::from([("test.cfg".into(), s)]), "test.cfg").unwrap()
        };
        let catalog = Catalog::from_config(&parse(format!("{base}{tools}"))).unwrap();
        assert_eq!(
            catalog.tools().iter().map(|t| t.number).collect::<Vec<_>>(),
            [0, 2]
        );
        assert_eq!(catalog.numbered(0).unwrap().nozzle_offset, [0., 0.5, 0.]);
        assert_eq!(catalog.numbered(2).unwrap().nozzle_offset, [-0.33, 0., 0.]);
        assert_eq!(catalog.numbered(2).unwrap().restore_axes, 5);
        assert!(catalog.policy().verify_pickup);
        assert!(!catalog.policy().allow_empty);
        assert_eq!(catalog.policy().axes, 3);
        let mixed = tools.replace("detection_pin: ignored_b", "");
        assert!(matches!(
            Catalog::from_config(&parse(format!("{base}{mixed}"))),
            Err(Error::Config)
        ));
    }
}
