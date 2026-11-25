// SPDX-License-Identifier: GPL-3.0-only
//! A bounded, read-only bank for selectable offline saved meshes.

use super::{profile, Transform};
use crate::config;
use std::collections::{BTreeMap, BTreeSet};

const MAX_BANK_BYTES: usize = 8 * 1024 * 1024;
const HEADER: &str = "#*# <---------------------- SAVE_CONFIG ---------------------->";
const WARNING: &str = "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.";

#[derive(Debug)]
pub enum Error {
    Names,
    Sections,
    Config(config::Error),
    Profile(String, profile::Error),
}

pub struct Bank(BTreeMap<String, Transform>);

impl Bank {
    /// Read exactly the requested profiles and one explicit [bed_mesh] section.
    /// No unrelated settings, includes, duplicated sections or autosave without
    /// the stock header. Individual transforms use the one-profile validator.
    pub fn load(entry: &str, names: &[&str]) -> Result<Self, Error> {
        if !(2..=8).contains(&names.len())
            || names.iter().any(|name| {
                !(1..=64).contains(&name.len())
                    || !name
                        .bytes()
                        .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-'))
            })
        {
            return Err(Error::Names);
        }
        let sections: BTreeSet<_> = names.iter().map(|n| format!("bed_mesh {n}")).collect();
        if sections.len() != names.len() {
            return Err(Error::Names);
        }

        if entry.len() > MAX_BANK_BYTES {
            return Err(Error::Sections);
        }
        let mut seen = BTreeSet::new();
        let mut saved = false;
        let mut lines = entry.trim_end().lines();
        while let Some(raw) = lines.next() {
            if !saved && raw == HEADER {
                if lines.next() != Some(WARNING) || lines.next() != Some("#*#") {
                    return Err(Error::Sections);
                }
                saved = true;
                continue;
            }
            let line = if saved {
                if raw == "#*#" {
                    continue;
                }
                raw.strip_prefix("#*# ").ok_or(Error::Sections)?
            } else {
                // Disallow ambiguous/misplaced SAVE_CONFIG-like prefixes.
                if raw.trim_start().starts_with("#*#") {
                    return Err(Error::Sections);
                }
                raw
            };
            let line = line.split(['#', ';']).next().unwrap_or_default().trim();
            if let Some(header) = line.strip_prefix('[') {
                let name = header.strip_suffix(']').ok_or(Error::Sections)?;
                if ((name != "bed_mesh" || saved) && !sections.contains(name)) || !seen.insert(name)
                {
                    return Err(Error::Sections);
                }
            }
        }
        if seen.len() != sections.len() + 1 {
            return Err(Error::Sections);
        }
        let config = config::parse(
            &BTreeMap::from([("mesh.cfg".into(), entry.into())]),
            "mesh.cfg",
        )
        .map_err(Error::Config)?;
        let mut bank = BTreeMap::new();
        for &name in names {
            let transform =
                profile::load(&config, entry, name).map_err(|e| Error::Profile(name.into(), e))?;
            bank.insert(name.into(), transform);
        }
        Ok(Self(bank))
    }

    pub(crate) fn get(&self, name: &str) -> Option<&Transform> {
        self.0.get(name)
    }

    // The existing non-LOAD path ignores the profile pair.
    pub(crate) fn first(&self) -> (&str, &Transform) {
        let (name, transform) = self.0.first_key_value().expect("validated nonempty bank");
        (name, transform)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_eight_explicit_profiles_and_preserves_case() {
        let source = include_str!("../../examples/preview/mesh.cfg");
        let source = source.split_once("\n").unwrap().1; // Remove the fixture include.
        let (base, data) = source.split_once("[bed_mesh test_surface]").unwrap();
        let names = [
            "A",
            "a",
            "north",
            "south",
            "east",
            "west",
            "left-edge",
            "right_edge",
        ];
        let mut config = base.to_owned();
        for name in names {
            config.push_str(&format!("\n[bed_mesh {name}]{data}\n"));
        }
        let bank = Bank::load(&config, &names).unwrap();
        assert_eq!(bank.0.len(), 8);
        assert!(bank.get("A").is_some());
        assert!(bank.get("a").is_some());
        assert!(bank.get("West").is_none());
    }
}
