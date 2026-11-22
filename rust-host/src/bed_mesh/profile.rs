// SPDX-License-Identifier: GPL-3.0-only
//! Read one bounded saved bed mesh for use in the offline motion preview.
//! Adapted from Klipper's extras/bed_mesh.py ProfileManager format.
//! This never probes a printer or writes its saved configuration.

use super::{Algorithm, Mesh, Transform};
use crate::config::{self, Config};
use std::collections::BTreeMap;

const HEADER: [&str; 3] = [
    "#*# <---------------------- SAVE_CONFIG ---------------------->",
    "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.",
    "#*#",
];
const MAX_SAVED_BYTES: usize = 128 * 1024;

#[derive(Debug)]
pub enum Error {
    Name,
    Missing,
    Duplicate,
    SavedBlock,
    Config(config::Error),
    Field(&'static str),
    Geometry(super::Error),
}

/// Accept an ordinary profile section or the selected section in the stock
/// SAVE_CONFIG block in the entry file. Other autosaved settings are ignored.
pub fn load(config: &Config, entry: &str, profile: &str) -> Result<Transform, Error> {
    if !(1..=64).contains(&profile.len())
        || !profile
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-'))
    {
        return Err(Error::Name);
    }
    let section = format!("bed_mesh {profile}");
    if config.values("bed_mesh").is_none() {
        return Err(Error::Missing);
    }
    let explicit = config.values(&section).is_some();
    let saved = saved_section(entry, &section)?;
    let secondary;
    let source = match (explicit, saved) {
        (true, Some(_)) => return Err(Error::Duplicate),
        (false, None) => return Err(Error::Missing),
        (true, None) => config,
        (false, Some(text)) => {
            secondary = config::parse(
                &BTreeMap::from([("profile.cfg".into(), text)]),
                "profile.cfg",
            )
            .map_err(Error::Config)?;
            &secondary
        }
    };
    let numeric = |parent: &Config, name: &str, key, default| {
        let value = parent.get(name, key);
        value
            .map_or(default, |s| s.parse::<f64>().ok())
            .filter(|v| v.is_finite())
            .ok_or(Error::Field(key))
    };
    let get = |key| numeric(source, &section, key, None);
    let usize_at = |key| {
        source
            .get(&section, key)
            .and_then(|s| s.parse::<usize>().ok())
            .ok_or(Error::Field(key))
    };
    if usize_at("version")? != 1 {
        return Err(Error::Field("version"));
    }
    let size = [usize_at("x_count")?, usize_at("y_count")?];
    if !size.iter().all(|x| (2..=128).contains(x)) {
        return Err(Error::Field("counts"));
    }
    let pps = [usize_at("mesh_x_pps")?, usize_at("mesh_y_pps")?];
    let tension = get("tension")?;
    let algo = match source.get(&section, "algo") {
        Some("direct") if pps == [0; 2] => Algorithm::Direct,
        Some("lagrange") => Algorithm::Lagrange {
            points_per_segment: pps,
        },
        Some("bicubic") => Algorithm::Bicubic {
            points_per_segment: pps,
            tension,
        },
        _ => return Err(Error::Field("algo")),
    };
    let mut points = Vec::new();
    let flat = source
        .get(&section, "points")
        .ok_or(Error::Field("points"))?;
    for (i, line) in flat
        .lines()
        .filter(|line| !line.trim().is_empty())
        .enumerate()
    {
        if i >= size[1] {
            return Err(Error::Field("points"));
        }
        let mut row = 0;
        for text in line.split(',') {
            row += 1;
            if row > size[0] {
                return Err(Error::Field("points"));
            }
            let value = text
                .trim()
                .parse::<f64>()
                .ok()
                .filter(|v| v.is_finite())
                .ok_or(Error::Field("points"))?;
            points.push(value);
        }
        if row != size[0] {
            return Err(Error::Field("points"));
        }
    }
    let bounds = [
        [get("min_x")?, get("min_y")?],
        [get("max_x")?, get("max_y")?],
    ];
    let mesh = Mesh::new(bounds, size, &points, algo).map_err(Error::Geometry)?;
    let base = |key, default| numeric(config, "bed_mesh", key, Some(default));
    let start = base("fade_start", 1.)?;
    let end = base("fade_end", 0.)?;
    let target = config
        .get("bed_mesh", "fade_target")
        .map(|_| numeric(config, "bed_mesh", "fade_target", None))
        .transpose()?;
    let fade = if end > start {
        Some((start, end, target.unwrap_or_else(|| mesh.average())))
    } else {
        None
    };
    Transform::from_parts(
        mesh,
        fade,
        target.is_some(),
        0.,
        base("split_delta_z", 0.025)?,
        base("move_check_distance", 5.)?,
    )
    .map_err(Error::Geometry)
}

fn saved_section(entry: &str, requested: &str) -> Result<Option<String>, Error> {
    let mut lines = entry.trim_end().lines();
    let Some(_) = lines.find(|line| *line == HEADER[0]) else {
        return Ok(None);
    };
    if lines.next() != Some(HEADER[1]) || lines.next() != Some(HEADER[2]) {
        return Err(Error::SavedBlock);
    }
    let mut capturing = false;
    let mut output = None::<String>;
    for raw in lines {
        let line = if raw == "#*#" {
            ""
        } else {
            raw.strip_prefix("#*# ").ok_or(Error::SavedBlock)?
        };
        if line == HEADER[0] {
            return Err(Error::SavedBlock);
        }
        if let Some(header) = line.trim().strip_prefix('[') {
            let key = header.strip_suffix(']').ok_or(Error::SavedBlock)?;
            capturing = key == requested;
            if capturing {
                if output.is_some() {
                    return Err(Error::Duplicate);
                }
                output = Some(String::new());
            }
        }
        if capturing {
            let txt = output.as_mut().expect("started saved section");
            if txt.len().saturating_add(line.len()) >= MAX_SAVED_BYTES {
                return Err(Error::SavedBlock);
            }
            txt.push_str(line);
            txt.push('\n');
        }
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(entry: &str) -> Result<Transform, Error> {
        let cfg = config::parse(
            &BTreeMap::from([
                ("mesh.cfg".into(), entry.into()),
                (
                    "printer.cfg".into(),
                    include_str!("../../examples/preview/printer.cfg").into(),
                ),
                (
                    "rails.cfg".into(),
                    include_str!("../../examples/preview/rails.cfg").into(),
                ),
            ]),
            "mesh.cfg",
        )
        .unwrap();
        load(&cfg, entry, "test_surface")
    }

    #[test]
    fn validates_saved_shape_and_refuses_duplicate_or_corrupt_profiles() {
        let inline = include_str!("../../examples/preview/mesh.cfg");
        let saved = include_str!("../../examples/preview/mesh-saved.cfg");
        let explicit = read(inline).unwrap();
        let autosave = read(saved).unwrap();
        let from = [1.0, 1.0, 0.3, 0.];
        let to = [33.0, 32.0, 1.0, 0.];
        assert_eq!(
            explicit.split(from, to).unwrap(),
            autosave.split(from, to).unwrap()
        );
        assert!(matches!(
            read(&inline.replace("version: 1", "version: 8")),
            Err(Error::Field("version"))
        ));
        assert!(matches!(
            read(&inline.replace("x_count: 3", "x_count: 4")),
            Err(Error::Field("points"))
        ));
        let header = saved.find(HEADER[0]).unwrap();
        assert!(matches!(
            read(&format!("{inline}\n{}", &saved[header..])),
            Err(Error::Duplicate)
        ));
        assert!(matches!(
            read(&format!("{saved}\n[untrusted]")),
            Err(Error::SavedBlock)
        ));
    }
}
