// SPDX-License-Identifier: GPL-3.0-only
//! Bounded, inert file-debug prediction with X/Y shaper updates.
use super::{
    render::{replay_error, shaped_plan, solve_e, ShapeCaptureSchedule},
    MotionTrace, Options, OutputOptions, TraceError,
};
use crate::{
    config,
    extrusion::Extruder,
    input_shaper::{captured, Shaper},
    step_times::Axis,
};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Error {
    Input,
    Limit,
}

/// Resolved keys retained across commands; storing the state is necessary
/// because a generated pulse schedule cannot recover its source settings.
struct ShapeSettings {
    values: BTreeMap<String, String>,
}

const NAMES: [&str; 7] = [
    "shaper_type",
    "shaper_type_x",
    "shaper_type_y",
    "shaper_freq_x",
    "shaper_freq_y",
    "damping_ratio_x",
    "damping_ratio_y",
];
const MAX_UPDATES: usize = 32;

impl ShapeSettings {
    fn from_config(config: &config::Config) -> Result<Self, Error> {
        // Reuse the fixed preview guard for starting configuration.
        Shaper::from_file_debug_config(config).map_err(|_| Error::Input)?;
        let values = config.values("input_shaper").ok_or(Error::Input)?.clone();
        Ok(Self { values })
    }

    fn shape(&self) -> Result<Shaper, Error> {
        let mut text = "[input_shaper]\n".to_string();
        for (key, value) in &self.values {
            use std::fmt::Write;
            writeln!(text, "{key}:{value}").map_err(|_| Error::Input)?;
        }
        let cfg = config::parse(&BTreeMap::from([("shape.cfg".into(), text)]), "shape.cfg")
            .map_err(|_| Error::Input)?;
        crate::input_shaper_defs::from_config(&cfg)
            .map(|s| s.unwrap_or_default())
            .map_err(|_| Error::Input)
    }

    /// Bare status queries return None and do not flush or modify the state.
    fn apply<'a>(
        &mut self,
        fields: impl Iterator<Item = &'a str>,
    ) -> Result<Option<Shaper>, Error> {
        let mut updates = BTreeMap::new();
        for f in fields {
            let (k, v) = f.split_once('=').ok_or(Error::Input)?;
            let key = k.to_ascii_lowercase();
            if !NAMES.contains(&key.as_str()) || v.is_empty() || v.len() > 32 {
                return Err(Error::Input);
            }
            let value = if key.starts_with("shaper_type") {
                let value = v.to_ascii_lowercase();
                if !["zv", "zvd", "mzv", "ei", "2hump_ei", "3hump_ei"].contains(&value.as_str()) {
                    return Err(Error::Input);
                }
                value
            } else {
                let parsed = v.parse::<f64>().map_err(|_| Error::Input)?;
                if !parsed.is_finite() || parsed < 0. {
                    return Err(Error::Input);
                }
                v.to_owned()
            };
            if updates.insert(key, value).is_some() {
                return Err(Error::Input);
            }
        }
        if updates.is_empty() {
            return Ok(None);
        }
        let mut values = self.values.clone();
        let global = updates.remove("shaper_type");
        values.extend(updates);
        // Pinned stock gives the global argument priority over axis keys.
        if let Some(global) = global {
            values.insert("shaper_type_x".into(), global.clone());
            values.insert("shaper_type_y".into(), global);
        }
        let next = Self { values };
        let shaper = next.shape()?;
        *self = next;
        Ok(Some(shaper))
    }
}

fn collect_updates(body: &str, mut settings: ShapeSettings) -> Result<Vec<(usize, Shaper)>, Error> {
    if body.len() > 65_536 {
        return Err(Error::Limit);
    }
    let mut changes = Vec::new();
    for (line, text) in body.lines().enumerate() {
        let mut fields = text
            .split(';')
            .next()
            .ok_or(Error::Input)?
            .split_ascii_whitespace();
        if fields
            .next()
            .is_some_and(|v| v.eq_ignore_ascii_case("SET_INPUT_SHAPER"))
        {
            if let Some(next) = settings.apply(fields)? {
                if changes.len() == MAX_UPDATES {
                    return Err(Error::Limit);
                }
                changes.push((line, next));
            }
        }
    }
    Ok(changes)
}

macro_rules! trace_runtime {
    ($file:ident,$start:ident,$options:ident,$geometries:ident,$wire:ident,$initial:ident,$mesh:expr) => {{
        let shaper = Shaper::from_file_debug_config($initial).map_err(|_| TraceError::Options)?;
        let (_, body) = $file.split_once('\n').ok_or(TraceError::Options)?;
        let settings = ShapeSettings::from_config($initial).map_err(|_| TraceError::Options)?;
        let shapes = collect_updates(body, settings).map_err(|_| TraceError::Options)?;
        let mut delays = Vec::with_capacity(shapes.len());
        let mut updates = Vec::with_capacity(shapes.len());
        for (line, shape) in shapes {
            let (pre, post) = shape.window(Axis::CoreXYPlus);
            delays.push((line, pre.max(post)));
            updates.push(shape);
        }
        let mut positions = Vec::with_capacity(delays.len());
        let (events, captures, lanes) = shaped_plan(
            $file,
            $start,
            $options,
            $geometries,
            $wire,
            &shaper,
            Some(ShapeCaptureSchedule {
                #[cfg(feature = "runtime-mesh-preview")]
                mesh: $mesh,
                delays: &delays,
                positions: &mut positions,
            }),
        )?;
        let [a, b, z] = captured::replay(
            &shaper,
            &captures,
            lanes,
            $wire.clock_hz,
            $wire.max_error_ticks,
            Some(captured::ReplayUpdates {
                positions: &positions,
                shapes: &updates,
            }),
        )
        .map_err(replay_error)?;
        let e = solve_e(&events, $options.smooth, $wire)?;
        Ok(MotionTrace {
            lanes: [a, b, z, e],
        })
    }};
}

/// Predict bounded file-debug steps including supported X/Y shaper changes.
/// Only the offline time model and ordered JSON records are affected.
pub fn trace_file_debug_runtime_shaper(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
    initial: &config::Config,
) -> Result<MotionTrace, TraceError> {
    trace_runtime!(file, start, options, geometries, wire, initial, None)
}

/// Same offline schedule with one validated selectable bed mesh.
#[cfg(feature = "runtime-mesh-preview")]
pub fn trace_file_debug_runtime_mesh(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
    initial: &config::Config,
    mesh: (&str, &crate::bed_mesh::Transform),
) -> Result<MotionTrace, TraceError> {
    trace_runtime!(file, start, options, geometries, wire, initial, Some(mesh))
}

#[cfg(feature = "runtime-shaper-workers")]
pub(super) struct PreparedRuntime {
    pub(super) events: Vec<super::Event>,
    pub(super) captures: Vec<captured::Capture>,
    pub(super) lanes: [(Axis, f64, bool); 3],
    pub(super) shaper: Shaper,
    pub(super) positions: Vec<(usize, usize)>,
    pub(super) updates: Vec<Shaper>,
}

#[cfg(feature = "runtime-shaper-workers")]
#[inline(always)]
pub(super) fn prepare_runtime(
    file: &str,
    start: [f64; 3],
    options: Options,
    geometries: [&Extruder; 2],
    wire: OutputOptions,
    initial: &config::Config,
) -> Result<PreparedRuntime, TraceError> {
    let shaper = Shaper::from_file_debug_config(initial).map_err(|_| TraceError::Options)?;
    let (_, body) = file.split_once('\n').ok_or(TraceError::Options)?;
    let settings = ShapeSettings::from_config(initial).map_err(|_| TraceError::Options)?;
    let shapes = collect_updates(body, settings).map_err(|_| TraceError::Options)?;
    let mut delays = Vec::with_capacity(shapes.len());
    let mut updates = Vec::with_capacity(shapes.len());
    for (line, shape) in shapes {
        let (pre, post) = shape.window(Axis::CoreXYPlus);
        delays.push((line, pre.max(post)));
        updates.push(shape);
    }
    let mut positions = Vec::with_capacity(delays.len());
    let (events, captures, lanes) = shaped_plan(
        file,
        start,
        options,
        geometries,
        wire,
        &shaper,
        Some(ShapeCaptureSchedule {
            #[cfg(feature = "runtime-mesh-preview")]
            mesh: None,
            delays: &delays,
            positions: &mut positions,
        }),
    )?;
    Ok(PreparedRuntime {
        events,
        captures,
        lanes,
        shaper,
        positions,
        updates,
    })
}
