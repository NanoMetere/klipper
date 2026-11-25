// SPDX-License-Identifier: GPL-3.0-only
//! Prototype: independent planning for a bounded pin-free stock debug slice.
//! It requires a caller-chosen virtual origin and does not model live pacing.

#[cfg(feature = "file-debug-preview")]
mod cadence;
mod render;
#[cfg(feature = "runtime-shaper-preview")]
mod runtime_shaper;
#[cfg(feature = "shaped-file-debug-preview")]
pub use render::trace_file_debug_shaped;
pub use render::{trace, MotionTrace, OutputOptions, TraceError};
#[cfg(feature = "file-debug-preview")]
pub use render::{trace_file_debug, trace_file_debug_parallel};
#[cfg(feature = "runtime-shaper-preview")]
pub use runtime_shaper::trace_file_debug_runtime_shaper;
#[cfg(feature = "shaped-file-debug-preview")]
mod shaped_workers;
#[cfg(feature = "shaped-file-debug-preview")]
pub use shaped_workers::trace_file_debug_shaped_parallel;

use crate::{
    extrusion::Extruder,
    gcode::{self, Preview},
    lookahead::{Limits, Profile},
    toolchange::ExtruderPositions,
};

#[derive(Debug, Clone, Copy)]
pub enum Event {
    Profile {
        at: f64,
        queue: u8,
        movement: Profile,
    },
    Gain {
        at: f64,
        advance: f64,
    },
    Generation {
        at: f64,
        retirement: f64,
    },
    #[cfg(feature = "file-debug-preview")]
    DebugTransmit(f64),
    Bind {
        queue: Option<u8>,
        anchor: f64,
    },
}

#[derive(Debug)]
pub enum Error {
    Gcode(gcode::Error),
    Inputs,
}

impl From<gcode::Error> for Error {
    fn from(e: gcode::Error) -> Self {
        Self::Gcode(e)
    }
}

#[derive(Clone, Copy)]
pub struct Options {
    pub limits: Limits,
    pub z_velocity: f64,
    pub z_accel: f64,
    pub xyz_bounds: [[f64; 2]; 3],
    pub simulation_temp: [f64; 2],
    pub smooth: f64,
    pub initial_advance: f64,
}

// Runtime-only sideband: keep the stable motion events unchanged.
#[cfg(feature = "runtime-shaper-preview")]
pub(super) struct RuntimePlan<'a> {
    #[cfg(feature = "runtime-mesh-preview")]
    mesh: Option<(&'a str, &'a crate::bed_mesh::Transform)>,
    #[cfg(feature = "runtime-pickup-mesh-bank-preview")]
    mesh_bank: Option<&'a crate::bed_mesh::profile_bank::Bank>,
    #[cfg(feature = "runtime-pickup-pressure-preview")]
    dynamic_pressure: bool,
    #[cfg(feature = "runtime-pickup-extended-preview")]
    extended_pickup: bool,
    updates: &'a [(usize, f64)],
    positions: &'a mut Vec<(usize, usize)>,
    #[cfg(feature = "runtime-pickup-preview")]
    pressure: Option<&'a mut Vec<PressureBoundary>>,
}

#[cfg(feature = "runtime-pickup-preview")]
pub(super) struct PressureBoundary {
    before: usize,
    advance: f64,
    smooth: f64,
}

struct OriginClock {
    move_end: f64,
    needs_steps: f64,
    last_steps: f64,
    priming: bool,
    bound: Option<u8>,
    smoothing: f64,
    #[cfg(feature = "shaped-file-debug-preview")]
    shape_delay: f64,
    #[cfg(feature = "file-debug-preview")]
    wire: f64,
    #[cfg(feature = "file-debug-preview")]
    scheduled: bool,
    #[cfg(feature = "file-debug-preview")]
    batched: bool,
}

impl OriginClock {
    fn delay(&self) -> f64 {
        let delay = self.bound.map_or(0.001, |_| self.smoothing.max(0.001));
        #[cfg(feature = "shaped-file-debug-preview")]
        let delay = delay.max(self.shape_delay);
        delay
    }

    fn profile(
        &mut self,
        output: &mut Vec<Event>,
        queue: u8,
        profile: Profile,
        consumed: &mut usize,
        deferred: &mut Vec<(usize, f64)>,
    ) -> Result<(), Error> {
        if self.priming {
            self.resync_print_time();
            self.priming = false;
        }
        output.push(Event::Profile {
            at: self.move_end,
            queue,
            movement: profile,
        });
        let cruise = if (-1e-12..0.0).contains(&profile.cruise_t) {
            0.0
        } else {
            profile.cruise_t
        };
        self.move_end = self.move_end + profile.accel_t + cruise + profile.decel_t;
        if !self.move_end.is_finite() || self.move_end >= 120. {
            return Err(Error::Inputs);
        }
        self.needs_steps = self.needs_steps.max(self.move_end + self.delay());
        #[cfg(feature = "file-debug-preview")]
        if self.batched {
            self.scheduled = true;
        }
        *consumed += 1;
        while deferred.first().is_some_and(|(n, _)| *n == *consumed) {
            let (_, advance) = deferred.remove(0);
            output.push(Event::Gain {
                at: self.move_end,
                advance,
            });
        }
        Ok(())
    }

    fn resync_print_time(&mut self) {
        self.move_end = self
            .move_end
            .max(0.25_f64.max(0.1_f64.max(self.last_steps) + self.delay()));
    }

    fn dwell(&mut self, seconds: f64) -> Result<(), Error> {
        // Stock dwell re-primes the look-ahead and advances only print
        // time. A dwell itself does not place activity in a step queue.
        self.priming = true;
        self.resync_print_time();
        self.move_end += seconds;
        if !self.move_end.is_finite() || self.move_end >= 120. {
            return Err(Error::Inputs);
        }
        Ok(())
    }

    #[cfg(feature = "file-debug-preview")]
    fn advance_debug(&mut self, output: &mut Vec<Event>, wire: f64, gen: f64) {
        self.wire = self.wire.max(wire).max(gen - 0.050);
        self.last_steps = self.last_steps.max(gen).max(self.wire);
        output.push(Event::Generation {
            at: self.last_steps,
            retirement: self.last_steps - self.delay(),
        });
        output.push(Event::DebugTransmit(self.wire));
    }

    #[cfg(feature = "file-debug-preview")]
    fn debug_batch(
        &mut self,
        output: &mut Vec<Event>,
        mut before: impl FnMut(&mut Self, f64, &mut Vec<Event>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        if !self.scheduled {
            return Ok(());
        }
        let mut cadence = cadence::Cadence::new(self.needs_steps, self.last_steps, self.wire)
            .ok_or(Error::Inputs)?;
        for (wire, target) in cadence.by_ref() {
            // Timer callbacks can append pending lookahead before step
            // generation; the cadence's limit is the pre-callback snapshot.
            before(self, target, output)?;
            self.advance_debug(output, wire, target);
        }
        self.scheduled = cadence.more_armed();
        Ok(())
    }

    fn flush(&mut self, output: &mut Vec<Event>, tail: f64) {
        self.priming = true;
        #[cfg(feature = "file-debug-preview")]
        if self.batched {
            self.advance_debug(output, self.needs_steps + tail, 0.0);
            return;
        }
        self.last_steps = self.last_steps.max(self.needs_steps + tail);
        output.push(Event::Generation {
            at: self.last_steps,
            retirement: self.last_steps - self.delay(),
        });
    }
}

/// Plan a deliberately bounded, no-pacing stock-debug-style script.
/// The caller sets a virtual origin and supplies both logical extrusion
/// geometries. The only supported device-binding vocabulary is the named
/// shared stepper with extruder/extruder1, fixed positive smoothing and
/// positive gain updates when look-ahead holds a move. M400 closes look-ahead
/// without pacing; G4 P schedules a dwell in milliseconds. A final M400 is
/// mandatory and is followed by the file drain.
/// Background timer interleavings, homing, shapers and physical I/O are
/// not modeled. No input is ever sent to a printer.
pub fn plan(
    script: &str,
    start: [f64; 3],
    opt: Options,
    geometries: [&Extruder; 2],
) -> Result<Vec<Event>, Error> {
    plan_body(
        script,
        start,
        opt,
        geometries,
        0,
        #[cfg(feature = "shaped-file-debug-preview")]
        0.,
        #[cfg(feature = "runtime-shaper-preview")]
        None,
    )
}

#[cfg(feature = "file-debug-preview")]
macro_rules! file_debug_planner {
    ($name:ident $(, $shape_delay:ident $(, $shape_updates:ident)?)?) => {
        fn $name(
            file: &str,
            start: [f64; 3],
            opt: Options,
            geometries: [&Extruder; 2],
            $($shape_delay: f64, $($shape_updates: Option<RuntimePlan<'_>>,)?)?
        ) -> Result<Vec<Event>, Error> {
            #[cfg(feature = "runtime-pickup-extended-preview")]
            let extended_pickup = false $($(
                || $shape_updates.as_ref().is_some_and(|plan| plan.extended_pickup)
            )?)?;
            // Unlike the compact preview, the pinned debug reader batches the
            // original bytes, including comments. Limit this mode to ASCII LF.
            if file.len() > 65_536
                || !file.ends_with('\n')
                || file.lines().count() > {
                    #[cfg(feature = "runtime-pickup-extended-preview")]
                    if extended_pickup { 1024 } else { 512 }
                    #[cfg(not(feature = "runtime-pickup-extended-preview"))]
                    512
                }
                || file.lines().any(|line| line.len() >= 4096)
                || !file
                    .bytes()
                    .all(|b| matches!(b, b'\n' | b'\t' | 0x20..=0x7e))
            {
                return Err(Error::Inputs);
            }
            let (header, body) = file.split_once('\n').ok_or(Error::Inputs)?;
            let mut words = header
                .split(';')
                .next()
                .ok_or(Error::Inputs)?
                .split_ascii_whitespace();
            if words.next() != Some("SET_KINEMATIC_POSITION") {
                return Err(Error::Inputs);
            }
            for (axis, expected) in ['X', 'Y', 'Z'].into_iter().zip(start) {
                let value = words
                    .next()
                    .and_then(|w| w.strip_prefix(axis))
                    .and_then(|s| s.strip_prefix('='))
                    .ok_or(Error::Inputs)?;
                // Stock's extended parameter dispatch is separate from traditional
                // G-code parsing; use this explicit fixture header only as a check.
                let position: f64 = value.parse().map_err(|_| Error::Inputs)?;
                if !position.is_finite() || position != expected {
                    return Err(Error::Inputs);
                }
            }
            if words.next().is_some() {
                return Err(Error::Inputs);
            }
            #[cfg(feature = "shaped-file-debug-preview")]
            let delay = 0.0 $(+ $shape_delay)?;
            #[cfg(feature = "runtime-shaper-preview")]
            let updates = None $($(.or($shape_updates))?)?;
            plan_body(
                body, start, opt, geometries, header.len() + 1,
                #[cfg(feature = "shaped-file-debug-preview")]
                delay,
                #[cfg(feature = "runtime-shaper-preview")]
                updates,
            )
        }
    };
}
#[cfg(feature = "file-debug-preview")]
file_debug_planner!(plan_file_debug);
#[cfg(feature = "shaped-file-debug-preview")]
file_debug_planner!(plan_file_debug_with_delay, shape_delay);
#[cfg(feature = "runtime-shaper-preview")]
file_debug_planner!(plan_file_debug_with_updates, shape_delay, shape_updates);

#[cfg_attr(not(feature = "file-debug-preview"), inline(always))]
fn plan_body(
    script: &str,
    start: [f64; 3],
    opt: Options,
    geometries: [&Extruder; 2],
    #[cfg_attr(not(feature = "file-debug-preview"), allow(unused_variables))]
    debug_prefix_bytes: usize,
    #[cfg(feature = "shaped-file-debug-preview")] shape_delay: f64,
    #[cfg(feature = "runtime-shaper-preview")] mut shape_updates: Option<RuntimePlan<'_>>,
) -> Result<Vec<Event>, Error> {
    let inside = |coords: &[f64]| {
        opt.xyz_bounds.iter().zip(coords).all(|(b, v)| {
            b[0].is_finite() && b[1].is_finite() && b[0] < b[1] && (b[0]..=b[1]).contains(v)
        })
    };
    #[cfg(feature = "runtime-pickup-preview")]
    let runtime_pa = shape_updates
        .as_ref()
        .is_some_and(|plan| plan.pressure.is_some());
    #[cfg(feature = "runtime-pickup-pressure-preview")]
    let mut retuned = shape_updates
        .as_ref()
        .filter(|p| p.dynamic_pressure)
        .map(|_| (opt.initial_advance, opt.smooth));
    if script.len() > 65_536
        || !inside(&start)
        || !opt.simulation_temp.iter().all(|t| t.is_finite())
        || {
            #[cfg(feature = "runtime-pickup-preview")]
            {
                ![opt.z_velocity, opt.z_accel]
                    .iter()
                    .all(|v| v.is_finite() && *v > 0.)
                    || !opt.initial_advance.is_finite()
                    || opt.initial_advance < 0.
                    || (!runtime_pa && opt.initial_advance == 0.)
            }
            #[cfg(not(feature = "runtime-pickup-preview"))]
            {
                ![opt.z_velocity, opt.z_accel, opt.initial_advance]
                    .iter()
                    .all(|v| v.is_finite() && *v > 0.0)
            }
        }
        || !(1e-6..={
            #[cfg(feature = "runtime-pickup-pressure-preview")]
            if retuned.is_some() {
                0.2
            } else {
                0.1
            }
            #[cfg(not(feature = "runtime-pickup-pressure-preview"))]
            0.1
        })
            .contains(&opt.smooth)
    {
        return Err(Error::Inputs);
    }
    let mut preview = Preview::new_at(
        opt.limits,
        256,
        Some(geometries[0].corner_velocity()),
        [start[0], start[1], start[2], 0.],
    )?;
    let mut selected = ExtruderPositions::new(&[(0, 0.), (1, 0.)], 0).map_err(|_| Error::Inputs)?;
    let mut time = OriginClock {
        move_end: 0.0,
        needs_steps: 0.0,
        last_steps: 0.0,
        priming: true,
        bound: Some(0),
        #[cfg(not(feature = "runtime-pickup-preview"))]
        smoothing: opt.smooth * 0.5,
        #[cfg(feature = "runtime-pickup-preview")]
        smoothing: if opt.initial_advance == 0. {
            0.
        } else {
            opt.smooth * 0.5
        },
        #[cfg(feature = "shaped-file-debug-preview")]
        shape_delay,
        #[cfg(feature = "file-debug-preview")]
        wire: 0.0,
        #[cfg(feature = "file-debug-preview")]
        scheduled: false,
        #[cfg(feature = "file-debug-preview")]
        batched: debug_prefix_bytes != 0,
    };
    let mut result = vec![
        Event::Gain {
            at: 0.0,
            advance: opt.initial_advance,
        },
        Event::Bind {
            queue: Some(0),
            anchor: 0.0,
        },
    ];
    let mut deferred = Vec::new();
    let mut consumed = 0;
    let mut last_was_wait = false;
    #[cfg(feature = "file-debug-preview")]
    let mut encoded_bytes = debug_prefix_bytes;
    #[cfg(feature = "file-debug-preview")]
    let batch = |time: &mut OriginClock,
                 result: &mut Vec<Event>,
                 preview: &mut Preview,
                 lane: u8,
                 consumed: &mut usize,
                 deferred: &mut Vec<(usize, f64)>| {
        time.debug_batch(result, |time, target, result| {
            if !time.priming && target >= time.move_end - time.delay() - 0.001 {
                for movement in preview.finish()? {
                    time.profile(result, lane, movement, consumed, deferred)?;
                }
                time.priming = true;
            }
            Ok(())
        })
    };
    #[cfg(feature = "file-debug-preview")]
    let mut prev_chunk = 0;
    for (i, line) in script.lines().enumerate() {
        #[cfg(feature = "file-debug-preview")]
        if time.batched {
            encoded_bytes += line.len() + 1;
            let chunk = (encoded_bytes - 1) / 4096;
            if chunk > prev_chunk {
                batch(
                    &mut time,
                    &mut result,
                    &mut preview,
                    selected.active(),
                    &mut consumed,
                    &mut deferred,
                )?;
            }
            prev_chunk = chunk;
        }
        let pure = line.split(';').next().unwrap().trim();
        if pure.is_empty() {
            continue;
        }
        let max_lines = if debug_prefix_bytes == 0 {
            128
        } else {
            #[cfg(feature = "runtime-pickup-extended-preview")]
            if shape_updates
                .as_ref()
                .is_some_and(|plan| plan.extended_pickup)
            {
                1024
            } else {
                512
            }
            #[cfg(not(feature = "runtime-pickup-extended-preview"))]
            512
        };
        if i >= max_lines {
            return Err(Error::Inputs);
        }
        let was_wait = std::mem::take(&mut last_was_wait);
        let active = selected.active();
        let mut append = |moves: Vec<Profile>, time: &mut OriginClock| {
            for movement in moves {
                time.profile(&mut result, active, movement, &mut consumed, &mut deferred)?;
            }
            Ok::<(), Error>(())
        };
        #[cfg(feature = "runtime-shaper-preview")]
        if let Some(runtime) = shape_updates.as_mut() {
            #[cfg(feature = "runtime-mesh-preview")]
            if let Some((name, mesh)) = runtime.mesh {
                if preview.saved_mesh_command(pure, name, mesh)? {
                    continue;
                }
            }
            #[cfg(feature = "runtime-pickup-mesh-bank-preview")]
            if let Some(bank) = runtime.mesh_bank {
                if preview.saved_mesh_bank_command(pure, bank)? {
                    continue;
                }
            }
            let mut words = pure.split_ascii_whitespace();
            if words
                .next()
                .is_some_and(|w| w.eq_ignore_ascii_case("SET_INPUT_SHAPER"))
            {
                if words.next().is_none() {
                    last_was_wait = was_wait;
                    continue;
                }
                let (index, &(_, delay)) = runtime
                    .updates
                    .iter()
                    .enumerate()
                    .find(|(_, (line, _))| *line == i)
                    .ok_or(Error::Inputs)?;
                // Stock flushes pending work before updating pulse windows.
                append(preview.finish()?, &mut time)?;
                time.flush(&mut result, 0.);
                time.shape_delay = delay;
                runtime.positions.push((result.len(), index));
                continue;
            }
        }
        #[cfg(feature = "runtime-pickup-pressure-preview")]
        let pressure_override = if let Some(configured) = &mut retuned {
            let next = gcode::pickup_pressure_setting(pure, *configured)?;
            if let Some(updated) = next {
                *configured = updated;
            }
            next
        } else {
            None
        };
        #[cfg(feature = "runtime-pickup-preview")]
        let control = if runtime_pa {
            #[cfg(feature = "runtime-pickup-pressure-preview")]
            if let Some((gain, _)) = pressure_override {
                Some(gcode::SharedDebugControl::Advance(gain))
            } else {
                gcode::pickup_control(pure)?
            }
            #[cfg(not(feature = "runtime-pickup-pressure-preview"))]
            gcode::pickup_control(pure)?
        } else {
            gcode::shared_debug_control(pure)?
        };
        #[cfg(not(feature = "runtime-pickup-preview"))]
        let control = gcode::shared_debug_control(pure)?;
        match control {
            Some(gcode::SharedDebugControl::Sync(queue)) => {
                append(preview.finish()?, &mut time)?;
                time.flush(&mut result, 0.0);
                let anchor = queue
                    .map(|q| selected.observed_position(q, &preview).ok_or(Error::Inputs))
                    .transpose()?
                    .unwrap_or(0.0);
                time.bound = queue;
                result.push(Event::Bind { queue, anchor });
            }
            Some(gcode::SharedDebugControl::Activate(target)) => {
                if let Some((_, old)) = selected
                    .switch(target, &mut preview)
                    .map_err(|_| Error::Inputs)?
                {
                    append(old.moves, &mut time)?;
                    time.flush(&mut result, 0.0);
                    preview
                        .set_extruder_corner(geometries[usize::from(target)].corner_velocity())?;
                }
            }
            Some(gcode::SharedDebugControl::Advance(advance)) => {
                #[cfg(feature = "runtime-pickup-preview")]
                if let Some(pressure) = shape_updates
                    .as_mut()
                    .and_then(|s| s.pressure.as_deref_mut())
                {
                    let configured_smooth = opt.smooth;
                    #[cfg(feature = "runtime-pickup-pressure-preview")]
                    let configured_smooth = pressure_override.map_or(configured_smooth, |p| p.1);
                    let smooth = if advance == 0. { 0. } else { configured_smooth };
                    if smooth * 0.5 != time.smoothing {
                        append(preview.finish()?, &mut time)?;
                        time.flush(&mut result, 0.);
                        time.smoothing = smooth * 0.5;
                        pressure.push(PressureBoundary {
                            before: result.len(),
                            advance,
                            smooth,
                        });
                        continue;
                    }
                    if preview.queued_moves() == 0 {
                        if time.priming {
                            time.resync_print_time();
                        }
                        result.push(Event::Gain {
                            at: time.move_end,
                            advance,
                        });
                        continue;
                    }
                }
                let due = consumed + preview.queued_moves();
                if due == consumed {
                    // A standalone positive adjustment before any pending
                    // move isn't part of this exact bounded debug fixture.
                    return Err(Error::Inputs);
                }
                deferred.push((due, advance));
            }
            None if pure.eq_ignore_ascii_case("M400") => {
                append(preview.finish()?, &mut time)?;
                // In stock file-output mode, M400 flushes the look-ahead
                // without waiting or advancing step generation.
                time.priming = true;
                last_was_wait = true;
            }
            None => {
                let mut words = pure.split_ascii_whitespace();
                let opcode = words.next().ok_or(Error::Inputs)?;
                if opcode.eq_ignore_ascii_case("G4") {
                    let delay = if let Some(word) = words.next() {
                        let raw = word
                            .strip_prefix('P')
                            .or_else(|| word.strip_prefix('p'))
                            .ok_or(Error::Inputs)?;
                        let magnitude = raw.strip_prefix('+').unwrap_or(raw);
                        if words.next().is_some()
                            || !magnitude.bytes().all(|b| b.is_ascii_digit() || b == b'.')
                        {
                            return Err(Error::Inputs);
                        }
                        let ms = raw.parse::<f64>().map_err(|_| Error::Inputs)?;
                        if !ms.is_finite() || ms < 0. {
                            return Err(Error::Inputs);
                        }
                        ms / 1000.
                    } else {
                        0.
                    };
                    append(preview.finish()?, &mut time)?;
                    time.dwell(delay)?;
                    continue;
                }
                if ![
                    "G0",
                    "G1",
                    "G90",
                    "G91",
                    "G92",
                    "M82",
                    "M83",
                    "M220",
                    "M221",
                    "M204",
                    "SET_VELOCITY_LIMIT",
                    "SAVE_GCODE_STATE",
                    "RESTORE_GCODE_STATE",
                    "SET_GCODE_OFFSET",
                ]
                .iter()
                .any(|code| code.eq_ignore_ascii_case(opcode))
                {
                    // Slicer display metadata does not schedule any work.
                    if gcode::display::parse(line)?.is_some() {
                        last_was_wait = was_wait;
                        continue;
                    }
                    return Err(Error::Inputs);
                }
                let lane = usize::from(active);
                let out = preview.line_with(pure, |previous, movement| {
                    if !inside(&movement.end_position()[..3]) {
                        return Err(gcode::Error::MoveOutsidePreview);
                    }
                    if let Some(z) = movement.z_ratio() {
                        movement.limit(opt.z_velocity * z, opt.z_accel * z)?;
                    }
                    geometries[lane]
                        .check_move(movement, previous, opt.simulation_temp[lane])
                        .map_err(|_| gcode::Error::MoveOutsidePreview)
                })?;
                append(out.moves, &mut time)?;
            }
        }
    }
    if !last_was_wait || !deferred.is_empty() {
        return Err(Error::Inputs);
    }
    #[cfg(feature = "file-debug-preview")]
    if time.batched {
        batch(
            &mut time,
            &mut result,
            &mut preview,
            selected.active(),
            &mut consumed,
            &mut deferred,
        )?;
        time.flush(&mut result, 0.0);
    }
    time.flush(&mut result, 0.25);
    Ok(result)
}

#[cfg(feature = "runtime-mesh-preview")]
pub use runtime_shaper::trace_file_debug_runtime_mesh;

#[cfg(feature = "runtime-pickup-preview")]
pub use runtime_shaper::trace_file_debug_runtime_pickup;

#[cfg(feature = "runtime-pickup-mesh-preview")]
pub use runtime_shaper::trace_file_debug_runtime_pickup_mesh;

#[cfg(feature = "runtime-pickup-mesh-bank-preview")]
pub use runtime_shaper::trace_file_debug_runtime_pickup_mesh_bank;

#[cfg(feature = "runtime-pickup-pressure-preview")]
pub use runtime_shaper::trace_file_debug_runtime_pickup_pressure;

#[cfg(feature = "runtime-pickup-extended-preview")]
pub use runtime_shaper::{
    trace_file_debug_runtime_pickup_extended, trace_file_debug_runtime_pickup_extended_parallel,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extrusion::Settings;

    #[test]
    fn requires_terminal_wait_matching_target_and_bounded_coordinates() {
        let e = Extruder::new(Settings {
            nozzle: 0.4,
            filament: 1.75,
            max_area: 0.64,
            retraction_distance: 50.,
            retraction_velocity: 30.,
            retraction_accel: 300.,
            corner_velocity: 1.,
            min_simulated_temperature: 170.,
        })
        .unwrap();
        let opt = Options {
            limits: Limits::new(250., 4000., 5., 0.5).unwrap(),
            z_velocity: 15.,
            z_accel: 200.,
            xyz_bounds: [[0., 300.]; 3],
            simulation_temp: [210.; 2],
            smooth: 0.04,
            initial_advance: 0.04,
        };
        let execute = |gcode| plan(gcode, [10., 10., 1.], opt, [&e, &e]);
        for denied in [
            "G1 X10.5",
            "G1 X301\nM400",
            "SYNC_EXTRUDER_MOTION EXTRUDER=other MOTION_QUEUE=extruder\nM400",
            "SYNC_EXTRUDER_MOTION EXTRUDER=shared_extruder MOTION_QUEUE=extruder2\nM400",
            "ACTIVATE_EXTRUDER EXTRUDER=extruder1 EXTRUDER=extruder\nM400",
            "SET_PRESSURE_ADVANCE EXTRUDER=shared_extruder ADVANCE=.065\nM400",
            "M400\nG1 X11",
            "G28\nM400",
            "G4 P-1\nM400",
            "G1 X11\nM73 P100",
            "M204 S0\nM400",
            "SET_VELOCITY_LIMIT ACCEL=0\nM400",
            "SET_VELOCITY_LIMIT ACCEL=900 ACCEL=1000\nM400",
            "G4 PNaN\nM400",
            "G4 P1e2\nM400",
            "G4 S10\nM400",
            "G4 P1 P2\nM400",
            "G4 P120000\nM400",
            "M400\nG4 P5",
        ] {
            assert!(execute(denied).is_err(), "{denied}");
        }
        let good = execute("M83\nG1 X12 E.08\nSYNC_EXTRUDER_MOTION EXTRUDER=shared_extruder MOTION_QUEUE=\nG1 X14 E.07\nM400").unwrap();
        assert!(good
            .iter()
            .any(|e| matches!(e, Event::Bind { queue: None, .. })));
        assert_eq!(
            good.iter()
                .filter(|e| matches!(e, Event::Profile { .. }))
                .count(),
            2
        );
        let waits =
            execute("M400\nM83\nG1 X12 E.08\nM400\nM400\nG1 X14 E.07\nM400; drain\n; end").unwrap();
        assert_eq!(
            waits
                .iter()
                .filter(|e| matches!(e, Event::Profile { .. }))
                .count(),
            2
        );
        assert_eq!(
            waits
                .iter()
                .filter(|e| matches!(e, Event::Generation { .. }))
                .count(),
            1
        );
        assert!(execute("G1 X12\nM400\nG28\nM400").is_err());
    }
}

#[cfg(feature = "runtime-shaper-workers")]
mod runtime_workers;
#[cfg(feature = "runtime-shaper-workers")]
pub use runtime_workers::trace_file_debug_runtime_shaper_parallel;
