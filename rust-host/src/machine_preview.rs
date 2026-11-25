// SPDX-License-Identifier: GPL-3.0-only
//! Connects a strict machine config subset to the in-memory XYZ step trace.
//! No powered homing, peripherals, tool/pin setup or hardware I/O.

use crate::{
    bed_mesh,
    config::Config,
    extrusion::{self, Extruder},
    gcode::{self, Output, Preview, TransformOwner},
    homing::{self, Kinematics, VirtualHoming},
    input_shaper::Shaper,
    input_shaper_defs,
    lookahead::{Limits, Profile},
    pressure_advance::{self, Advance, GainCallbacks},
    step_clock::{self, Clock, Command},
    step_times::{self, Axis, Event, Solver},
    toolchange::{self, split_tool_gcode, ExtruderPositions, SegmentError},
    trajectory::{self, Trajectory},
};

pub mod mixed;
pub mod tool_profiles;

const SECTIONS: [&str; 3] = ["stepper_x", "stepper_y", "stepper_z"];
const MAX_EVENTS: usize = 65_535;
const MAX_LINES: usize = 32768;
const MAX_LANE_WORKERS: usize = 8;
const PARALLEL_SEGMENTS: usize = 512;

#[derive(Debug)]
pub enum Error {
    Config {
        section: &'static str,
        option: &'static str,
    },
    Unsupported,
    InitialPosition,
    VirtualHoming(homing::Error),
    MaximumLines,
    LaneWorkers,
    Gcode {
        line: usize,
        error: gcode::Error,
    },
    Extrusion(extrusion::Error),
    ToolSegment,
    ToolDispatch(SegmentError),
    Dock(toolchange::Failure),
    Trajectory(trajectory::Error),
    Solver(step_times::Error),
    Clock(step_clock::Error),
}

pub struct Machine {
    motor_axis: [Axis; 3],
    motor_step: [f64; 3],
    travel: [(f64, f64); 3],
    limits: Limits,
    z_velocity: f64,
    z_accel: f64,
    arc_resolution: Option<f64>,
    fixed_shaper: Option<Shaper>,
    fixed_mesh: Option<bed_mesh::Transform>,
    script_mesh: Option<(String, bed_mesh::Transform)>,
    lane_workers: usize,
    virtual_tool_pressure_script: bool,
    pressure_tool_names: Vec<(String, u8)>,
}

fn config_error(section: &'static str, option: &'static str) -> Error {
    Error::Config { section, option }
}

fn number(
    cfg: &Config,
    section: &'static str,
    key: &'static str,
    default: Option<f64>,
) -> Result<f64, Error> {
    cfg.get(section, key)
        .map_or(default, |s| s.parse().ok())
        .filter(|v| v.is_finite())
        .ok_or_else(|| config_error(section, key))
}

fn gear_product(cfg: &Config, section: &'static str) -> Result<f64, Error> {
    gearing(cfg.get(section, "gear_ratio")).ok_or_else(|| config_error(section, "gear_ratio"))
}

fn gearing(raw: Option<&str>) -> Option<f64> {
    let Some(raw) = raw else {
        return Some(1.0);
    };
    fn valid(v: &str) -> Option<f64> {
        v.trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite() && *v > 0.0)
    }
    raw.split(',').try_fold(1.0_f64, |product, part| {
        let (a, b) = part.split_once(':')?;
        let ratio = product * (valid(a)? / valid(b)?);
        (ratio.is_finite() && ratio > 0.0).then_some(ratio)
    })
}

/// All values are for offline simulation; no physical thermistor is read.
#[derive(Clone, Copy)]
pub struct SimulatedExtruder<'a> {
    pub geometry: &'a Extruder,
    pub temperature: f64,
    pub step_distance: f64,
    pub inverse_direction: bool,
    pub pressure_advance: Option<&'a Advance>,
}

/// A bounded fixture's per-tool motion calibration and starting local E.
#[derive(Clone, Copy)]
pub struct ToolLane<'a> {
    pub id: u8,
    pub parked_e: f64,
    pub extruder: SimulatedExtruder<'a>,
}

/// Two synthetic lanes and a scripted, proof-gated exchange. The observed
/// ID and virtual homing evidence are simulation-only inputs.
#[derive(Clone, Copy)]
pub struct DockInput<'a> {
    pub catalog: &'a toolchange::Catalog,
    pub outgoing: ToolLane<'a>,
    pub incoming: ToolLane<'a>,
    pub scripts: toolchange::Scripts<'a>,
    pub detected_after_pickup: Option<u8>,
}

/// Explicit tool phases, never an implicit T-command or docking macro.
pub struct ToolSegment<'a> {
    pub tool: u8,
    pub gcode: &'a str,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlannedTool {
    pub id: u8,
    /// Absolute planned trajectory time, not physical actuation or pickup.
    pub at: f64,
}

pub struct MultiTrace {
    pub motors: [Vec<Command>; 3],
    pub tools: Vec<(u8, Vec<Command>)>,
    pub switches: Vec<PlannedTool>,
    pub duration: f64,
}

impl Machine {
    /// Supports one motor per X/A, Y/B and Z rail, no transform/pluggable
    /// geometry. Peripheral sections are not initialized or interpreted.
    pub fn from_config(config: &Config) -> Result<Self, Error> {
        let motor_axis = match config.get("printer", "kinematics") {
            Some("cartesian") => [Axis::X, Axis::Y, Axis::Z],
            Some("corexy") => [Axis::CoreXYPlus, Axis::CoreXYMinus, Axis::Z],
            _ => return Err(config_error("printer", "kinematics")),
        };
        if config
            .sections()
            .any(|s| s == "dual_carriage" || SECTIONS.iter().any(|p| s.starts_with(p) && s != *p))
        {
            return Err(Error::Unsupported);
        }
        let max_v = number(config, "printer", "max_velocity", None)?;
        let max_a = number(config, "printer", "max_accel", None)?;
        let scv = number(config, "printer", "square_corner_velocity", Some(5.0))?;
        let cruise_ratio = number(config, "printer", "minimum_cruise_ratio", Some(0.5))?;
        let limits = Limits::new(max_v, max_a, scv, cruise_ratio)
            .map_err(|_| config_error("printer", "motion limits"))?;
        let z_velocity = number(config, "printer", "max_z_velocity", Some(max_v))?;
        let z_accel = number(config, "printer", "max_z_accel", Some(max_a))?;
        if !(0.0 < z_velocity && z_velocity <= max_v && 0.0 < z_accel && z_accel <= max_a) {
            return Err(config_error("printer", "Z limits"));
        }
        let mut travel = [(0.0, 0.0); 3];
        let mut motor_step = [0.0; 3];
        for (i, section) in SECTIONS.into_iter().enumerate() {
            let minimum = number(config, section, "position_min", Some(0.0))?;
            let maximum = number(config, section, "position_max", None)?;
            if minimum >= maximum {
                return Err(config_error(section, "position_max"));
            }
            travel[i] = (minimum, maximum);
            let rotation = number(config, section, "rotation_distance", None)?;
            let integer = |key, fallback| -> Result<u32, Error> {
                config
                    .get(section, key)
                    .map_or(fallback, |s| s.parse().ok())
                    .filter(|v| *v > 0)
                    .ok_or_else(|| config_error(section, key))
            };
            let micro = integer("microsteps", None)?;
            let full = integer("full_steps_per_rotation", Some(200))?;
            if full % 4 != 0 {
                return Err(config_error(section, "full_steps_per_rotation"));
            }
            let steps = f64::from(micro) * f64::from(full) * gear_product(config, section)?;
            let step = rotation / steps;
            if !(1e-9..=1000.0).contains(&step) {
                return Err(config_error(section, "rotation_distance"));
            }
            motor_step[i] = step;
        }
        let arc_resolution = if config.values("gcode_arcs").is_some() {
            let resolution = number(config, "gcode_arcs", "resolution", Some(1.0))?;
            if resolution <= 0.0 {
                return Err(config_error("gcode_arcs", "resolution"));
            }
            Some(resolution)
        } else {
            None
        };
        Ok(Self {
            motor_axis,
            motor_step,
            travel,
            limits,
            z_velocity,
            z_accel,
            arc_resolution,
            fixed_shaper: input_shaper_defs::from_config(config)
                .map_err(|field| config_error("input_shaper", field))?,
            fixed_mesh: None,
            script_mesh: None,
            lane_workers: 1,
            virtual_tool_pressure_script: false,
            pressure_tool_names: Vec::new(),
        })
    }

    pub fn step_distances(&self) -> [f64; 3] {
        self.motor_step
    }

    fn in_range(&self, pos: &[f64]) -> bool {
        self.travel
            .iter()
            .zip(pos)
            .all(|(&(lo, hi), x)| (lo..=hi).contains(x))
    }

    /// Apply an explicitly supplied, fixed offline shaper. No accelerometer,
    /// config calibration, or dynamic SET_INPUT_SHAPER behavior is implied.
    /// Limit CPU workers used for independent lanes of long offline tool traces.
    /// The result and lane order are independent of the requested worker count.
    pub fn with_lane_workers(mut self, count: usize) -> Result<Self, Error> {
        if !(1..=MAX_LANE_WORKERS).contains(&count) {
            return Err(Error::LaneWorkers);
        }
        self.lane_workers = count;
        Ok(self)
    }

    /// Opt in to positive, fixed-smoothing gain changes for virtual tool
    /// selection only. This does not enable them during docking programs.
    pub fn with_virtual_tool_pressure_script(mut self) -> Self {
        self.virtual_tool_pressure_script = true;
        self
    }

    /// Also allow exact backing extruder names for independently stepped
    /// virtual lanes. Tool numbers (such as T6) are not extruder names.
    pub fn with_configured_tool_pressure_script(
        mut self,
        profiles: &[tool_profiles::Profile],
    ) -> Self {
        self.virtual_tool_pressure_script = true;
        self.pressure_tool_names = profiles.iter().map(|p| (p.section.clone(), p.id)).collect();
        self
    }

    /// Enable one fixed offline mesh. This does not calibrate or probe it.
    pub fn with_bed_mesh(mut self, mesh: bed_mesh::Transform) -> Self {
        self.fixed_mesh = Some(mesh);
        self
    }

    pub fn with_input_shaper(mut self, shaper: Shaper) -> Self {
        self.fixed_shaper = Some(shaper);
        self
    }

    fn homing_kinematics(&self) -> Kinematics {
        match self.motor_axis[0] {
            Axis::X => Kinematics::Cartesian,
            _ => Kinematics::CoreXy,
        }
    }

    /// Permit load and clear of exactly one validated mesh during a single
    /// offline motion trace. Do not load it before reading the script.
    pub fn with_script_bed_mesh(mut self, name: String, mesh: bed_mesh::Transform) -> Self {
        self.fixed_mesh = None;
        self.script_mesh = Some((name, mesh));
        self
    }

    /// Create a gate using this machine's geometry. No G28 command, firmware
    /// interface or pin action is run; the fixture must feed the evidence.
    pub fn virtual_homing(&self, position: [f64; 3]) -> Result<VirtualHoming, Error> {
        VirtualHoming::new(self.homing_kinematics(), position, self.motor_step)
            .map_err(Error::VirtualHoming)
    }

    /// Inert preview path that requires a complete virtual home of the
    /// compatible geometry. The unrestricted trace remains useful for fixtures.
    pub fn trace_homed(
        &self,
        script: &str,
        home: &VirtualHoming,
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<Trace, Error> {
        self.trace_homed_with_extruder(script, home, at, clock, max_error, invert, None)
    }

    /// Optional inert extrusion on the same verified virtual home geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn trace_homed_with_extruder(
        &self,
        script: &str,
        home: &VirtualHoming,
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
        extruder: Option<SimulatedExtruder<'_>>,
    ) -> Result<Trace, Error> {
        let start = home
            .ready_position_for(self.homing_kinematics(), self.motor_step)
            .map_err(Error::VirtualHoming)?;
        self.trace_with_extruder(script, start, at, clock, max_error, invert, extruder)
    }

    /// Previews a bounded script from an explicit virtual position. Inversion
    /// flags are caller-supplied (no parsing device pin configuration).
    pub fn trace(
        &self,
        script: &str,
        start: [f64; 3],
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<Trace, Error> {
        self.trace_with_extruder(script, start, at, clock, max_error, invert, None)
    }

    /// Preview one inert dock exchange with real preview geometry limits.
    /// Validate every synthetic move (including generated restore travel)
    /// during planning. No partial profiles escape a failed exchange.
    pub fn preview_dock(
        &self,
        home: &VirtualHoming,
        changer: &mut toolchange::Changer,
        input: DockInput<'_>,
    ) -> Result<toolchange::LanePreview, Error> {
        let start = home
            .ready_position_for(self.homing_kinematics(), self.motor_step)
            .map_err(Error::VirtualHoming)?;
        if self.fixed_mesh.is_some() {
            return Err(Error::Unsupported);
        }
        if !self.in_range(&start) || input.outgoing.id == input.incoming.id {
            return Err(Error::ToolSegment);
        }
        let [outgoing, incoming] = [input.outgoing, input.incoming];
        let offsets = input
            .catalog
            .numbered(outgoing.id)
            .ok_or(Error::ToolSegment)?
            .nozzle_offset;
        let slots = ExtruderPositions::new(
            &[
                (outgoing.id, outgoing.parked_e),
                (incoming.id, incoming.parked_e),
            ],
            outgoing.id,
        )
        .map_err(|_| Error::ToolSegment)?;
        let owner = TransformOwner(1);
        let prepare = |error| Error::Gcode { line: 0, error };
        let mut preview = Preview::new_at(
            self.limits,
            4096,
            Some(outgoing.extruder.geometry.corner_velocity()),
            [start[0], start[1], start[2], outgoing.parked_e],
        )
        .map_err(prepare)?;
        preview
            .wrap_translation(owner, offsets, true)
            .map_err(prepare)?;
        preview.reset_last_position().map_err(prepare)?;
        if let Some(resolution) = self.arc_resolution {
            preview.enable_arcs(resolution).map_err(prepare)?;
        }
        let mut extruder_error = None;
        let result = input.catalog.preview_change_lanes_checked(
            changer,
            preview,
            owner,
            toolchange::ChangeRequest {
                target: Some(incoming.id),
                homed_axes: 7,
                detected_after_pickup: input.detected_after_pickup,
                scripts: input.scripts,
            },
            slots,
            Some(incoming.extruder.geometry.corner_velocity()),
            |tool, previous, next| {
                let end = next.end_position();
                if !self.in_range(&end[..3]) {
                    return Err(gcode::Error::MoveOutsidePreview);
                }
                if let Some(z) = next.z_ratio() {
                    next.limit(z * self.z_velocity, z * self.z_accel)?;
                }
                if end[3] != previous[3] {
                    let ext = if tool == outgoing.id {
                        &outgoing.extruder
                    } else {
                        &incoming.extruder
                    };
                    ext.geometry
                        .check_move(next, previous, ext.temperature)
                        .map_err(|reason| {
                            extruder_error = Some(reason);
                            gcode::Error::MoveOutsidePreview
                        })?;
                }
                Ok(())
            },
        );
        result.map_err(|e| extruder_error.map_or(Error::Dock(e), Error::Extrusion))
    }

    /// Produce an inert motion trace only after the exchange is fully
    /// validated and the observed pickup satisfies the configured policy.
    /// Failure during step encoding leaves the prior changer state intact.
    #[allow(clippy::too_many_arguments)]
    pub fn trace_dock(
        &self,
        home: &VirtualHoming,
        changer: &mut toolchange::Changer,
        input: DockInput<'_>,
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<MultiTrace, Error> {
        if !(0.0..1e12).contains(&at) || at == 0.0 {
            return Err(Error::InitialPosition);
        }
        let start = home
            .ready_position_for(self.homing_kinematics(), self.motor_step)
            .map_err(Error::VirtualHoming)?;
        let lanes = [input.outgoing, input.incoming];
        let mut trial = changer.clone();
        let draft = match self.preview_dock(home, &mut trial, input) {
            Ok(draft) => draft,
            Err(error) => {
                if trial.state() == toolchange::State::Faulted {
                    *changer = trial;
                }
                return Err(error);
            }
        };
        let mut xyz = Trajectory::new(16_384).map_err(Error::Trajectory)?;
        let mut all_e = vec![
            Trajectory::new(16_384).map_err(Error::Trajectory)?,
            Trajectory::new(16_384).map_err(Error::Trajectory)?,
        ];
        let mut time = at;
        let mut switches = Vec::new();
        for group in draft.groups {
            let lane = usize::from(group.tool == input.incoming.id);
            if lane == 1 {
                switches.push(PlannedTool {
                    id: group.tool,
                    at: time,
                });
            }
            for output in group.phases {
                for profile in output.moves {
                    extrusion::append_nominal(&mut all_e[lane], time, &profile)
                        .map_err(Error::Trajectory)?;
                    append_move(&mut xyz, &mut time, &profile).map_err(Error::Trajectory)?;
                }
                time += output.dwell_seconds.unwrap_or(0.);
                if !time.is_finite() || time >= 1e12 {
                    return Err(Error::Trajectory(trajectory::Error::Invalid));
                }
            }
        }
        let trace = self.lane_traces(
            &xyz,
            all_e,
            &[],
            &lanes,
            start,
            clock,
            max_error,
            invert,
            switches,
            time - at,
        )?;
        *changer = trial;
        Ok(trace)
    }

    /// Preview bare T selections as topology changes with no physical
    /// docking or plugin calls. Every selected tool, including a trailing
    /// motionless T command, must be present in the declared lane list.
    #[allow(clippy::too_many_arguments)]
    pub fn trace_tool_gcode(
        &self,
        home: &VirtualHoming,
        script: &str,
        initial: u8,
        lanes: &[ToolLane<'_>],
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<MultiTrace, Error> {
        let pieces = split_tool_gcode(script, initial).map_err(Error::ToolDispatch)?;
        self.trace_tool_segments(home, &pieces, lanes, at, clock, max_error, invert)
    }

    /// Run bounded, explicitly selected tool phases from verified virtual home.
    /// XYZ time is shared. E positions, pressure advance, and step traces
    /// remain independent for each tool. This does not simulate tool pickup.
    #[allow(clippy::too_many_arguments)]
    pub fn trace_tool_segments(
        &self,
        home: &VirtualHoming,
        pieces: &[ToolSegment<'_>],
        lanes: &[ToolLane<'_>],
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<MultiTrace, Error> {
        let start = home
            .ready_position_for(self.homing_kinematics(), self.motor_step)
            .map_err(Error::VirtualHoming)?;
        self.trace_tool_segments_at(start, pieces, lanes, at, clock, max_error, invert)
    }

    /// Non-device preview from an explicitly chosen virtual XYZ origin.
    /// In contrast to trace_tool_gcode, this does not claim virtual homing.
    #[allow(clippy::too_many_arguments)]
    pub fn trace_tool_gcode_at_virtual_origin(
        &self,
        start: [f64; 3],
        script: &str,
        initial: u8,
        lanes: &[ToolLane<'_>],
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<MultiTrace, Error> {
        let pieces = split_tool_gcode(script, initial).map_err(Error::ToolDispatch)?;
        self.trace_tool_segments_at(start, &pieces, lanes, at, clock, max_error, invert)
    }

    #[allow(clippy::too_many_arguments)]
    fn trace_tool_segments_at(
        &self,
        start: [f64; 3],
        pieces: &[ToolSegment<'_>],
        lanes: &[ToolLane<'_>],
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
    ) -> Result<MultiTrace, Error> {
        let mut bytes = 0usize;
        if !self.in_range(&start)
            || !(0.0..1e12).contains(&at)
            || at == 0.0
            || !(1..=8).contains(&lanes.len())
            || !(1..=128).contains(&pieces.len())
            || pieces.iter().any(|p| {
                bytes = bytes.saturating_add(p.gcode.len());
                bytes > 8 * 1024 * 1024
            })
        {
            return Err(Error::ToolSegment);
        }
        let lookup = |id| {
            lanes
                .iter()
                .position(|t| t.id == id)
                .ok_or(Error::ToolSegment)
        };
        let mut active = lookup(pieces[0].tool)?;
        let first = &lanes[active];
        let slots = lanes.iter().map(|t| (t.id, t.parked_e)).collect::<Vec<_>>();
        let mut positions =
            ExtruderPositions::new(&slots, first.id).map_err(|_| Error::ToolSegment)?;
        let mut preview = Preview::new_at(
            self.limits,
            4096,
            Some(first.extruder.geometry.corner_velocity()),
            [start[0], start[1], start[2], first.parked_e],
        )
        .map_err(|error| Error::Gcode { line: 0, error })?;
        preview
            .activate_extruder_position(first.parked_e)
            .map_err(|error| Error::Gcode { line: 0, error })?;
        if let Some(mesh) = &self.fixed_mesh {
            preview
                .install_bed_mesh(mesh.clone())
                .map_err(|error| Error::Gcode { line: 0, error })?;
        }
        if let Some(resolution) = self.arc_resolution {
            preview
                .enable_arcs(resolution)
                .map_err(|error| Error::Gcode { line: 0, error })?;
        }
        let mut xyz = Trajectory::new(16_384).map_err(Error::Trajectory)?;
        let mut all_e = (0..lanes.len())
            .map(|_| Trajectory::new(16_384))
            .collect::<Result<Vec<_>, _>>()
            .map_err(Error::Trajectory)?;
        let mut time = at;
        let mut switches = Vec::new();
        let mut line_no = 0;
        let mut callbacks: [Option<GainCallbacks>; 8] = std::array::from_fn(|_| None);
        let mut pending = 0u8;
        let mut consumed = 0;
        let mut append = |idx: usize,
                          out: Output,
                          time: &mut f64,
                          consumed: &mut usize,
                          callbacks: &mut [Option<GainCallbacks>; 8],
                          pending: &mut u8| {
            for profile in out.moves {
                extrusion::append_nominal(&mut all_e[idx], *time, &profile)
                    .map_err(Error::Trajectory)?;
                append_move(&mut xyz, time, &profile).map_err(Error::Trajectory)?;
                *consumed += 1;
                if *pending != 0 {
                    let mut lanes = *pending;
                    while lanes != 0 {
                        let slot = lanes.trailing_zeros() as usize;
                        let bit = 1 << slot;
                        lanes &= !bit;
                        let callback = callbacks[slot].as_mut().ok_or(Error::ToolSegment)?;
                        callback
                            .resolve(*consumed, *time)
                            .map_err(|_| Error::ToolSegment)?;
                        if !callback.has_pending() {
                            *pending &= !bit;
                        }
                    }
                }
            }
            *time += out.dwell_seconds.unwrap_or(0.0);
            if !time.is_finite() || *time >= 1e12 {
                return Err(Error::Trajectory(trajectory::Error::Invalid));
            }
            Ok(*time)
        };
        for section in pieces {
            let id = lookup(section.tool)?;
            if let Some((_, flushed)) = positions
                .switch(section.tool, &mut preview)
                .map_err(|_| Error::ToolSegment)?
            {
                let at = append(
                    active,
                    flushed,
                    &mut time,
                    &mut consumed,
                    &mut callbacks,
                    &mut pending,
                )?;
                switches.push(PlannedTool {
                    id: section.tool,
                    at,
                });
                active = id;
                preview
                    .set_extruder_corner(lanes[active].extruder.geometry.corner_velocity())
                    .map_err(|error| Error::Gcode {
                        line: line_no,
                        error,
                    })?;
            }
            let tool = &lanes[active].extruder;
            for line in section.gcode.lines() {
                line_no += 1;
                if line_no > MAX_LINES {
                    return Err(Error::MaximumLines);
                }
                if self.virtual_tool_pressure_script {
                    if let Some((gain, target)) =
                        Preview::fixed_pressure_command(line, !self.pressure_tool_names.is_empty())
                            .map_err(|error| Error::Gcode {
                                line: line_no,
                                error,
                            })?
                    {
                        let invalid = |_| Error::Gcode {
                            line: line_no,
                            error: gcode::Error::InvalidParameter,
                        };
                        let target_lane = if let Some(target) = target {
                            let id = self
                                .pressure_tool_names
                                .iter()
                                .find_map(|(name, id)| (name == &target).then_some(*id))
                                .ok_or_else(|| invalid(pressure_advance::Error::Settings))?;
                            lookup(id).map_err(|_| invalid(pressure_advance::Error::Settings))?
                        } else {
                            active
                        };
                        let slot = &mut callbacks[target_lane];
                        if slot.is_none() {
                            let base = lanes[target_lane]
                                .extruder
                                .pressure_advance
                                .ok_or_else(|| invalid(pressure_advance::Error::Settings))?;
                            *slot = Some(GainCallbacks::new(base, at).map_err(invalid)?);
                        }
                        let c = slot.as_mut().unwrap();
                        // Stock registers this on the shared look-ahead queue
                        // even when the target extruder is not the active lane.
                        c.defer(consumed + preview.queued_moves(), gain)
                            .map_err(invalid)?;
                        c.resolve(consumed, time).map_err(invalid)?;
                        let bit = 1 << target_lane;
                        if c.has_pending() {
                            pending |= bit;
                        } else {
                            pending &= !bit;
                        }
                        continue;
                    }
                }
                let mut ext_error = None;
                let out = preview
                    .line_with(line, |previous, next| {
                        let end = next.end_position();
                        if !self.in_range(&end[..3]) {
                            return Err(gcode::Error::MoveOutsidePreview);
                        }
                        if let Some(z) = next.z_ratio() {
                            next.limit(z * self.z_velocity, z * self.z_accel)?;
                        }
                        if end[3] != previous[3] {
                            tool.geometry
                                .check_move(next, previous, tool.temperature)
                                .map_err(|error| {
                                    ext_error = Some(error);
                                    gcode::Error::MoveOutsidePreview
                                })?;
                        }
                        Ok(())
                    })
                    .map_err(|error| {
                        ext_error.map_or(
                            Error::Gcode {
                                line: line_no,
                                error,
                            },
                            Error::Extrusion,
                        )
                    })?;
                append(
                    active,
                    out,
                    &mut time,
                    &mut consumed,
                    &mut callbacks,
                    &mut pending,
                )?;
            }
        }
        let moves = preview.finish().map_err(|error| Error::Gcode {
            line: line_no + 1,
            error,
        })?;
        append(
            active,
            Output {
                moves,
                dwell_seconds: None,
            },
            &mut time,
            &mut consumed,
            &mut callbacks,
            &mut pending,
        )?;
        let mut runtime_advance: [Option<Advance>; 8] = std::array::from_fn(|_| None);
        for (callback, advance) in callbacks.into_iter().zip(&mut runtime_advance) {
            if let Some(callback) = callback {
                *advance = Some(callback.finish().map_err(|_| Error::ToolSegment)?);
            }
        }
        self.lane_traces(
            &xyz,
            all_e,
            &runtime_advance,
            lanes,
            start,
            clock,
            max_error,
            invert,
            switches,
            time - at,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn lane_traces(
        &self,
        xyz: &Trajectory,
        all_e: Vec<Trajectory>,
        runtime_advance: &[Option<Advance>],
        lanes: &[ToolLane<'_>],
        start: [f64; 3],
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
        switches: Vec<PlannedTool>,
        duration: f64,
    ) -> Result<MultiTrace, Error> {
        let n = lanes.len() + 3;
        let workers = if xyz.segments().len() >= PARALLEL_SEGMENTS {
            self.lane_workers.min(n)
        } else {
            1
        };
        let solve = |i: usize| -> Result<Vec<Command>, Error> {
            if i < 3 {
                let axis = self.motor_axis[i];
                return if let Some(shaper) = self
                    .fixed_shaper
                    .as_ref()
                    .filter(|s| s.is_enabled_for(axis))
                {
                    let events = shaper
                        .crossings(xyz, axis, self.motor_step[i], start)
                        .map_err(Error::Solver)?;
                    step_clock::encode_trace(clock, &events, max_error, invert[i])
                        .map_err(Error::Clock)
                } else {
                    step_lane(
                        xyz,
                        axis,
                        self.motor_step[i],
                        start,
                        clock,
                        max_error,
                        invert[i],
                    )
                };
            }
            let (spec, queue) = (&lanes[i - 3], &all_e[i - 3]);
            let tool = &spec.extruder;
            if let Some(advance) = runtime_advance
                .get(i - 3)
                .and_then(Option::as_ref)
                .or(tool.pressure_advance)
            {
                let events = advance
                    .crossings(queue, tool.step_distance)
                    .map_err(Error::Solver)?;
                step_clock::encode_trace(clock, &events, max_error, tool.inverse_direction)
                    .map_err(Error::Clock)
            } else {
                step_lane(
                    queue,
                    Axis::X,
                    tool.step_distance,
                    [spec.parked_e, 0., 0.],
                    clock,
                    max_error,
                    tool.inverse_direction,
                )
            }
        };
        let mut solved = solve_lanes(n, workers, solve)?.into_iter();
        let motors = std::array::from_fn(|_| solved.next().expect("three motion lanes"));
        let tools = lanes.iter().map(|spec| spec.id).zip(solved).collect();
        Ok(MultiTrace {
            motors,
            tools,
            switches,
            duration,
        })
    }

    /// Offline trace with an explicitly modeled single extruder.
    #[allow(clippy::too_many_arguments)]
    pub fn trace_with_extruder(
        &self,
        script: &str,
        start: [f64; 3],
        at: f64,
        clock: Clock,
        max_error: u32,
        invert: [bool; 3],
        extruder: Option<SimulatedExtruder<'_>>,
    ) -> Result<Trace, Error> {
        if !self.in_range(&start) || !at.is_finite() || at <= 0.0 {
            return Err(Error::InitialPosition);
        }
        if script.len() > 8 * 1024 * 1024 {
            return Err(Error::MaximumLines);
        }
        let mut preview = Preview::new_at(
            self.limits,
            4096,
            extruder.as_ref().map(|e| e.geometry.corner_velocity()),
            [start[0], start[1], start[2], 0.0],
        )
        .map_err(|error| Error::Gcode { line: 0, error })?;
        if let Some(mesh) = &self.fixed_mesh {
            preview
                .install_bed_mesh(mesh.clone())
                .map_err(|error| Error::Gcode { line: 0, error })?;
        }
        if let Some(resolution) = self.arc_resolution {
            preview
                .enable_arcs(resolution)
                .map_err(|error| Error::Gcode { line: 0, error })?;
        }
        let mut queue = Trajectory::new(16_384).map_err(Error::Trajectory)?;
        let mut e_queue = if extruder.is_some() {
            Some(Trajectory::new(16_384).map_err(Error::Trajectory)?)
        } else {
            None
        };
        let mut time = at;
        let mut n = 0;
        let mut consumed = 0;
        let mut callbacks: Option<GainCallbacks> = None;
        let mut append = |o: Output,
                          time: &mut f64,
                          consumed: &mut usize,
                          callbacks: &mut Option<GainCallbacks>,
                          line|
         -> Result<(), Error> {
            for m in o.moves {
                if let Some(queue) = &mut e_queue {
                    extrusion::append_nominal(queue, *time, &m).map_err(Error::Trajectory)?;
                }
                append_move(&mut queue, time, &m).map_err(Error::Trajectory)?;
                *consumed += 1;
                if let Some(c) = callbacks {
                    c.resolve(*consumed, *time).map_err(|_| Error::Gcode {
                        line,
                        error: gcode::Error::InvalidSession,
                    })?;
                }
            }
            *time += o.dwell_seconds.unwrap_or(0.0);
            if !time.is_finite() || *time >= 1e12 {
                return Err(Error::Trajectory(trajectory::Error::Invalid));
            }
            Ok(())
        };
        for line in script.lines() {
            n += 1;
            if n > MAX_LINES {
                return Err(Error::MaximumLines);
            }
            if let Some((name, mesh)) = &self.script_mesh {
                if preview
                    .saved_mesh_command(line, name, mesh)
                    .map_err(|error| Error::Gcode { line: n, error })?
                {
                    continue;
                }
            }
            if let Some(gain) = Preview::fixed_pressure_gain(line)
                .map_err(|error| Error::Gcode { line: n, error })?
            {
                let invalid = |_| Error::Gcode {
                    line: n,
                    error: gcode::Error::InvalidParameter,
                };
                if callbacks.is_none() {
                    let base = extruder
                        .as_ref()
                        .and_then(|e| e.pressure_advance)
                        .ok_or_else(|| invalid(pressure_advance::Error::Settings))?;
                    callbacks = Some(GainCallbacks::new(base, at).map_err(invalid)?);
                }
                let c = callbacks.as_mut().unwrap();
                // Match stock's current position when empty, otherwise the
                // end of the last queued motion, without requesting a flush.
                c.defer(consumed + preview.queued_moves(), gain)
                    .map_err(invalid)?;
                c.resolve(consumed, time).map_err(invalid)?;
                continue;
            }
            let mut extrusion_error = None;
            let output = preview
                .line_with(line, |previous, next| {
                    let end = next.end_position();
                    if !self.in_range(&end[..3]) {
                        return Err(gcode::Error::MoveOutsidePreview);
                    }
                    if let Some(z) = next.z_ratio() {
                        next.limit(z * self.z_velocity, z * self.z_accel)?;
                    }
                    if end[3] != previous[3] {
                        let Some(ex) = &extruder else {
                            return Err(gcode::Error::MoveOutsidePreview);
                        };
                        if let Err(err) = ex.geometry.check_move(next, previous, ex.temperature) {
                            extrusion_error = Some(err);
                            return Err(gcode::Error::MoveOutsidePreview);
                        }
                    }
                    Ok(())
                })
                .map_err(|error| match extrusion_error {
                    Some(e) => Error::Extrusion(e),
                    None => Error::Gcode { line: n, error },
                })?;
            append(output, &mut time, &mut consumed, &mut callbacks, n)?;
        }
        let moves = preview
            .finish()
            .map_err(|error| Error::Gcode { line: n + 1, error })?;
        append(
            Output {
                moves,
                dwell_seconds: None,
            },
            &mut time,
            &mut consumed,
            &mut callbacks,
            n + 1,
        )?;
        let runtime_advance = callbacks
            .map(GainCallbacks::finish)
            .transpose()
            .map_err(|_| Error::Gcode {
                line: n + 1,
                error: gcode::Error::InvalidSession,
            })?;
        let mut commands = std::array::from_fn(|_| Vec::new());
        for (motor, output) in commands.iter_mut().enumerate() {
            let (axis, dist, inv) = (
                self.motor_axis[motor],
                self.motor_step[motor],
                invert[motor],
            );
            *output = if let Some(shaper) = self
                .fixed_shaper
                .as_ref()
                .filter(|s| s.is_enabled_for(axis))
            {
                let events = shaper
                    .crossings(&queue, axis, dist, start)
                    .map_err(Error::Solver)?;
                step_clock::encode_trace(clock, &events, max_error, inv).map_err(Error::Clock)?
            } else {
                step_lane(&queue, axis, dist, start, clock, max_error, inv)?
            };
        }
        let extruder = match (e_queue, extruder) {
            (Some(queue), Some(e)) => Some(
                if let Some(advance) = runtime_advance.as_ref().or(e.pressure_advance) {
                    let raw = advance
                        .crossings(&queue, e.step_distance)
                        .map_err(Error::Solver)?;
                    step_clock::encode_trace(clock, &raw, max_error, e.inverse_direction)
                        .map_err(Error::Clock)?
                } else {
                    step_lane(
                        &queue,
                        Axis::X,
                        e.step_distance,
                        [0.0; 3],
                        clock,
                        max_error,
                        e.inverse_direction,
                    )?
                },
            ),
            _ => None,
        };
        Ok(Trace {
            motors: commands,
            extruder,
            duration: time - at,
        })
    }
}

// Plan shared geometry once. With more than two workers and more lanes
// than workers, claim new lanes as each finishes: shaped motion and the
// extruder timelines can have very different solve costs.
fn solve_lanes(
    count: usize,
    workers: usize,
    solve: impl Fn(usize) -> Result<Vec<Command>, Error> + Sync,
) -> Result<Vec<Vec<Command>>, Error> {
    if workers == 1 {
        return (0..count).map(solve).collect();
    }
    use std::sync::atomic::{AtomicUsize, Ordering};
    let next = AtomicUsize::new(0);
    let run = |start| {
        if workers <= 2 || workers >= count {
            return (start..count)
                .step_by(workers)
                .map(|i| (i, solve(i)))
                .collect::<Vec<_>>();
        }
        let mut completed = Vec::new();
        loop {
            let i = next.fetch_add(1, Ordering::Relaxed);
            if i >= count {
                return completed;
            }
            completed.push((i, solve(i)));
        }
    };
    std::thread::scope(|scope| {
        let mut handles = Vec::new();
        let mut spawn_failed = false;
        for id in 1..workers {
            let run = &run;
            match std::thread::Builder::new().spawn_scoped(scope, move || run(id)) {
                Ok(worker) => handles.push(worker),
                Err(_) => {
                    spawn_failed = true;
                    break;
                }
            }
        }
        let mut results = run(0);
        for worker in handles {
            match worker.join() {
                Ok(result) => results.extend(result),
                Err(_) => spawn_failed = true,
            }
        }
        if spawn_failed {
            return Err(Error::LaneWorkers);
        }
        results.sort_unstable_by_key(|(i, _)| *i);
        results.into_iter().map(|(_, trace)| trace).collect()
    })
}

fn step_lane(
    queue: &Trajectory,
    axis: Axis,
    distance: f64,
    start: [f64; 3],
    clock: Clock,
    max_error: u32,
    invert: bool,
) -> Result<Vec<Command>, Error> {
    let mut solver = Solver::new(axis, distance, start).map_err(Error::Solver)?;
    let mut events: Vec<Event> = Vec::new();
    for seg in queue.segments().iter().filter(|s| s.direction != [0.0; 3]) {
        let room = MAX_EVENTS
            .checked_sub(events.len())
            .filter(|n| *n > 0)
            .ok_or(Error::Solver(step_times::Error::EventLimit))?;
        solver
            .crossing_events_into(seg, seg.at..seg.end(), room, &mut events)
            .map_err(Error::Solver)?;
    }
    step_clock::encode_trace(clock, &events, max_error, invert).map_err(Error::Clock)
}

fn append_move(queue: &mut Trajectory, at: &mut f64, m: &Profile) -> Result<(), trajectory::Error> {
    if m.kinematic {
        queue.append_profile(*at, m)?;
        *at = queue.segments().last().unwrap().end();
    } else {
        let cruise = if (-1e-12..0.0).contains(&m.cruise_t) {
            0.0
        } else {
            m.cruise_t
        };
        *at = *at + m.accel_t + cruise + m.decel_t;
    }
    Ok(())
}

pub struct Trace {
    pub motors: [Vec<Command>; 3],
    pub extruder: Option<Vec<Command>>,
    pub duration: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    const CONFIG: &str = "[printer]\nkinematics: corexy\nmax_velocity: 160\nmax_accel: 2500\nmax_z_velocity: 10\n\
        [stepper_x]\nposition_max: 200\nrotation_distance: 40\nmicrosteps: 16\n\
        [stepper_y]\nposition_max: 200\nrotation_distance: 40\nmicrosteps: 16\n\
        [stepper_z]\nposition_max: 200\nrotation_distance: 8\nmicrosteps: 16\ngear_ratio: 80:16, 2:1";

    #[test]
    fn previews_corexy_and_rejects_physical_parameters_outside_supported_scope() {
        let cfg = crate::config::parse(
            &BTreeMap::from([("printer.cfg".into(), CONFIG.into())]),
            "printer.cfg",
        )
        .unwrap();
        let machine = Machine::from_config(&cfg).unwrap();
        assert_eq!(machine.step_distances(), [0.0125, 0.0125, 0.00025]);
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: 20e6,
        };
        let trace = machine
            .trace(
                "G1 X1 Y1 Z0.02 F60\nG1 X2 Y2",
                [0.; 3],
                2.0,
                clock,
                0,
                [false; 3],
            )
            .unwrap();
        assert!(
            !trace.motors[0].is_empty()
                && trace.motors[1].is_empty()
                && !trace.motors[2].is_empty()
        );
        assert!(trace.duration > 2.8);
        for text in ["G1 X-0.1", "G1 E0.1", "G28", "G1 Z200.1"] {
            assert!(matches!(
                machine.trace(text, [0.; 3], 2.0, clock, 0, [false; 3]),
                Err(Error::Gcode { line: 1, .. })
            ));
        }
    }

    #[test]
    fn scoped_lanes_restore_index_order_after_independent_execution() {
        let expected: Vec<_> = (0..8)
            .map(|i| vec![Command::SetDirection(true); i])
            .collect();
        for workers in 1..=8 {
            let result =
                solve_lanes(8, workers, |i| Ok(vec![Command::SetDirection(true); i])).unwrap();
            assert_eq!(result, expected);
        }
        let result = solve_lanes(6, 4, |i| {
            if i == 2 {
                Err(Error::Unsupported)
            } else {
                Ok(vec![])
            }
        });
        assert!(matches!(result, Err(Error::Unsupported)));
    }
}
