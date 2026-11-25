// SPDX-License-Identifier: GPL-3.0-only
//! Offline motion traces isolated at fixed heater boundaries. The leading
//! ticks and local clocks are for simulations, not a firmware reset recipe.

use super::{
    append_move, bed_mesh, extrusion, gcode, step_lane, trajectory, Axis, Clock, Command,
    Error as MachineError, Machine, Preview, SimulatedExtruder, ToolLane, Trace, Trajectory,
};
use crate::{
    homing::VirtualHoming,
    thermal::{commands, planned},
};

const LEAD: f64 = 2.0;

#[derive(Debug)]
pub enum Error {
    Stage(planned::Error),
    Geometry(MachineError),
    ExtruderRequired,
    Tools(planned::ToolError),
    FanNotConfigured { line: usize },
}

pub struct Input<'a> {
    pub script: &'a str,
    pub extruder: Option<SimulatedExtruder<'a>>,
    pub hz: f64,
    pub max_error: u32,
    pub invert: [bool; 3],
}

pub enum Stage<'a> {
    Motion {
        first_line: usize,
        trace: Trace,
    },
    Heater {
        line: usize,
        outcome: commands::Output<'a>,
        targets: Vec<f64>,
    },
}

pub struct MixedTrace<'a> {
    pub hz: f64,
    pub phases: Vec<Stage<'a>>,
    pub final_position: [f64; 4],
}

#[derive(Clone, Copy)]
pub struct ToolSlot<'a> {
    pub lane: ToolLane<'a>,
    /// Explicit role used by an unqualified heater command for this tool.
    /// The T parameter of a heater command still uses stock heater indexing.
    pub heater: u32,
}

pub struct ToolInput<'a> {
    pub script: &'a str,
    pub slots: &'a [ToolSlot<'a>],
    pub initial_tool: u8,
    pub print_fan: bool,
    pub hz: f64,
    pub max_error: u32,
    pub invert: [bool; 3],
}

pub enum ToolStage<'a> {
    Motion {
        first_line: usize,
        tool: u8,
        trace: Trace,
    },
    Heater {
        line: usize,
        active_tool: u8,
        outcome: commands::Output<'a>,
        targets: Vec<f64>,
    },
    Fan {
        line: usize,
        active_tool: u8,
        demand: f64,
    },
    Select {
        line: usize,
        from: u8,
        to: u8,
    },
}

pub struct ToolTrace<'a> {
    pub hz: f64,
    pub initial_tool: u8,
    pub phases: Vec<ToolStage<'a>>,
    pub final_position: [f64; 4],
    pub final_extruders: Vec<(u8, f64)>,
}

#[derive(Clone, Copy)]
struct PhaseSettings {
    start: [f64; 3],
    clock: Clock,
    max_error: u32,
    invert: [bool; 3],
}

impl Machine {
    /// Preserve logical coordinates across stages but compile step commands
    /// separately on phase-local clocks. A future phase has no shared wire
    /// queue with an earlier stage, even if its first release tick is zero.
    /// Extrusion geometry uses the explicitly supplied modeled temperature.
    pub fn trace_mixed<'a>(
        &self,
        home: &VirtualHoming,
        bank: &commands::Bank<'a>,
        input: Input<'_>,
    ) -> Result<MixedTrace<'a>, Error> {
        let start = home
            .ready_position_for(self.homing_kinematics(), self.motor_step)
            .map_err(|e| Error::Geometry(MachineError::VirtualHoming(e)))?;
        self.trace_mixed_at_virtual_origin(start, bank, input)
    }

    /// CLI variant with explicitly supplied virtual coordinates instead of
    /// a simulated homing proof. Not a printer session.
    pub fn trace_mixed_at_virtual_origin<'a>(
        &self,
        start: [f64; 3],
        bank: &commands::Bank<'a>,
        input: Input<'_>,
    ) -> Result<MixedTrace<'a>, Error> {
        if !(1.0..=1e9).contains(&input.hz) {
            return Err(Error::Geometry(MachineError::Clock(
                crate::step_clock::Error::InvalidTime,
            )));
        }
        if !self.in_range(&start) {
            return Err(Error::Geometry(MachineError::InitialPosition));
        }
        let geometry = Error::Geometry;
        let prepare_error = |error| geometry(MachineError::Gcode { line: 0, error });
        let mut preview = Preview::new_at(
            self.limits,
            4096,
            input
                .extruder
                .as_ref()
                .map(|e| e.geometry.corner_velocity()),
            [start[0], start[1], start[2], 0.],
        )
        .map_err(prepare_error)?;
        if let Some(mesh) = &self.fixed_mesh {
            preview
                .install_bed_mesh(bed_mesh::Transform::clone(mesh))
                .map_err(prepare_error)?;
        }
        if let Some(resolution) = self.arc_resolution {
            preview.enable_arcs(resolution).map_err(prepare_error)?;
        }
        let mut no_lane = false;
        let mut bad_extrusion = None;
        let preflight = planned::preflight(preview, bank, input.script, |_, previous, next| {
            let end = next.end_position();
            if !self.in_range(&end[..3]) {
                return Err(gcode::Error::MoveOutsidePreview);
            }
            if let Some(z) = next.z_ratio() {
                next.limit(z * self.z_velocity, z * self.z_accel)?;
            }
            if end[3] != previous[3] {
                let Some(ex) = &input.extruder else {
                    no_lane = true;
                    return Err(gcode::Error::MoveOutsidePreview);
                };
                ex.geometry
                    .check_move(next, previous, ex.temperature)
                    .map_err(|err| {
                        bad_extrusion = Some(err);
                        gcode::Error::MoveOutsidePreview
                    })?;
            }
            Ok(())
        })
        .map_err(|e| {
            if no_lane {
                Error::ExtruderRequired
            } else if let Some(err) = bad_extrusion {
                geometry(MachineError::Extrusion(err))
            } else {
                Error::Stage(e)
            }
        })?;
        let settings = PhaseSettings::new(start, input.hz, input.max_error, input.invert);
        let mut phases = Vec::new();
        let mut records = 0usize;
        let mut xyz_steps = [0i64; 3];
        let mut e_steps = 0i64;
        for stage in preflight.stages {
            match stage {
                planned::Stage::Heater {
                    line,
                    outcome,
                    targets,
                } => {
                    phases.push(Stage::Heater {
                        line,
                        outcome,
                        targets,
                    });
                }
                planned::Stage::Motion {
                    first_line,
                    origin: _,
                    outputs,
                } => {
                    let ext = input
                        .extruder
                        .map(|ex| (ex, e_steps as f64 * ex.step_distance));
                    let trace = self
                        .compile_mixed_phase(settings, outputs, &mut xyz_steps, ext)
                        .map_err(geometry)?;
                    if let (Some(ex), Some(encoded)) = (input.extruder, &trace.extruder) {
                        e_steps += signed_count(encoded, ex.inverse_direction);
                    }
                    count_records(&mut records, &trace)?;
                    phases.push(Stage::Motion { first_line, trace });
                }
            }
        }
        Ok(MixedTrace {
            hz: input.hz,
            phases,
            final_position: preflight.final_position,
        })
    }
    /// Phase-local clocks for explicitly mapped virtual tools. The selector
    /// changes only the active lane and unqualified heater index; it is not a
    /// claim of physical docking, probing or Toolchanger macro execution.
    pub fn trace_mixed_tools_at_virtual_origin<'a>(
        &self,
        start: [f64; 3],
        bank: &commands::Bank<'a>,
        input: ToolInput<'_>,
    ) -> Result<ToolTrace<'a>, Error> {
        if !(1.0..=1e9).contains(&input.hz) {
            return Err(Error::Geometry(MachineError::Clock(
                crate::step_clock::Error::InvalidTime,
            )));
        }
        if !self.in_range(&start) {
            return Err(Error::Geometry(MachineError::InitialPosition));
        }
        let mut preview =
            Preview::new_at(self.limits, 4096, None, [start[0], start[1], start[2], 0.])
                .map_err(|error| Error::Geometry(MachineError::Gcode { line: 0, error }))?;
        if let Some(mesh) = &self.fixed_mesh {
            preview
                .install_bed_mesh(mesh.clone())
                .map_err(|error| Error::Geometry(MachineError::Gcode { line: 0, error }))?;
        }
        if let Some(resolution) = self.arc_resolution {
            preview
                .enable_arcs(resolution)
                .map_err(|error| Error::Geometry(MachineError::Gcode { line: 0, error }))?;
        }
        let slots = input.slots;
        if !(1..=8).contains(&slots.len())
            || slots.iter().any(|s| {
                !s.lane.extruder.temperature.is_finite()
                    || !s.lane.extruder.step_distance.is_finite()
                    || s.lane.extruder.step_distance <= 0.
            })
        {
            return Err(Error::Tools(planned::ToolError::Mapping));
        }
        let position = |id| {
            slots
                .iter()
                .position(|s| s.lane.id == id)
                .ok_or(Error::Tools(planned::ToolError::Mapping))
        };
        let tools: Vec<_> = slots
            .iter()
            .map(|s| planned::Tool {
                id: s.lane.id,
                heater: s.heater,
                parked_e: s.lane.parked_e,
                corner_velocity: s.lane.extruder.geometry.corner_velocity(),
            })
            .collect();
        let mut bad_extrusion = None;
        let plan = planned::preflight_tools(
            preview,
            bank,
            &tools,
            input.initial_tool,
            input.script,
            |tool, _, before, after| {
                let end = after.end_position();
                if !self.in_range(&end[..3]) {
                    return Err(gcode::Error::MoveOutsidePreview);
                }
                if let Some(z) = after.z_ratio() {
                    after.limit(z * self.z_velocity, z * self.z_accel)?;
                }
                if before[3] != end[3] {
                    let ext = &slots
                        [position(tool).map_err(|_| gcode::Error::MoveOutsidePreview)?]
                    .lane
                    .extruder;
                    ext.geometry
                        .check_move(after, before, ext.temperature)
                        .map_err(|error| {
                            bad_extrusion = Some(error);
                            gcode::Error::MoveOutsidePreview
                        })?;
                }
                Ok(())
            },
        )
        .map_err(|error| {
            bad_extrusion.map_or_else(
                || Error::Tools(error),
                |e| Error::Geometry(MachineError::Extrusion(e)),
            )
        })?;
        let settings = PhaseSettings::new(start, input.hz, input.max_error, input.invert);
        let mut xyz_steps = [0i64; 3];
        let mut e_steps = [0i64; 8];
        let mut records = 0;
        let mut phases = Vec::with_capacity(plan.stages.len());
        for phase in plan.stages {
            phases.push(match phase {
                planned::ToolStage::Motion {
                    tool,
                    first_line,
                    outputs,
                    ..
                } => {
                    let index = position(tool)?;
                    let slot = slots[index].lane;
                    let ext = slot.extruder;
                    let seed = slot.parked_e + e_steps[index] as f64 * ext.step_distance;
                    let trace = self
                        .compile_mixed_phase(settings, outputs, &mut xyz_steps, Some((ext, seed)))
                        .map_err(Error::Geometry)?;
                    if let Some(encoded) = &trace.extruder {
                        e_steps[index] += signed_count(encoded, ext.inverse_direction);
                    }
                    count_records(&mut records, &trace)?;
                    ToolStage::Motion {
                        first_line,
                        tool,
                        trace,
                    }
                }
                planned::ToolStage::Heater {
                    active_tool,
                    line,
                    outcome,
                    targets,
                } => ToolStage::Heater {
                    active_tool,
                    line,
                    outcome,
                    targets,
                },
                planned::ToolStage::Fan {
                    line,
                    active_tool,
                    demand,
                } => {
                    if !input.print_fan {
                        return Err(Error::FanNotConfigured { line });
                    }
                    ToolStage::Fan {
                        line,
                        active_tool,
                        demand,
                    }
                }
                planned::ToolStage::Select { line, from, to } => {
                    ToolStage::Select { line, from, to }
                }
            });
        }
        Ok(ToolTrace {
            hz: input.hz,
            initial_tool: input.initial_tool,
            phases,
            final_position: plan.final_position,
            final_extruders: plan.final_extruders,
        })
    }

    fn compile_mixed_phase(
        &self,
        settings: PhaseSettings,
        outputs: Vec<gcode::Output>,
        xyz_steps: &mut [i64; 3],
        ext: Option<(SimulatedExtruder<'_>, f64)>,
    ) -> Result<Trace, MachineError> {
        let mut xyz = Trajectory::new(16_384).map_err(MachineError::Trajectory)?;
        let mut e = ext
            .map(|_| Trajectory::new(16_384))
            .transpose()
            .map_err(MachineError::Trajectory)?;
        let mut time = LEAD;
        for output in outputs {
            for m in output.moves {
                if let Some(track) = &mut e {
                    extrusion::append_nominal(track, time, &m).map_err(MachineError::Trajectory)?;
                }
                append_move(&mut xyz, &mut time, &m).map_err(MachineError::Trajectory)?;
            }
            time += output.dwell_seconds.unwrap_or(0.);
            if !time.is_finite() || time >= 1e12 {
                return Err(MachineError::Trajectory(trajectory::Error::Invalid));
            }
        }
        let PhaseSettings {
            start,
            clock,
            max_error,
            invert,
        } = settings;
        let physical = std::array::from_fn::<_, 3, _>(|i| {
            self.motor_axis[i].position(start) + xyz_steps[i] as f64 * self.motor_step[i]
        });
        let from_xyz = if matches!(self.motor_axis[0], Axis::CoreXYPlus) {
            [
                (physical[0] + physical[1]) * 0.5,
                (physical[0] - physical[1]) * 0.5,
                physical[2],
            ]
        } else {
            physical
        };
        let traced = self.lane_traces(
            &xyz,
            vec![],
            &[],
            &[],
            from_xyz,
            clock,
            max_error,
            invert,
            Vec::new(),
            time - LEAD,
        )?;
        for (i, lane) in traced.motors.iter().enumerate() {
            xyz_steps[i] += signed_count(lane, invert[i]);
        }
        let extruder = if let (Some((ex, seed)), Some(e)) = (ext, e) {
            let encoded = if let Some(advance) = ex.pressure_advance {
                let events = advance
                    .crossings_from(&e, ex.step_distance, seed)
                    .map_err(MachineError::Solver)?;
                crate::step_clock::encode_trace(clock, &events, max_error, ex.inverse_direction)
                    .map_err(MachineError::Clock)?
            } else {
                step_lane(
                    &e,
                    Axis::X,
                    ex.step_distance,
                    [seed, 0., 0.],
                    clock,
                    max_error,
                    ex.inverse_direction,
                )?
            };
            Some(encoded)
        } else {
            None
        };
        Ok(Trace {
            motors: traced.motors,
            extruder,
            duration: traced.duration,
        })
    }
}

impl PhaseSettings {
    fn new(start: [f64; 3], hz: f64, max_error: u32, invert: [bool; 3]) -> Self {
        Self {
            start,
            clock: Clock {
                last_encoded: 0,
                time_origin: 0.,
                hz,
            },
            max_error,
            invert,
        }
    }
}

fn count_records(records: &mut usize, trace: &Trace) -> Result<(), Error> {
    *records += trace.motors.iter().map(Vec::len).sum::<usize>()
        + trace.extruder.as_ref().map_or(0, Vec::len);
    if *records > 196_000 {
        return Err(Error::Stage(planned::Error::Limit));
    }
    Ok(())
}

fn signed_count(lane: &[Command], invert: bool) -> i64 {
    let mut positive = false;
    let mut count = 0;
    for cmd in lane {
        match cmd {
            Command::SetDirection(bit) => positive = *bit ^ invert,
            Command::Steps(block) => {
                count += i64::from(block.count) * if positive { 1 } else { -1 }
            }
        }
    }
    count
}
