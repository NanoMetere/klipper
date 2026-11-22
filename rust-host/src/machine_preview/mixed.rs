// SPDX-License-Identifier: GPL-3.0-only
//! Offline motion traces isolated at fixed heater boundaries. The leading
//! ticks and local clocks are for simulations, not a firmware reset recipe.

use super::{
    append_move, bed_mesh, extrusion, gcode, step_lane, trajectory, Axis, Clock, Command,
    Error as MachineError, Machine, Preview, SimulatedExtruder, Trace, Trajectory,
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
        if !(1.0..=1e9).contains(&input.hz) {
            return Err(Error::Geometry(MachineError::Clock(
                crate::step_clock::Error::InvalidTime,
            )));
        }
        let start = home
            .ready_position_for(self.homing_kinematics(), self.motor_step)
            .map_err(|e| Error::Geometry(MachineError::VirtualHoming(e)))?;
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
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.,
            hz: input.hz,
        };
        let mut phases = Vec::new();
        let mut records = 0usize;
        let mut stepped = [0i64; 4];
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
                    let track = Trajectory::new(16_384)
                        .map_err(|e| geometry(MachineError::Trajectory(e)))?;
                    let mut xyz = track;
                    let mut e_queues = if input.extruder.is_some() {
                        vec![Trajectory::new(16_384)
                            .map_err(|e| geometry(MachineError::Trajectory(e)))?]
                    } else {
                        vec![]
                    };
                    let mut time = LEAD;
                    for output in outputs {
                        for m in output.moves {
                            if let Some(e) = e_queues.first_mut() {
                                extrusion::append_nominal(e, time, &m)
                                    .map_err(|e| geometry(MachineError::Trajectory(e)))?;
                            }
                            append_move(&mut xyz, &mut time, &m)
                                .map_err(|e| geometry(MachineError::Trajectory(e)))?;
                        }
                        time += output.dwell_seconds.unwrap_or(0.);
                        if !time.is_finite() || time >= 1e12 {
                            return Err(geometry(MachineError::Trajectory(
                                trajectory::Error::Invalid,
                            )));
                        }
                    }
                    let physical = std::array::from_fn::<_, 3, _>(|i| {
                        self.motor_axis[i].position(start) + stepped[i] as f64 * self.motor_step[i]
                    });
                    let for_xyz = if matches!(self.motor_axis[0], Axis::CoreXYPlus) {
                        [
                            (physical[0] + physical[1]) * 0.5,
                            (physical[0] - physical[1]) * 0.5,
                            physical[2],
                        ]
                    } else {
                        physical
                    };
                    let traced = self
                        .lane_traces(
                            &xyz,
                            vec![],
                            &[],
                            for_xyz,
                            clock,
                            input.max_error,
                            input.invert,
                            Vec::new(),
                            time - LEAD,
                        )
                        .map_err(geometry)?;
                    for (i, lane) in traced.motors.iter().enumerate() {
                        stepped[i] += signed_count(lane, input.invert[i]);
                    }
                    let extruder = if let (Some(ex), Some(e)) = (input.extruder, e_queues.first()) {
                        let seed = stepped[3] as f64 * ex.step_distance;
                        let encoded = if let Some(advance) = ex.pressure_advance {
                            let events = advance
                                .crossings_from(e, ex.step_distance, seed)
                                .map_err(|e| geometry(MachineError::Solver(e)))?;
                            crate::step_clock::encode_trace(
                                clock,
                                &events,
                                input.max_error,
                                ex.inverse_direction,
                            )
                            .map_err(|e| geometry(MachineError::Clock(e)))?
                        } else {
                            step_lane(
                                e,
                                Axis::X,
                                ex.step_distance,
                                [seed, 0., 0.],
                                clock,
                                input.max_error,
                                ex.inverse_direction,
                            )
                            .map_err(geometry)?
                        };
                        stepped[3] += signed_count(&encoded, ex.inverse_direction);
                        Some(encoded)
                    } else {
                        None
                    };
                    records += traced.motors.iter().map(Vec::len).sum::<usize>()
                        + extruder.as_ref().map_or(0, Vec::len);
                    if records > 196_000 {
                        return Err(Error::Stage(planned::Error::Limit));
                    }
                    phases.push(Stage::Motion {
                        first_line,
                        trace: Trace {
                            motors: traced.motors,
                            extruder,
                            duration: traced.duration,
                        },
                    });
                }
            }
        }
        Ok(MixedTrace {
            hz: input.hz,
            phases,
            final_position: preflight.final_position,
        })
    }
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
