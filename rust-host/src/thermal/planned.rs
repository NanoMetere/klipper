// SPDX-License-Identifier: GPL-3.0-only
//! A non-device preflight of mixed heater/motion chunks. This deliberately
//! stops geometry at heater boundaries; consuming later profiles before a
//! wait is satisfied is outside this API.

use super::{
    commands,
    script::{split_mixed, split_tool_mixed, Chunk, SplitError, ToolChunk, ToolSplitError},
};
use crate::{
    gcode::{self, Output, Preview},
    lookahead::Move,
    toolchange::ExtruderPositions,
};

const MAX_LINES: usize = 32_768;
const MAX_PROFILES: usize = 65_536;

#[derive(Debug)]
pub enum Error {
    Split(SplitError),
    Heater { line: usize, error: commands::Error },
    Motion { line: usize, error: gcode::Error },
    Limit,
}

pub enum Stage<'a> {
    Motion {
        first_line: usize,
        origin: [f64; 4],
        outputs: Vec<Output>,
    },
    Heater {
        line: usize,
        outcome: commands::Output<'a>,
        targets: Vec<f64>,
    },
}

pub struct Prepared<'a> {
    pub stages: Vec<Stage<'a>>,
    pub final_targets: commands::Bank<'a>,
    pub final_position: [f64; 4],
}

/// Plan geometry using one G-code coordinate and parser state across the
/// supplied chunks. Any error discards the whole result without modifying
/// the customer's original heater table. This uses an explicit M400-like
/// barrier even at nonblocking heater commands; it is a conservative
/// fixture, not a claim about exact stock heater/trajectory overlap.
pub fn preflight<'a>(
    mut preview: Preview,
    initial: &commands::Bank<'a>,
    script: &str,
    mut verify_move: impl FnMut(usize, [f64; 4], &mut Move) -> Result<(), gcode::Error>,
) -> Result<Prepared<'a>, Error> {
    if script.lines().take(MAX_LINES + 1).count() > MAX_LINES {
        return Err(Error::Limit);
    }
    let mut staged = initial.clone();
    let mut stages = Vec::new();
    let mut profile_count = 0;
    for part in split_mixed(script).map_err(Error::Split)? {
        match part {
            Chunk::Motion { first_line, text } => {
                let origin = preview.physical_position();
                let outputs = plan_motion(
                    &mut preview,
                    first_line,
                    text,
                    &mut profile_count,
                    &mut verify_move,
                )?;
                stages.push(Stage::Motion {
                    first_line,
                    origin,
                    outputs,
                });
            }
            Chunk::Heater { line, command } => {
                let outcome = staged
                    .command(command)
                    .map_err(|error| Error::Heater { line, error })?;
                stages.push(Stage::Heater {
                    line,
                    outcome,
                    targets: staged.targets().to_vec(),
                });
            }
        }
    }
    Ok(Prepared {
        final_position: preview.physical_position(),
        stages,
        final_targets: staged,
    })
}

fn plan_motion(
    preview: &mut Preview,
    first_line: usize,
    text: &str,
    profile_count: &mut usize,
    mut verify_move: impl FnMut(usize, [f64; 4], &mut Move) -> Result<(), gcode::Error>,
) -> Result<Vec<Output>, Error> {
    let mut outputs = Vec::new();
    let mut record = |output: Output| {
        *profile_count += output.moves.len();
        if *profile_count > MAX_PROFILES {
            return Err(Error::Limit);
        }
        if !output.moves.is_empty() || output.dwell_seconds.is_some() {
            outputs.push(output);
        }
        Ok(())
    };
    for (idx, line) in text.lines().enumerate() {
        let n = first_line + idx;
        let output = preview
            .line_with(line, |before, after| verify_move(n, before, after))
            .map_err(|error| Error::Motion { line: n, error })?;
        record(output)?;
    }
    let moves = preview.finish().map_err(|error| Error::Motion {
        line: first_line + text.lines().count(),
        error,
    })?;
    record(Output {
        moves,
        dwell_seconds: None,
    })?;
    Ok(outputs)
}

#[derive(Clone, Copy, Debug)]
pub struct Tool {
    pub id: u8,
    /// Stock heater role index, not necessarily the physical selector number.
    pub heater: u32,
    pub parked_e: f64,
    pub corner_velocity: f64,
}

#[derive(Debug)]
pub enum ToolError {
    Split(ToolSplitError),
    MotionOrHeater(Error),
    Mapping,
    Selection { line: usize, tool: u8 },
    Switch { line: usize, error: gcode::Error },
}

pub enum ToolStage<'a> {
    Motion {
        tool: u8,
        first_line: usize,
        origin: [f64; 4],
        outputs: Vec<Output>,
    },
    Heater {
        active_tool: u8,
        line: usize,
        outcome: commands::Output<'a>,
        targets: Vec<f64>,
    },
    Fan {
        line: usize,
        active_tool: u8,
        demand: f64,
    },
    /// Inert selection metadata; no dock, probe, transport or motion.
    Select { line: usize, from: u8, to: u8 },
}

pub struct ToolPrepared<'a> {
    pub stages: Vec<ToolStage<'a>>,
    pub final_targets: commands::Bank<'a>,
    pub final_position: [f64; 4],
    pub final_tool: u8,
    pub final_extruders: Vec<(u8, f64)>,
}

/// Preflight selected tools and heaters against one coordinate state. The
/// preview must be fresh: this contract cannot hide an earlier profile at
/// a heater or tool boundary. Modelled heater targets are not observations.
pub fn preflight_tools<'a>(
    mut preview: Preview,
    initial: &commands::Bank<'a>,
    tools: &[Tool],
    active: u8,
    script: &str,
    mut verify_move: impl FnMut(u8, usize, [f64; 4], &mut Move) -> Result<(), gcode::Error>,
) -> Result<ToolPrepared<'a>, ToolError> {
    if script.lines().take(MAX_LINES + 1).count() > MAX_LINES {
        return Err(ToolError::MotionOrHeater(Error::Limit));
    }
    let mut staged = initial.clone();
    let first = tools
        .iter()
        .find(|t| t.id == active)
        .ok_or(ToolError::Mapping)?;
    if !(1..=8).contains(&tools.len())
        || tools.iter().enumerate().any(|(i, tool)| {
            !tool.parked_e.is_finite()
                || !tool.corner_velocity.is_finite()
                || tool.corner_velocity < 0.
                || tools[..i]
                    .iter()
                    .any(|prior| prior.id == tool.id || prior.heater == tool.heater)
                || staged.activate(tool.heater).is_err()
        })
    {
        return Err(ToolError::Mapping);
    }
    staged
        .activate(first.heater)
        .map_err(|_| ToolError::Mapping)?;
    let slots: Vec<_> = tools.iter().map(|t| (t.id, t.parked_e)).collect();
    let mut positions = ExtruderPositions::new(&slots, active).map_err(|_| ToolError::Mapping)?;
    if !preview
        .activate_extruder_position(first.parked_e)
        .map_err(|error| ToolError::Switch { line: 0, error })?
        .moves
        .is_empty()
    {
        return Err(ToolError::Mapping);
    }
    preview
        .set_extruder_corner(first.corner_velocity)
        .map_err(|error| ToolError::Switch { line: 0, error })?;

    let mut profile_count = 0;
    let mut stages = Vec::new();
    for item in split_tool_mixed(script).map_err(ToolError::Split)? {
        let active_tool = positions.active();
        match item {
            ToolChunk::Chunk(Chunk::Motion { first_line, text }) => {
                let origin = preview.physical_position();
                let outputs = plan_motion(
                    &mut preview,
                    first_line,
                    text,
                    &mut profile_count,
                    |line, before, after| verify_move(active_tool, line, before, after),
                )
                .map_err(ToolError::MotionOrHeater)?;
                stages.push(ToolStage::Motion {
                    tool: active_tool,
                    first_line,
                    origin,
                    outputs,
                });
            }
            ToolChunk::Chunk(Chunk::Heater { line, command }) => {
                let outcome = staged
                    .command(command)
                    .map_err(|error| ToolError::MotionOrHeater(Error::Heater { line, error }))?;
                stages.push(ToolStage::Heater {
                    active_tool,
                    line,
                    outcome,
                    targets: staged.targets().to_vec(),
                });
            }
            ToolChunk::Fan { line, demand } => stages.push(ToolStage::Fan {
                line,
                active_tool,
                demand,
            }),
            ToolChunk::Select { line, tool } => {
                let next = tools
                    .iter()
                    .find(|selected| selected.id == tool)
                    .ok_or(ToolError::Selection { line, tool })?;
                if let Some((_, output)) =
                    positions
                        .switch(tool, &mut preview)
                        .map_err(|error| match error {
                            crate::toolchange::ExtruderPositionError::Gcode(error) => {
                                ToolError::Switch { line, error }
                            }
                            _ => ToolError::Selection { line, tool },
                        })?
                {
                    if !output.moves.is_empty() {
                        return Err(ToolError::Mapping);
                    }
                    staged
                        .activate(next.heater)
                        .map_err(|_| ToolError::Mapping)?;
                    preview
                        .set_extruder_corner(next.corner_velocity)
                        .map_err(|error| ToolError::Switch { line, error })?;
                    stages.push(ToolStage::Select {
                        line,
                        from: active_tool,
                        to: tool,
                    });
                }
            }
        }
    }
    let final_extruders = tools
        .iter()
        .map(|t| {
            (
                t.id,
                positions
                    .observed_position(t.id, &preview)
                    .expect("validated tool mapping"),
            )
        })
        .collect();
    Ok(ToolPrepared {
        stages,
        final_targets: staged,
        final_position: preview.physical_position(),
        final_tool: positions.active(),
        final_extruders,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        lookahead::Limits,
        thermal::commands::{Bank, Output as Thermal, Profile, Role, Wait},
    };

    #[test]
    fn stages_preserve_absolute_coordinates_and_drop_all_results_on_failure() {
        let shape = [Profile {
            name: "extruder",
            role: Role::Extruder(0),
            minimum: 0.,
            maximum: 280.,
        }];
        let bank = Bank::new(&shape, &[], Some(0), false).unwrap();
        let preview =
            || Preview::new(Limits::new(100., 1000., 5., 0.5).unwrap(), 100, Some(1.)).unwrap();
        let code = "G90\nM82\nG1 X10 E.2 F1200\nM109 S210\nG91\nM83\nG1 X.5 E.1\nG4 P120\nTURN_OFF_HEATERS\n";
        let mut lines = Vec::new();
        let result = preflight(preview(), &bank, code, |n, _, _| {
            lines.push(n);
            Ok(())
        })
        .unwrap();
        assert_eq!(lines, [3, 7]);
        let [Stage::Motion { outputs: a, .. }, Stage::Heater {
            line: 4,
            outcome:
                Thermal::Set {
                    wait: Some(Wait::Controller { target: 210., .. }),
                    ..
                },
            ..
        }, Stage::Motion { outputs: b, .. }, Stage::Heater {
            line: 9,
            outcome: Thermal::AllOff,
            ..
        }] = result.stages.as_slice()
        else {
            panic!("wrong heater boundaries");
        };
        assert_eq!(a.iter().flat_map(|p| &p.moves).last().unwrap().end[0], 10.);
        assert_eq!(b.iter().flat_map(|p| &p.moves).last().unwrap().end[0], 10.5);
        assert_eq!(b.iter().filter_map(|p| p.dwell_seconds).sum::<f64>(), 0.120);
        assert!((result.final_position[3] - 0.3).abs() < 1e-12);
        assert_eq!(result.final_targets.targets(), [0.]);
        for (code, number) in [
            ("G1 X10\nM109 S400\n", 2),
            ("G1 X10\nM109 S210\nBAD_TOOL\n", 3),
        ] {
            let error = match preflight(preview(), &bank, code, |_, _, _| Ok(())) {
                Err(e) => e,
                _ => panic!("partial plan escaped"),
            };
            assert!(matches!(error,Error::Heater{line,..}|Error::Motion{line,..} if line==number));
            assert_eq!(bank.targets(), [0.]);
        }
    }

    #[test]
    fn sparse_tool_ids_map_to_stock_heaters_and_keep_parked_e() {
        use crate::thermal::commands::{Bank, Output as Thermal, Profile, Role, Wait};
        let profiles = [
            Profile {
                name: "extruder",
                role: Role::Extruder(0),
                minimum: 0.,
                maximum: 280.,
            },
            Profile {
                name: "extruder1",
                role: Role::Extruder(1),
                minimum: 0.,
                maximum: 280.,
            },
        ];
        let initial = Bank::new(&profiles, &[], Some(0), false).unwrap();
        let tools = [
            Tool {
                id: 2,
                heater: 0,
                parked_e: 0.5,
                corner_velocity: 1.,
            },
            Tool {
                id: 5,
                heater: 1,
                parked_e: 8.,
                corner_velocity: 0.75,
            },
        ];
        let preview =
            || Preview::new(Limits::new(100., 1000., 5., 0.5).unwrap(), 100, None).unwrap();
        let code = "G91\nM83\nG1 X1 E.5 F1200\nM104 T1 S199\nT5\nM109 S205\nG1 X2 E.25\nT2\nM109 S200\nG1 X1 E.125";
        let mut moves = Vec::new();
        let prepared = preflight_tools(preview(), &initial, &tools, 2, code, |tool, line, _, _| {
            moves.push((tool, line));
            Ok(())
        })
        .unwrap();
        assert_eq!(moves, [(2, 3), (5, 7), (2, 10)]);
        assert_eq!(prepared.final_extruders, [(2, 1.125), (5, 8.25)]);
        assert_eq!(prepared.final_position, [4., 0., 0., 1.125]);
        assert_eq!(prepared.final_tool, 2);
        assert_eq!(prepared.final_targets.targets(), [200., 205.]);
        assert_eq!(initial.targets(), [0., 0.]);
        let effects: Vec<_> = prepared
            .stages
            .iter()
            .filter_map(|stage| match stage {
                ToolStage::Heater {
                    active_tool,
                    line,
                    outcome: Thermal::Set { index, wait, .. },
                    ..
                } => Some((*active_tool, *line, *index, *wait)),
                _ => None,
            })
            .collect();
        assert_eq!(
            effects,
            [
                (2, 4, 1, None),
                (
                    5,
                    6,
                    1,
                    Some(Wait::Controller {
                        index: 1,
                        target: 205.
                    })
                ),
                (
                    2,
                    9,
                    0,
                    Some(Wait::Controller {
                        index: 0,
                        target: 200.
                    })
                ),
            ]
        );
        assert!(matches!(
            preflight_tools(
                preview(),
                &initial,
                &tools,
                2,
                "G91\nM104 S190\nT9",
                |_, _, _, _| Ok(())
            ),
            Err(ToolError::Selection { line: 3, tool: 9 })
        ));
        let duplicate = [tools[0], Tool { id: 5, ..tools[0] }];
        assert!(matches!(
            preflight_tools(preview(), &initial, &duplicate, 2, "G90", |_, _, _, _| Ok(
                ()
            )),
            Err(ToolError::Mapping)
        ));
    }
}
