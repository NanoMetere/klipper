// SPDX-License-Identifier: GPL-3.0-only
//! A non-device preflight of mixed heater/motion chunks. This deliberately
//! stops geometry at heater boundaries; consuming later profiles before a
//! wait is satisfied is outside this API.

use super::{
    commands,
    script::{split_mixed, Chunk, SplitError},
};
use crate::{
    gcode::{self, Output, Preview},
    lookahead::Move,
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
                let mut outputs = Vec::new();
                for (idx, line) in text.lines().enumerate() {
                    let n = first_line + idx;
                    let output = preview
                        .line_with(line, |before, after| verify_move(n, before, after))
                        .map_err(|error| Error::Motion { line: n, error })?;
                    profile_count += output.moves.len();
                    if profile_count > MAX_PROFILES {
                        return Err(Error::Limit);
                    }
                    if !output.moves.is_empty() || output.dwell_seconds.is_some() {
                        outputs.push(output);
                    }
                }
                let moves = preview.finish().map_err(|error| Error::Motion {
                    line: first_line + text.lines().count(),
                    error,
                })?;
                profile_count += moves.len();
                if profile_count > MAX_PROFILES {
                    return Err(Error::Limit);
                }
                if !moves.is_empty() {
                    outputs.push(Output {
                        moves,
                        dwell_seconds: None,
                    });
                }
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
}
