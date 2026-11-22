// SPDX-License-Identifier: GPL-3.0-only
//! Segment a bounded mixed file without issuing, timing, or applying commands.
//! A host must hold the following motion slice behind any actual thermal wait.

use super::commands::{parse_gcode, Command, Error};

const MAX_BYTES: usize = 8 * 1024 * 1024;
const MAX_CHECKPOINTS: usize = 512;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Chunk<'a> {
    /// Unmodified text for separate motion planning; line numbers are 1-based.
    Motion { first_line: usize, text: &'a str },
    /// Action at the boundary after the previous motion chunk.
    Heater { line: usize, command: Command<'a> },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SplitError {
    Bounds,
    Heater { line: usize, error: Error },
}

/// Extract explicit heater boundaries. A waiting command's following motion
/// must not be submitted as look-ahead for an earlier chunk. Unknown or other
/// G-code is preserved verbatim for separate validation. No progress is
/// reported on an invalid heater line or an oversized plan.
pub fn split_mixed(script: &str) -> Result<Vec<Chunk<'_>>, SplitError> {
    if script.len() > MAX_BYTES {
        return Err(SplitError::Bounds);
    }
    let mut chunks = Vec::new();
    let (mut advance, mut start, mut first_line) = (0, 0, 1);
    let mut heat = 0;
    for (index, raw) in script.split_inclusive('\n').enumerate() {
        let line = index + 1;
        if let Some(command) =
            parse_gcode(raw).map_err(|error| SplitError::Heater { line, error })?
        {
            heat += 1;
            if heat > MAX_CHECKPOINTS {
                return Err(SplitError::Bounds);
            }
            if start < advance {
                chunks.push(Chunk::Motion {
                    first_line,
                    text: &script[start..advance],
                });
            }
            chunks.push(Chunk::Heater { line, command });
            start = advance + raw.len();
            first_line = line + 1;
        }
        advance += raw.len();
    }
    if start < script.len() {
        chunks.push(Chunk::Motion {
            first_line,
            text: &script[start..],
        });
    }
    Ok(chunks)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::thermal::commands::Heater;

    #[test]
    fn returns_borrowed_safe_boundaries_and_preserves_nonheater_text() {
        let source = "G90 ; 시작\r\nT2\r\nM104 T1 S190 ; no wait\r\nM109 S210\r\nG1 X15 F800\nTURN_OFF_HEATERS";
        assert_eq!(
            split_mixed(source).unwrap(),
            [
                Chunk::Motion {
                    first_line: 1,
                    text: "G90 ; 시작\r\nT2\r\n"
                },
                Chunk::Heater {
                    line: 3,
                    command: Command::Set {
                        heater: Heater::Extruder(Some(1)),
                        target: 190.,
                        wait: false
                    }
                },
                Chunk::Heater {
                    line: 4,
                    command: Command::Set {
                        heater: Heater::Extruder(None),
                        target: 210.,
                        wait: true
                    }
                },
                Chunk::Motion {
                    first_line: 5,
                    text: "G1 X15 F800\n"
                },
                Chunk::Heater {
                    line: 6,
                    command: Command::Off
                },
            ]
        );
        assert_eq!(
            split_mixed("G90\nM109 S200\nG1 X20\nM140 T1"),
            Err(SplitError::Heater {
                line: 4,
                error: Error::Syntax
            })
        );
        assert_eq!(
            split_mixed(&"M109 S180\n".repeat(MAX_CHECKPOINTS + 1)),
            Err(SplitError::Bounds)
        );
        let nonheater = format!("CUSTOM_MOTION ARG={}", "x".repeat(9000));
        assert_eq!(
            split_mixed(&nonheater).unwrap(),
            [Chunk::Motion {
                first_line: 1,
                text: &nonheater
            }]
        );
    }
}
