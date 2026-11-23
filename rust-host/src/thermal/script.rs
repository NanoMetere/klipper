// SPDX-License-Identifier: GPL-3.0-only
//! Segment a bounded mixed file without issuing, timing, or applying commands.
//! A host must hold the following motion slice behind any actual thermal wait.

use super::commands::{parse_gcode, Command, Error};

const MAX_BYTES: usize = 8 * 1024 * 1024;
const MAX_CHECKPOINTS: usize = 512;
const MAX_TOOL_SELECTORS: usize = 127;

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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ToolChunk<'a> {
    Chunk(Chunk<'a>),
    Select { line: usize, tool: u8 },
    Fan { line: usize, demand: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolSplitError {
    Mixed(SplitError),
    Selector(crate::toolchange::SegmentError),
    Fan {
        line: usize,
        error: crate::fan::Error,
    },
    Bounds,
}

/// Preserve source order for heater commands and bare tool selectors.
/// This variant neither resolves tool IDs nor runs tool macros. An explicit
/// heater target such as `M104 T5` never becomes a tool selector.
pub fn split_tool_mixed(script: &str) -> Result<Vec<ToolChunk<'_>>, ToolSplitError> {
    let mut selectors = 0;
    let mut ordered = Vec::new();
    for chunk in split_mixed(script).map_err(ToolSplitError::Mixed)? {
        let Chunk::Motion { first_line, text } = chunk else {
            ordered.push(ToolChunk::Chunk(chunk));
            continue;
        };
        let (mut start, mut offset, mut first) = (0, 0, first_line);
        for (i, raw) in text.split_inclusive('\n').enumerate() {
            let line = first_line + i;
            let end = offset + raw.len();
            let selector =
                crate::toolchange::tool_selector(raw, line).map_err(ToolSplitError::Selector)?;
            let stage = if let Some(tool) = selector {
                selectors += 1;
                if selectors > MAX_TOOL_SELECTORS {
                    return Err(ToolSplitError::Bounds);
                }
                Some(ToolChunk::Select { line, tool })
            } else {
                crate::fan::command::print_demand(raw)
                    .map_err(|error| ToolSplitError::Fan { line, error })?
                    .map(|demand| ToolChunk::Fan { line, demand })
            };
            if let Some(stage) = stage {
                if ordered.len() >= MAX_CHECKPOINTS * 2 + 256 {
                    return Err(ToolSplitError::Bounds);
                }
                if start < offset {
                    ordered.push(ToolChunk::Chunk(Chunk::Motion {
                        first_line: first,
                        text: &text[start..offset],
                    }));
                }
                ordered.push(stage);
                start = end;
                first = line + 1;
            }
            offset = end;
        }
        if start < text.len() {
            ordered.push(ToolChunk::Chunk(Chunk::Motion {
                first_line: first,
                text: &text[start..],
            }));
        }
    }
    if ordered.len() > MAX_CHECKPOINTS * 2 + 257 {
        return Err(ToolSplitError::Bounds);
    }
    Ok(ordered)
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

    #[test]
    fn keeps_sparse_switches_and_explicit_heater_selection_separate() {
        let raw = "G90 ; comment\r\nT2\r\nM104 T5 S190\r\nG1 X15\r\nT5\r\nM109 S215\r\nG1 E.2";
        let result = split_tool_mixed(raw).unwrap();
        assert!(matches!(
            result.as_slice(),
            [
                ToolChunk::Chunk(Chunk::Motion {
                    first_line: 1,
                    text: "G90 ; comment\r\n"
                }),
                ToolChunk::Select { line: 2, tool: 2 },
                ToolChunk::Chunk(Chunk::Heater {
                    line: 3,
                    command: Command::Set {
                        heater: Heater::Extruder(Some(5)),
                        target: 190.,
                        wait: false
                    }
                }),
                ToolChunk::Chunk(Chunk::Motion {
                    first_line: 4,
                    text: "G1 X15\r\n"
                }),
                ToolChunk::Select { line: 5, tool: 5 },
                ToolChunk::Chunk(Chunk::Heater {
                    line: 6,
                    command: Command::Set {
                        heater: Heater::Extruder(None),
                        target: 215.,
                        wait: true
                    }
                }),
                ToolChunk::Chunk(Chunk::Motion {
                    first_line: 7,
                    text: "G1 E.2"
                }),
            ]
        ));
        assert!(matches!(
            split_tool_mixed("M109 S200\nG90\nT02"),
            Err(ToolSplitError::Selector(
                crate::toolchange::SegmentError::BadTool { line: 3 }
            ))
        ));
        assert_eq!(
            split_tool_mixed(&"T5\n".repeat(128)),
            Err(ToolSplitError::Bounds)
        );
    }
}
