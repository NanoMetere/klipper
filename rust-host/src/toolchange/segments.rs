// SPDX-License-Identifier: GPL-3.0-only
//! Strict, zero-copy T-command parsing for offline segmented previews only.

use crate::machine_preview::ToolSegment;

const MAX_SEGMENTS: usize = 128;
const MAX_BYTES: usize = 8 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    BadTool { line: usize },
    Limit,
}

/// Split bare T0..T255 lines into slices. Temperature parameters such as
/// M104 T1 are not switches. This neither looks up a tool nor performs
/// docking or T-command macro execution.
pub fn split(script: &str, initial: u8) -> Result<Vec<ToolSegment<'_>>, Error> {
    if script.len() > MAX_BYTES {
        return Err(Error::Limit);
    }
    let mut parts = Vec::new();
    let mut tool = initial;
    let mut start = 0;
    let mut offset = 0;
    for (index, line) in script.split_inclusive('\n').enumerate() {
        let end = offset + line.len();
        let head = line.split(';').next().unwrap().trim();
        if matches!(head.as_bytes().first(), Some(b't' | b'T'))
            && head.as_bytes().get(1).is_some_and(u8::is_ascii_digit)
        {
            let digits = &head[1..];
            let next = digits
                .parse::<u8>()
                .ok()
                .filter(|_| {
                    digits.bytes().all(|v| v.is_ascii_digit())
                        && !(digits.len() > 1 && digits.starts_with('0'))
                })
                .ok_or(Error::BadTool { line: index + 1 })?;
            push(
                &mut parts,
                ToolSegment {
                    tool,
                    gcode: &script[start..offset],
                },
            )?;
            start = end;
            tool = next;
        }
        offset = end;
    }
    push(
        &mut parts,
        ToolSegment {
            tool,
            gcode: &script[start..],
        },
    )?;
    Ok(parts)
}

fn push<'a>(parts: &mut Vec<ToolSegment<'a>>, part: ToolSegment<'a>) -> Result<(), Error> {
    if parts.len() == MAX_SEGMENTS {
        return Err(Error::Limit);
    }
    parts.push(part);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leaves_non_switch_lines_in_place_and_drops_only_bare_tool_lines() {
        let parts = split("M104 T3 S195\n  t1 ; 한글\r\nG90\nT1\nG1 X3\nT0\nG1 E.1", 0).unwrap();
        assert_eq!(
            parts.iter().map(|p| (p.tool, p.gcode)).collect::<Vec<_>>(),
            [
                (0, "M104 T3 S195\n"),
                (1, "G90\n"),
                (1, "G1 X3\n"),
                (0, "G1 E.1")
            ]
        );
        assert!(matches!(
            split("T004\n", 0),
            Err(Error::BadTool { line: 1 })
        ));
        assert!(matches!(
            split("G90\nT3 RESTORE_AXIS=XY", 0),
            Err(Error::BadTool { line: 2 })
        ));
        assert!(matches!(
            split(&"G90\nT1\n".repeat(130), 0),
            Err(Error::Limit)
        ));
    }
}
