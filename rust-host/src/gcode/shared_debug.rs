// SPDX-License-Identifier: GPL-3.0-only
//! Strict extended commands for the bounded shared extrusion debug planner.

use super::{extended, Error, MAX_LINE};

#[derive(Clone, Copy, Debug)]
pub(crate) enum SharedDebugControl {
    Sync(Option<u8>),
    Activate(u8),
    Advance(f64),
}

pub(crate) fn shared_debug_control(line: &str) -> Result<Option<SharedDebugControl>, Error> {
    let end = line.find(char::is_whitespace).unwrap_or(line.len());
    let command = &line[..end];
    let tag = match command.to_ascii_uppercase().as_str() {
        "SYNC_EXTRUDER_MOTION" => 0,
        "ACTIVATE_EXTRUDER" => 1,
        "SET_PRESSURE_ADVANCE" => 2,
        _ => return Ok(None),
    };
    if line.len() > MAX_LINE || line.contains(['\n', '\r', '\0']) {
        return Err(Error::InvalidLine);
    }
    let args = extended::tokenize(&line[end..])?;
    let extruder = args.get("EXTRUDER").ok_or(Error::InvalidParameter)?;
    let queue = |name: &str| match name {
        "extruder" => Ok(0),
        "extruder1" => Ok(1),
        _ => Err(Error::InvalidParameter),
    };
    let parsed = match tag {
        0 if args.len() == 2 && extruder == "shared_extruder" => {
            let q = args.get("MOTION_QUEUE").ok_or(Error::InvalidParameter)?;
            SharedDebugControl::Sync(if q.is_empty() { None } else { Some(queue(q)?) })
        }
        1 if args.len() == 1 => SharedDebugControl::Activate(queue(extruder)?),
        2 if args.len() == 2 && extruder == "shared_extruder" => {
            let a = extended::float(&args, "ADVANCE")?.ok_or(Error::InvalidParameter)?;
            if a <= 0.0 {
                return Err(Error::InvalidParameter);
            }
            SharedDebugControl::Advance(a)
        }
        _ => return Err(Error::InvalidParameter),
    };
    Ok(Some(parsed))
}
