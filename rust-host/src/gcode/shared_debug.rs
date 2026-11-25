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

// The pickup surface alone permits zero gain on the named shared stepper.
#[cfg(feature = "runtime-pickup-preview")]
pub(crate) fn pickup_control(line: &str) -> Result<Option<SharedDebugControl>, Error> {
    let end = line.find(char::is_whitespace).unwrap_or(line.len());
    if !line[..end].eq_ignore_ascii_case("SET_PRESSURE_ADVANCE") {
        return shared_debug_control(line);
    }
    if line.len() > MAX_LINE || line.contains(['\n', '\r', '\0']) {
        return Err(Error::InvalidLine);
    }
    let args = extended::tokenize(&line[end..])?;
    if args.len() != 2 || args.get("EXTRUDER").is_none_or(|e| e != "shared_extruder") {
        return Err(Error::InvalidParameter);
    }
    let gain = extended::float(&args, "ADVANCE")?.ok_or(Error::InvalidParameter)?;
    if gain < 0.0 {
        return Err(Error::InvalidParameter);
    }
    Ok(Some(SharedDebugControl::Advance(gain)))
}

// Opt-in offline runtime updates retain omitted settings, just as stock does.
#[cfg(feature = "runtime-pickup-pressure-preview")]
pub(crate) fn pickup_pressure_setting(
    line: &str,
    previous: (f64, f64),
) -> Result<Option<(f64, f64)>, Error> {
    let end = line.find(char::is_whitespace).unwrap_or(line.len());
    if !line[..end].eq_ignore_ascii_case("SET_PRESSURE_ADVANCE") {
        return Ok(None);
    }
    if line.len() > MAX_LINE || line.contains(['\n', '\r', '\0']) {
        return Err(Error::InvalidLine);
    }
    let args = extended::tokenize(&line[end..])?;
    if args.get("EXTRUDER").is_none_or(|e| e != "shared_extruder")
        || args
            .keys()
            .any(|k| !matches!(k.as_str(), "EXTRUDER" | "ADVANCE" | "SMOOTH_TIME"))
    {
        return Err(Error::InvalidParameter);
    }
    let gain = extended::float(&args, "ADVANCE")?.unwrap_or(previous.0);
    let smooth = extended::float(&args, "SMOOTH_TIME")?.unwrap_or(previous.1);
    if gain < 0. || !(smooth == 0. || (1e-6..=0.2).contains(&smooth)) {
        return Err(Error::InvalidParameter);
    }
    Ok(Some((gain, smooth)))
}
