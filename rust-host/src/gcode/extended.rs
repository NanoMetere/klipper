// SPDX-License-Identifier: GPL-3.0-only
//! Coordinate-only, named commands adapted from Klipper extras/gcode_move.py.

use super::{Effect, Error, State, MAX_LINE};
use std::{collections::BTreeMap, iter::Peekable, str::Chars};

const MAX_STATES: usize = 128;
const MAX_NAME_BYTES: usize = 128;
const AXES: [(&str, &str); 4] = [
    ("X", "X_ADJUST"),
    ("Y", "Y_ADJUST"),
    ("Z", "Z_ADJUST"),
    ("E", "E_ADJUST"),
];

pub(super) fn apply(
    next: &mut State,
    saved: &mut BTreeMap<String, State>,
    line: &str,
) -> Result<Option<Effect>, Error> {
    if line.len() > MAX_LINE {
        return Err(Error::InvalidLine);
    }
    let line = line.trim_start();
    if !matches!(line.as_bytes().first(), Some(b'S' | b's' | b'R' | b'r')) {
        return Ok(None);
    }
    let headline = line
        .split_once(';')
        .map_or(line, |(prefix, _)| prefix)
        .trim();
    let cmd_end = headline.find(char::is_whitespace).unwrap_or(headline.len());
    let cmd = &headline[..cmd_end];
    let (save, restore, offsets) = (
        cmd.eq_ignore_ascii_case("SAVE_GCODE_STATE"),
        cmd.eq_ignore_ascii_case("RESTORE_GCODE_STATE"),
        cmd.eq_ignore_ascii_case("SET_GCODE_OFFSET"),
    );
    if !save && !restore && !offsets {
        return Ok(None);
    }
    if headline.contains(['\n', '\r', '\0']) {
        return Err(Error::InvalidLine);
    }
    let args = tokenize(&headline[cmd_end..])?;
    let outcome = if offsets {
        for k in args.keys() {
            if k != "MOVE"
                && k != "MOVE_SPEED"
                && !AXES.iter().any(|(axis, adjust)| k == axis || k == adjust)
            {
                return Err(Error::InvalidParameter);
            }
        }
        let mut delta = [0.0; 4];
        for (i, (axis, adjust)) in AXES.into_iter().enumerate() {
            let offset = if let Some(abs) = float(&args, axis)? {
                abs
            } else if let Some(adj) = float(&args, adjust)? {
                next.homing[i] + adj
            } else {
                continue;
            };
            delta[i] = offset - next.homing[i];
            next.base[i] += delta[i];
            next.homing[i] = offset;
        }
        if requested_move(&args)? {
            for (v, step) in next.last.iter_mut().zip(delta) {
                *v += step;
            }
            movement(next, &args)?
        } else {
            Effect::None
        }
    } else {
        if args
            .keys()
            .any(|k| k != "NAME" && (save || (k != "MOVE" && k != "MOVE_SPEED")))
        {
            return Err(Error::InvalidParameter);
        }
        let name = args.get("NAME").map_or("default", String::as_str);
        if name.is_empty() || name.len() > MAX_NAME_BYTES {
            return Err(Error::InvalidParameter);
        }
        if save {
            if saved.len() >= MAX_STATES && !saved.contains_key(name) {
                return Err(Error::StateLimit);
            }
            saved.insert(name.to_owned(), *next);
            Effect::None
        } else {
            let previous = saved.get(name).ok_or(Error::UnknownState)?;
            let current = next.last;
            // Restore modes and E's relative position without moving E.
            *next = *previous;
            next.last = current;
            next.base[3] += current[3] - previous.last[3];
            if requested_move(&args)? {
                next.last[..3].copy_from_slice(&previous.last[..3]);
                movement(next, &args)?
            } else {
                Effect::None
            }
        }
    };
    next.validate()?;
    Ok(Some(outcome))
}

fn movement(next: &State, args: &BTreeMap<String, String>) -> Result<Effect, Error> {
    let speed = float(args, "MOVE_SPEED")?.unwrap_or(next.speed);
    if speed <= 0.0 {
        return Err(Error::InvalidParameter);
    }
    Ok(Effect::Linear {
        position: next.last,
        speed,
    })
}

fn requested_move(args: &BTreeMap<String, String>) -> Result<bool, Error> {
    args.get("MOVE").map_or(Ok(false), |v| {
        v.parse::<i64>()
            .map(|n| n != 0)
            .map_err(|_| Error::InvalidParameter)
    })
}

pub(super) fn float(args: &BTreeMap<String, String>, key: &str) -> Result<Option<f64>, Error> {
    args.get(key)
        .map(|v| {
            let f: f64 = v.parse().map_err(|_| Error::InvalidParameter)?;
            if f.is_finite() {
                Ok(f)
            } else {
                Err(Error::Nonfinite)
            }
        })
        .transpose()
}

pub(super) fn tokenize(text: &str) -> Result<BTreeMap<String, String>, Error> {
    let mut args = BTreeMap::new();
    let mut chars = text.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else if c == '#' {
            break;
        } else {
            let token = word(&mut chars)?;
            let (key, val) = token.split_once('=').ok_or(Error::InvalidParameter)?;
            if key.is_empty() || !key.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_') {
                return Err(Error::InvalidParameter);
            }
            if args
                .insert(key.to_ascii_uppercase(), val.to_owned())
                .is_some()
            {
                return Err(Error::DuplicateParameter);
            }
        }
    }
    Ok(args)
}

fn word(chars: &mut Peekable<Chars<'_>>) -> Result<String, Error> {
    let mut token = String::new();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() || c == '#' {
            break;
        }
        chars.next();
        if c == '\\' {
            token.push(chars.next().ok_or(Error::InvalidParameter)?);
        } else if c == '\'' || c == '"' {
            loop {
                let ch = chars.next().ok_or(Error::InvalidParameter)?;
                if ch == c {
                    break;
                }
                if ch == '\\' && c == '"' {
                    let escaped = chars.next().ok_or(Error::InvalidParameter)?;
                    if escaped != '"' && escaped != '\\' {
                        token.push(ch);
                    }
                    token.push(escaped);
                } else {
                    token.push(ch);
                }
            }
        } else {
            token.push(c);
        }
    }
    Ok(token)
}
