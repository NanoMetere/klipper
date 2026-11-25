// SPDX-License-Identifier: GPL-3.0-only
//! Runtime limits for subsequent moves in the offline planner.
use super::{extended, parse, Error, Limits};

pub(super) fn change(limits: Limits, line: &str) -> Result<Option<Limits>, Error> {
    let line = line.trim_start_matches(|c: char| c.is_ascii_whitespace());
    match line.as_bytes().first().map(u8::to_ascii_uppercase) {
        Some(b'M') if line.as_bytes().get(1..4) == Some(b"204") => {
            let Some(words) = parse(line)? else {
                return Ok(None);
            };
            if words.code != 204 {
                return Ok(None);
            }
            words.allow(b"SPT")?;
            let positive = |v: Option<f64>| match v {
                Some(n) if n <= 0.0 => Err(Error::InvalidParameter),
                _ => Ok(v),
            };
            // Stock gives S precedence, otherwise uses the lower of P and T.
            let accel = match positive(words.get(b'S'))? {
                Some(s) => Some(s),
                None => positive(words.get(b'P'))?
                    .zip(positive(words.get(b'T'))?)
                    .map(|(p, t)| p.min(t)),
            };
            Ok(Some(limits.updated(None, accel, None, None)?))
        }
        Some(b'S') => {
            let headline = line
                .split_once(';')
                .map_or(line, |(prefix, _)| prefix)
                .trim_end();
            let end = headline.find(char::is_whitespace).unwrap_or(headline.len());
            if !headline[..end].eq_ignore_ascii_case("SET_VELOCITY_LIMIT") {
                return Ok(None);
            }
            if headline.contains(['\n', '\r', '\0']) {
                return Err(Error::InvalidLine);
            }
            let args = extended::tokenize(&headline[end..])?;
            let fields = [
                "VELOCITY",
                "ACCEL",
                "SQUARE_CORNER_VELOCITY",
                "MINIMUM_CRUISE_RATIO",
            ];
            if args.keys().any(|key| !fields.contains(&key.as_str())) {
                return Err(Error::InvalidParameter);
            }
            let [v, a, c, r] = fields.map(|key| extended::float(&args, key));
            Ok(Some(limits.updated(v?, a?, c?, r?)?))
        }
        _ => Ok(None),
    }
}
