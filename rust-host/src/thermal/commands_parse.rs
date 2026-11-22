// SPDX-License-Identifier: GPL-3.0-only
//! Strict offline lexical subset for stock heater commands. This is not
//! Klippain's macro dispatcher; numeric T selects the stock heater index.

use super::commands::{Command, Error, Heater};

pub fn parse_gcode(line: &str) -> Result<Option<Command<'_>>, Error> {
    let text = line.split(';').next().unwrap_or(line).trim();
    if text.is_empty() {
        return Ok(None);
    }
    if text.len() > 8192 {
        return Err(Error::Syntax);
    }
    let data = text.as_bytes();
    if data[0].eq_ignore_ascii_case(&b'm') {
        let end = 1 + data[1..].iter().take_while(|v| v.is_ascii_digit()).count();
        let code = match &text[1..end] {
            "104" => 104,
            "109" => 109,
            "140" => 140,
            "190" => 190,
            _ => return Ok(None),
        };
        if data.get(end) == Some(&b'.') {
            // A renamed Klippy macro implementation is not this command.
            return Ok(None);
        }
        let mut target = None;
        let mut tool = None;
        let mut rest = &text[end..];
        while !rest.trim_start().is_empty() {
            rest = rest.trim_start();
            let key = rest.as_bytes()[0].to_ascii_uppercase();
            if !matches!(key, b'S' | b'T') {
                return Err(Error::Syntax);
            }
            let value = rest[1..].trim_start();
            let limit = value
                .as_bytes()
                .iter()
                .position(u8::is_ascii_alphabetic)
                .unwrap_or(value.len());
            let raw = value[..limit].trim();
            match key {
                b'S' if target.is_none() => target = Some(number(raw)?),
                b'T' if code < 140 && tool.is_none() => {
                    tool = Some(raw.parse::<u32>().map_err(|_| Error::Syntax)?)
                }
                _ => return Err(Error::Syntax),
            }
            rest = &value[limit..];
        }
        let heater = if code < 140 {
            Heater::Extruder(tool)
        } else {
            Heater::Bed
        };
        return Ok(Some(Command::Set {
            heater,
            target: target.unwrap_or(0.),
            wait: matches!(code, 109 | 190),
        }));
    }

    let split = text.find(char::is_whitespace).unwrap_or(text.len());
    let (name, mut tail) = text.split_at(split);
    if name.eq_ignore_ascii_case("TURN_OFF_HEATERS") {
        return if tail.trim().is_empty() {
            Ok(Some(Command::Off))
        } else {
            Err(Error::Syntax)
        };
    }
    let set = name.eq_ignore_ascii_case("SET_HEATER_TEMPERATURE");
    if !set && !name.eq_ignore_ascii_case("TEMPERATURE_WAIT") {
        return Ok(None);
    }
    let mut source = None;
    let mut target = None;
    let mut minimum = None;
    let mut maximum = None;
    while !tail.trim_start().is_empty() {
        let (key, value, remainder) = pair(tail.trim_start())?;
        if (set && key.eq_ignore_ascii_case("HEATER") || !set && key.eq_ignore_ascii_case("SENSOR"))
            && source.is_none()
        {
            source = Some(value);
        } else if set && key.eq_ignore_ascii_case("TARGET") && target.is_none() {
            target = Some(number(value)?);
        } else if !set && key.eq_ignore_ascii_case("MINIMUM") && minimum.is_none() {
            minimum = Some(number(value)?);
        } else if !set && key.eq_ignore_ascii_case("MAXIMUM") && maximum.is_none() {
            maximum = Some(number(value)?);
        } else {
            return Err(Error::Syntax);
        }
        tail = remainder;
    }
    let source = source.ok_or(Error::Syntax)?;
    Ok(Some(if set {
        Command::Set {
            heater: Heater::Named(source),
            target: target.unwrap_or(0.),
            wait: false,
        }
    } else {
        Command::WaitSensor {
            name: source,
            minimum,
            maximum,
        }
    }))
}

fn pair(text: &str) -> Result<(&str, &str, &str), Error> {
    let (key, value) = text.split_once('=').ok_or(Error::Syntax)?;
    if key.is_empty() || !key.bytes().all(|b| b.is_ascii_alphabetic() || b == b'_') {
        return Err(Error::Syntax);
    }
    let first = value.as_bytes().first().copied().ok_or(Error::Syntax)?;
    let (raw, tail) = if matches!(first, b'"' | b'\'') {
        let body = &value[1..];
        let end = body.find(char::from(first)).ok_or(Error::Syntax)?;
        if body[..end].contains('\\') {
            return Err(Error::Syntax);
        }
        (&body[..end], &body[end + 1..])
    } else {
        let end = value.find(char::is_whitespace).unwrap_or(value.len());
        (&value[..end], &value[end..])
    };
    if raw.is_empty() || !tail.is_empty() && !tail.starts_with(char::is_whitespace) {
        return Err(Error::Syntax);
    }
    Ok((key, raw, tail))
}

fn number(text: &str) -> Result<f64, Error> {
    text.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or(Error::Syntax)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lexes_targets_without_confusing_stock_t_with_bare_tool_selection() {
        assert_eq!(
            parse_gcode(" m109s215t1 ; note").unwrap(),
            Some(Command::Set {
                heater: Heater::Extruder(Some(1)),
                target: 215.,
                wait: true,
            })
        );
        assert_eq!(
            parse_gcode("M140").unwrap(),
            Some(Command::Set {
                heater: Heater::Bed,
                target: 0.,
                wait: false
            })
        );
        assert_eq!(parse_gcode("t2"), Ok(None));
        assert_eq!(parse_gcode("M104.1 S200"), Ok(None));
        assert_eq!(
            parse_gcode("TEMPERATURE_WAIT SENSOR='temperature_sensor chamber' MAXIMUM=24").unwrap(),
            Some(Command::WaitSensor {
                name: "temperature_sensor chamber",
                minimum: None,
                maximum: Some(24.)
            })
        );
        for input in [
            "M104 T1.0",
            "M109 T-1",
            "M104 S210 S220",
            "M140 T1",
            "TEMPERATURE_WAIT SENSOR='unfinished",
            "SET_HEATER_TEMPERATURE TARGET=200",
        ] {
            assert_eq!(parse_gcode(input), Err(Error::Syntax), "{input}");
        }
    }
}
