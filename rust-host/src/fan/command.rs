// SPDX-License-Identifier: GPL-3.0-only
//! Parse only the stock print-fan demand. No macros, pins or PWM writes.
use super::Error;

/// For the bounded stock subset, return the normalized print-fan request
/// (Klippy clamps it at the fan scheduler). A renamed macro alias or other
/// command is not a match. Unsupported extra parameters are rejected.
pub fn print_demand(raw: &str) -> Result<Option<f64>, Error> {
    let head = raw.split(';').next().unwrap_or_default().trim();
    let bytes = head.as_bytes();
    if !bytes.first().is_some_and(|b| b.eq_ignore_ascii_case(&b'm')) {
        return Ok(None);
    }
    let n = 1 + bytes[1..].iter().take_while(|b| b.is_ascii_digit()).count();
    let code = match &head[1..n] {
        "106" => true,
        "107" => false,
        _ => return Ok(None),
    };
    if bytes.get(n) == Some(&b'.') {
        return Ok(None);
    }
    let tail = head[n..].trim();
    if tail.is_empty() {
        return Ok(Some(if code { 1. } else { 0. }));
    }
    if !code || tail.len() > 64 || !tail.as_bytes()[0].eq_ignore_ascii_case(&b's') {
        return Err(Error::Request);
    }
    let speed = tail[1..].trim();
    if !speed
        .bytes()
        .all(|b| b.is_ascii_digit() || matches!(b, b'+' | b'-' | b'.'))
    {
        return Err(Error::Request);
    }
    let value = speed.parse::<f64>().map_err(|_| Error::Request)?;
    if !value.is_finite() || value < 0. {
        return Err(Error::Request);
    }
    Ok(Some(value / 255.))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_stock_print_fan_is_recognized() {
        assert_eq!(print_demand(" m106 ; default\r\n"), Ok(Some(1.)));
        assert_eq!(print_demand("m106s127.5"), Ok(Some(0.5)));
        assert_eq!(print_demand("M106 S 510"), Ok(Some(2.)));
        assert_eq!(print_demand("M107"), Ok(Some(0.)));
        for raw in [
            "M106 S-0.5",
            "M106 P0 S128",
            "M106 S1 S2",
            "M107 S1",
            "M106 S1E4",
        ] {
            assert_eq!(print_demand(raw), Err(Error::Request), "{raw}");
        }
        for raw in [
            "M106.1 S255",
            "T2",
            "M109 S210",
            "SET_FAN_SPEED FAN=aux SPEED=.5",
            "; M106 S100",
        ] {
            assert_eq!(print_demand(raw), Ok(None), "{raw}");
        }
    }
}
