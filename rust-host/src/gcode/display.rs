// SPDX-License-Identifier: GPL-3.0-only
//! Inert display state adapted from Klipper's extras/display_status.py.
//! Upstream copyright (C) 2018-2020 Kevin O'Connor and Eric Callahan.

use super::{Error, MAX_LINE};

#[derive(Debug, PartialEq)]
pub enum Command<'a> {
    /// Stock ignores M73 without P. R (estimated minutes) is not displayed.
    Progress(Option<f64>),
    Message(Option<&'a str>),
    ExtendedMessage(Option<String>),
}

/// The bounded, unnumbered subset used by the offline preview. For M117
/// the raw message, including embedded semicolons, is not a G-code command.
pub fn parse(line: &str) -> Result<Option<Command<'_>>, Error> {
    let raw = line.trim();
    let prefix = |cmd: &str| {
        let (head, tail) = raw.split_at_checked(cmd.len())?;
        (head.eq_ignore_ascii_case(cmd)
            && tail
                .chars()
                .next()
                .is_none_or(|c| c.is_whitespace() || c.is_ascii_alphabetic() || c == ';'))
        .then_some(tail)
    };
    let message = prefix("M117");
    let progress = prefix("M73");
    let extended_message = "SET_DISPLAY_TEXT";
    let extended = raw
        .split_at_checked(extended_message.len())
        .filter(|(head, tail)| {
            head.eq_ignore_ascii_case(extended_message)
                && tail
                    .chars()
                    .next()
                    .is_none_or(|c| c.is_whitespace() || c == ';')
        })
        .map(|(_, tail)| tail);
    if message.is_none() && progress.is_none() && extended.is_none() {
        return Ok(None);
    }
    if line.len() > MAX_LINE || line.chars().any(|c| c.is_control() && c != '\t') {
        return Err(Error::InvalidLine);
    }
    if let Some(tail) = message {
        let text = tail.strip_prefix(char::is_whitespace).unwrap_or(tail);
        return Ok(Some(Command::Message((!text.is_empty()).then_some(text))));
    }
    if let Some(tail) = extended {
        let mut params = super::extended::display_args(tail)?;
        return Ok(Some(Command::ExtendedMessage(params.remove("MSG"))));
    }
    let words = super::parse(raw)?.ok_or(Error::InvalidLine)?;
    words.allow(b"PRC")?;
    Ok(Some(Command::Progress(words.get(b'P'))))
}

#[derive(Debug, Default)]
pub struct Display {
    progress: Option<f64>,
    expires: f64,
    message: Option<String>,
}

#[derive(Debug, PartialEq)]
pub struct Status<'a> {
    pub progress: f64,
    pub message: Option<&'a str>,
}

impl Display {
    /// Observe a line at its explicit virtual dispatch time, not its
    /// motion-completion time. Returns false for a non-display command.
    pub fn observe(&mut self, line: &str, at: f64) -> Result<bool, Error> {
        if !at.is_finite() || at < 0.0 {
            return Err(Error::InvalidParameter);
        }
        match parse(line)? {
            Some(Command::Progress(Some(percent))) => {
                self.progress = Some(0.0_f64.max(percent / 100.0).min(1.0));
                self.expires = at + 5.0;
            }
            Some(Command::Progress(None)) => {}
            Some(Command::Message(text)) => self.message = text.map(str::to_owned),
            Some(Command::ExtendedMessage(text)) => self.message = text,
            None => return Ok(false),
        }
        Ok(true)
    }

    /// The caller supplies simulated idle-timeout state and, if present,
    /// current virtual SD progress; preflight bytes are not runtime progress.
    pub fn status(
        &mut self,
        at: f64,
        printing: bool,
        virtual_sd_progress: Option<f64>,
    ) -> Result<Status<'_>, Error> {
        if !at.is_finite()
            || at < 0.0
            || virtual_sd_progress.is_some_and(|v| !(0.0..=1.0).contains(&v))
        {
            return Err(Error::InvalidParameter);
        }
        if !printing && at > self.expires {
            self.progress = None;
        }
        Ok(Status {
            progress: self.progress.or(virtual_sd_progress).unwrap_or(0.0),
            message: self.message.as_deref(),
        })
    }

    #[cfg(feature = "webhooks")]
    pub fn publish(
        &mut self,
        snapshot: &mut crate::webhooks::Snapshot,
        at: f64,
        printing: bool,
        virtual_sd_progress: Option<f64>,
    ) -> Result<bool, Error> {
        let status = self.status(at, printing, virtual_sd_progress)?;
        let fields = serde_json::json!({
            "progress": status.progress,
            "message": status.message,
        });
        Ok(snapshot.set_object("display_status".into(), fields.as_object().unwrap().clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_never_consumes_other_commands_or_seeks() {
        for line in ["G1 X3", "M730 P20", "M117.1", "N1 M117 text*10"] {
            assert_eq!(parse(line), Ok(None));
        }
        for line in ["M73 Pnan", "M73 P1 P2", "M73 P20 X3", "M117 x\0y"] {
            assert!(parse(line).is_err(), "{line}");
        }
        assert_eq!(
            parse(" m117  출력 ; 1 "),
            Ok(Some(Command::Message(Some(" 출력 ; 1"))))
        );
        assert_eq!(parse("M117"), Ok(Some(Command::Message(None))));
        assert_eq!(parse("M73 R20"), Ok(Some(Command::Progress(None))));
    }
}
