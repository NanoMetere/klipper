// SPDX-License-Identifier: GPL-3.0-only
//! Wire codecs specifically for the no-GPIO Linux motion test firmware.
//! No normal printer dictionary can pass the fixture capability check.

use crate::{encode, identify::Dictionary, mcu_config::crc32, vlq, Frame, FrameError};
use std::fmt::Write;

const SETUP: &str =
    "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u";
const RESET: &str = "reset_step_clock oid=%c clock=%u";
const GET: &str = "stepper_get_position oid=%c";
const POSITION: &str = "stepper_position oid=%c pos=%i";
const MAX_FIXTURE_OID: u8 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    NotFixture,
    MissingFormat(&'static str),
    Invalid,
    UnknownMessage,
    Frame(FrameError),
}

pub struct Codec {
    allocate: i32,
    setup: i32,
    finalize: i32,
    reset: i32,
    get: i32,
    advance: i32,
    position: i32,
}

impl Codec {
    pub fn from_dictionary(d: &Dictionary) -> Result<Self, Error> {
        if d.config
            .get("GPIO_SIM_ONLY")
            .and_then(serde_json::Value::as_str)
            != Some("rust-host-test-v1")
            || d.config
                .get("TIMER_SIM_ONLY")
                .and_then(serde_json::Value::as_str)
                != Some("rust-host-test-v1")
            || !d
                .enumerations
                .get("pin")
                .and_then(|v| v.get("sim0"))
                .is_some_and(|v| v == &serde_json::json!([0, 32]))
        {
            return Err(Error::NotFixture);
        }
        let command = |key| {
            d.commands
                .get(key)
                .copied()
                .ok_or(Error::MissingFormat(key))
        };
        Ok(Self {
            allocate: command("allocate_oids count=%c")?,
            setup: command(SETUP)?,
            finalize: command("finalize_config crc=%u")?,
            reset: command(RESET)?,
            get: command(GET)?,
            advance: command("rust_fixture_advance clock=%u")?,
            position: d
                .responses
                .get(POSITION)
                .copied()
                .ok_or(Error::MissingFormat(POSITION))?,
        })
    }

    /// One to six fixture objects using only the inert sim0..sim11 pins.
    pub fn setup(&self, count: u8, pulse_ticks: u32) -> Result<(u32, Vec<Vec<u8>>), Error> {
        if !(1..=MAX_FIXTURE_OID + 1).contains(&count) || !(1..=10_000).contains(&pulse_ticks) {
            return Err(Error::Invalid);
        }
        let count = u32::from(count);
        let mut records = vec![values(self.allocate, &[count])];
        let mut config = format!("allocate_oids count={count}");
        for oid in 0..count {
            let (step, dir) = (oid * 2, oid * 2 + 1);
            records.push(values(self.setup, &[oid, step, dir, 0, pulse_ticks]));
            write!(&mut config, "\nconfig_stepper oid={oid} step_pin=sim{step} dir_pin=sim{dir} invert_step=0 step_pulse_ticks={pulse_ticks}").unwrap();
        }
        let crc = crc32(config.as_bytes());
        records.push(values(self.finalize, &[crc]));
        Ok((crc, records))
    }

    pub fn reset_clock(&self, seq: u8, oid: u8, clock: u32) -> Result<Vec<u8>, Error> {
        if oid > MAX_FIXTURE_OID {
            return Err(Error::Invalid);
        }
        encode(seq, &values(self.reset, &[u32::from(oid), clock])).map_err(Error::Frame)
    }

    pub fn advance_time(&self, seq: u8, clock: u32) -> Result<Vec<u8>, Error> {
        encode(seq, &values(self.advance, &[clock])).map_err(Error::Frame)
    }

    pub fn get_position(&self, seq: u8, oid: u8) -> Result<Vec<u8>, Error> {
        if oid > MAX_FIXTURE_OID {
            return Err(Error::Invalid);
        }
        encode(seq, &values(self.get, &[u32::from(oid)])).map_err(Error::Frame)
    }

    pub fn position(&self, frame: Frame<'_>) -> Result<(u8, i32), Error> {
        let mut raw = frame.payload;
        let id = vlq::read(&mut raw).ok_or(Error::Invalid)?;
        if i64::from(self.position) != id {
            return Err(Error::UnknownMessage);
        }
        let oid = vlq::read(&mut raw)
            .and_then(|v| u8::try_from(v).ok())
            .ok_or(Error::Invalid)?;
        let position = vlq::read(&mut raw)
            .and_then(|v| i32::try_from(v).ok())
            .ok_or(Error::Invalid)?;
        if !raw.is_empty() || oid > MAX_FIXTURE_OID {
            return Err(Error::Invalid);
        }
        Ok((oid, position))
    }
}

fn values(id: i32, fields: &[u32]) -> Vec<u8> {
    let mut bytes = Vec::new();
    vlq::encode(&mut bytes, id);
    for &x in fields {
        vlq::encode_unsigned(&mut bytes, x);
    }
    bytes
}

/// Captures an unsolicited shutdown reason during a fixture run.
pub struct ShutdownWatch {
    live: i32,
    sticky: i32,
    pub code: Option<u16>,
}

impl ShutdownWatch {
    pub fn new(d: &Dictionary) -> Option<Self> {
        Some(Self {
            live: *d.responses.get("shutdown clock=%u static_string_id=%hu")?,
            sticky: *d.responses.get("is_shutdown static_string_id=%hu")?,
            code: None,
        })
    }

    pub fn observe(&mut self, mut raw: &[u8]) {
        let Some(id) = vlq::read(&mut raw) else {
            return;
        };
        if id == i64::from(self.live) {
            if vlq::read(&mut raw).is_none() {
                return;
            }
        } else if id != i64::from(self.sticky) {
            return;
        }
        if let Some(code) = vlq::read(&mut raw).and_then(|i| u16::try_from(i).ok()) {
            if raw.is_empty() {
                self.code = Some(code);
            }
        }
    }

    pub fn description<'a>(&self, dict: &'a Dictionary) -> Option<&'a str> {
        let code = self.code?;
        dict.enumerations
            .get("static_string_id")?
            .as_object()?
            .iter()
            .find(|(_, v)| v.as_u64() == Some(u64::from(code)))
            .map(|(k, _)| k.as_str())
    }
}
