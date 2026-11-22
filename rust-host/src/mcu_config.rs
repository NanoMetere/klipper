// SPDX-License-Identifier: GPL-3.0-only
//! Offline MCU startup configuration and response validation.
//! Adapted from Klipper klippy/mcu.py (MCUConfigHelper).
//! Upstream copyright (C) 2016-2026 Kevin O'Connor <kevin@koconnor.net>.
//! The supported profile is deliberately pin-free; this module never opens a device.

use crate::{encode, identify::Dictionary, vlq, Frame, FrameError};

const GET: &str = "get_config";
const REPORT: &str = "config is_config=%c crc=%u is_shutdown=%c move_count=%hu";
const ALLOCATE: &str = "allocate_oids count=%c";
const TRSYNC: &str = "config_trsync oid=%c";
const FINALIZE: &str = "finalize_config crc=%u";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    MissingFormat(&'static str),
    InvalidFormat,
    InvalidReply,
    UnexpectedMessage,
    Frame(FrameError),
    Shutdown,
    RestartDidNotReset,
    CrcMismatch { expected: u32, actual: u32 },
    NotConfigured,
    MoveSlots { required: u16, available: u16 },
    InvalidState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    pub configured: bool,
    pub crc: u32,
    pub shutdown: bool,
    pub move_count: u16,
}

pub struct Messages {
    get: i32,
    report: i32,
    allocate: i32,
    trsync: i32,
    finalize: i32,
}

impl Messages {
    pub fn from_dictionary(dictionary: &Dictionary) -> Result<Self, Error> {
        let command = |k| {
            dictionary
                .commands
                .get(k)
                .copied()
                .ok_or(Error::MissingFormat(k))
        };
        let get = command(GET)?;
        let report = dictionary
            .responses
            .get(REPORT)
            .copied()
            .ok_or(Error::MissingFormat(REPORT))?;
        let (allocate, trsync, finalize) =
            (command(ALLOCATE)?, command(TRSYNC)?, command(FINALIZE)?);
        let ids = [get, report, allocate, trsync, finalize];
        if ids.iter().enumerate().any(|(i, id)| ids[..i].contains(id)) {
            return Err(Error::InvalidFormat);
        }
        Ok(Self {
            get,
            report,
            allocate,
            trsync,
            finalize,
        })
    }

    pub fn get_config(&self, sequence: u8) -> Result<Vec<u8>, Error> {
        encode(sequence, &payload(&[self.get])).map_err(Error::Frame)
    }

    pub fn report(&self, frame: Frame<'_>) -> Result<Report, Error> {
        let mut bytes = frame.payload;
        let id = read(&mut bytes)?;
        if id != i64::from(self.report) {
            return Err(Error::UnexpectedMessage);
        }
        let flag = |v| match v {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(Error::InvalidReply),
        };
        let configured = flag(read(&mut bytes)?)?;
        let crc = read(&mut bytes)?;
        if !(i32::MIN as i64..=u32::MAX as i64).contains(&crc) {
            return Err(Error::InvalidReply);
        }
        let shutdown = flag(read(&mut bytes)?)?;
        let move_count = u16::try_from(read(&mut bytes)?).map_err(|_| Error::InvalidReply)?;
        if !bytes.is_empty() {
            return Err(Error::InvalidReply);
        }
        Ok(Report {
            configured,
            crc: crc as u32,
            shutdown,
            move_count,
        })
    }

    /// Inert payload records in the same order as Klippy's resolved strings.
    pub fn payloads(&self, profile: &Profile) -> Vec<Vec<u8>> {
        let mut commands = Vec::with_capacity(usize::from(profile.trsync_count) + 2);
        commands.push(payload(&[self.allocate, i32::from(profile.trsync_count)]));
        for oid in 0..profile.trsync_count {
            commands.push(payload(&[self.trsync, i32::from(oid)]));
        }
        let mut finalize = payload(&[self.finalize]);
        vlq::encode_unsigned(&mut finalize, profile.crc);
        commands.push(finalize);
        commands
    }
}

fn payload(parameters: &[i32]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(parameters.len() * 2);
    for &value in parameters {
        vlq::encode(&mut bytes, value);
    }
    bytes
}

fn read(bytes: &mut &[u8]) -> Result<i64, Error> {
    vlq::read(bytes).ok_or(Error::InvalidReply)
}

pub struct Profile {
    trsync_count: u8,
    reserved: u16,
    crc: u32,
}

impl Profile {
    /// A pin-free list of trigger synchronizers for software-only fixtures.
    pub fn trsync_only(count: u8, reserved_move_slots: u16) -> Self {
        let mut text = format!("allocate_oids count={count}");
        for oid in 0..count {
            use std::fmt::Write;
            write!(&mut text, "\nconfig_trsync oid={oid}").unwrap();
        }
        Self {
            trsync_count: count,
            reserved: reserved_move_slots,
            crc: crc32(text.as_bytes()),
        }
    }

    pub fn crc(&self) -> u32 {
        self.crc
    }
}

pub(crate) fn crc32(bytes: &[u8]) -> u32 {
    const fn table() -> [u32; 256] {
        let mut out = [0; 256];
        let mut i = 0;
        while i < 256 {
            let mut value = i as u32;
            let mut bit = 0;
            while bit < 8 {
                value = (value >> 1) ^ if value & 1 != 0 { 0xedb8_8320 } else { 0 };
                bit += 1;
            }
            out[i] = value;
            i += 1;
        }
        out
    }
    const TABLE: [u32; 256] = table();
    let mut crc = u32::MAX;
    for &b in bytes {
        crc = (crc >> 8) ^ TABLE[(crc as u8 ^ b) as usize];
    }
    !crc
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Start {
    NeedsConfiguration,
    ReuseMatching,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Waiting,
    Verify,
    Terminal,
}

/// Deterministic config gate. A restart or shutdown is reported to the
/// caller instead of performing any automatic reset or side effects.
pub struct Session<'a> {
    profile: &'a Profile,
    phase: Phase,
}

impl<'a> Session<'a> {
    pub fn new(profile: &'a Profile) -> Self {
        Self {
            profile,
            phase: Phase::Waiting,
        }
    }

    pub fn first(&mut self, report: Report, after_explicit_restart: bool) -> Result<Start, Error> {
        if self.phase != Phase::Waiting {
            return Err(Error::InvalidState);
        }
        self.phase = Phase::Terminal;
        if report.shutdown {
            return Err(Error::Shutdown);
        }
        let start = if report.configured {
            if after_explicit_restart {
                return Err(Error::RestartDidNotReset);
            }
            self.check_crc(report.crc)?;
            Start::ReuseMatching
        } else {
            Start::NeedsConfiguration
        };
        self.phase = Phase::Verify;
        Ok(start)
    }

    pub fn verify(&mut self, report: Report) -> Result<u16, Error> {
        if self.phase != Phase::Verify {
            return Err(Error::InvalidState);
        }
        self.phase = Phase::Terminal;
        if report.shutdown {
            return Err(Error::Shutdown);
        }
        if !report.configured {
            return Err(Error::NotConfigured);
        }
        self.check_crc(report.crc)?;
        report
            .move_count
            .checked_sub(self.profile.reserved)
            .ok_or(Error::MoveSlots {
                required: self.profile.reserved,
                available: report.move_count,
            })
    }

    fn check_crc(&self, actual: u32) -> Result<(), Error> {
        (actual == self.profile.crc)
            .then_some(())
            .ok_or(Error::CrcMismatch {
                expected: self.profile.crc,
                actual,
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn startup_gate_is_sticky_and_checks_crc_shutdown_and_reserved_slots() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        let profile = Profile::trsync_only(3, 13);
        let blank = Report {
            configured: false,
            crc: 0,
            shutdown: false,
            move_count: 0,
        };
        let ready = Report {
            configured: true,
            crc: profile.crc(),
            shutdown: false,
            move_count: 100,
        };
        let mut fresh = Session::new(&profile);
        assert_eq!(fresh.first(blank, false), Ok(Start::NeedsConfiguration));
        assert_eq!(fresh.verify(ready), Ok(87));
        assert_eq!(fresh.verify(ready), Err(Error::InvalidState));
        let mut reused = Session::new(&profile);
        assert_eq!(reused.first(ready, false), Ok(Start::ReuseMatching));
        assert_eq!(
            reused.verify(Report {
                move_count: 12,
                ..ready
            }),
            Err(Error::MoveSlots {
                required: 13,
                available: 12
            })
        );
        assert_eq!(
            Session::new(&profile).first(ready, true),
            Err(Error::RestartDidNotReset)
        );
        assert_eq!(
            Session::new(&profile).first(
                Report {
                    shutdown: true,
                    ..ready
                },
                false
            ),
            Err(Error::Shutdown)
        );
        let mut bad = Session::new(&profile);
        assert!(matches!(
            bad.first(Report { crc: 0, ..ready }, false),
            Err(Error::CrcMismatch { .. })
        ));
        assert_eq!(bad.first(ready, false), Err(Error::InvalidState));
    }
}
