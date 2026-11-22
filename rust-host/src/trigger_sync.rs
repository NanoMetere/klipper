// SPDX-License-Identifier: GPL-3.0-only
//! Firmware trigger-sync wire types and offline host-side timing.
//! Adapted from Klipper klippy/mcu.py and src/trsync.c.
//! Upstream copyright (C) 2016-2026 Kevin O'Connor <kevin@koconnor.net>.
//! No firmware or endstop is opened by this module.

use crate::{encode, identify::Dictionary, vlq, Frame, FrameError};

const ARM: &str = "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c";
const EXPIRE: &str = "trsync_set_timeout oid=%c clock=%u";
const TRIGGER: &str = "trsync_trigger oid=%c reason=%c";
const STATE: &str = "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    MissingFormat(&'static str),
    BadDictionary,
    InvalidTiming,
    InvalidReply,
    UnexpectedMessage,
    Frame(FrameError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    EndstopHit,
    HostRequest,
    PastEndTime,
    CommsTimeout,
    Other(u8),
}

impl Reason {
    fn from_code(value: u8) -> Self {
        match value {
            1 => Self::EndstopHit,
            2 => Self::HostRequest,
            3 => Self::PastEndTime,
            4 => Self::CommsTimeout,
            code => Self::Other(code),
        }
    }

    pub fn code(self) -> u8 {
        match self {
            Self::EndstopHit => 1,
            Self::HostRequest => 2,
            Self::PastEndTime => 3,
            Self::CommsTimeout => 4,
            Self::Other(code) => code,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    pub oid: u8,
    pub can_trigger: bool,
    pub reason: Reason,
    pub clock: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    pub report_clock: u64,
    pub report_ticks: u32,
    pub expires: u64,
    pub extend_threshold: u32,
}

impl Timing {
    /// Derive stock report and expiration clock positions. Each peer's first
    /// report is staggered, and multi-MCU homing uses a shorter timeout.
    pub fn for_peer(start: u64, hz: f64, peer: usize, peers: usize) -> Result<Self, Error> {
        if !(1..=255).contains(&peers) || peer >= peers || !(1e3..=1e9).contains(&hz) {
            return Err(Error::InvalidTiming);
        }
        let seconds = if peers == 1 { 0.250 } else { 0.025 };
        let expire_ticks = (hz * seconds) as u32;
        let report_ticks = (hz * (seconds * 0.3)) as u32;
        let stagger = (f64::from(report_ticks) * (peer as f64 / peers as f64) + 0.5) as u64;
        Ok(Self {
            report_clock: start.checked_add(stagger).ok_or(Error::InvalidTiming)?,
            report_ticks,
            expires: start
                .checked_add(u64::from(expire_ticks))
                .ok_or(Error::InvalidTiming)?,
            extend_threshold: (f64::from(report_ticks) * 0.8 + 0.5) as u32,
        })
    }
}

pub struct Messages {
    arm: i32,
    expire: i32,
    trigger: i32,
    state: i32,
}

impl Messages {
    pub fn from_dictionary(dictionary: &Dictionary) -> Result<Self, Error> {
        let command = |key| {
            dictionary
                .commands
                .get(key)
                .copied()
                .ok_or(Error::MissingFormat(key))
        };
        let (arm, expire, trigger) = (command(ARM)?, command(EXPIRE)?, command(TRIGGER)?);
        let state = dictionary
            .responses
            .get(STATE)
            .copied()
            .ok_or(Error::MissingFormat(STATE))?;
        let ids = [arm, expire, trigger, state];
        if ids.iter().enumerate().any(|(i, id)| ids[..i].contains(id)) {
            return Err(Error::BadDictionary);
        }
        Ok(Self {
            arm,
            expire,
            trigger,
            state,
        })
    }

    pub fn arm(
        &self,
        seq: u8,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        reason: Reason,
    ) -> Result<Vec<u8>, Error> {
        self.request(
            seq,
            self.arm,
            &[
                u32::from(oid),
                report_clock,
                report_ticks,
                u32::from(reason.code()),
            ],
        )
    }

    pub fn expire(&self, seq: u8, oid: u8, clock: u32) -> Result<Vec<u8>, Error> {
        self.request(seq, self.expire, &[u32::from(oid), clock])
    }

    pub fn trigger(&self, seq: u8, oid: u8, reason: Reason) -> Result<Vec<u8>, Error> {
        self.request(
            seq,
            self.trigger,
            &[u32::from(oid), u32::from(reason.code())],
        )
    }

    pub fn response(&self, frame: Frame<'_>) -> Result<State, Error> {
        let mut input = frame.payload;
        let mut next = || vlq::read(&mut input).ok_or(Error::InvalidReply);
        if next()? != i64::from(self.state) {
            return Err(Error::UnexpectedMessage);
        }
        let oid = u8::try_from(next()?).map_err(|_| Error::InvalidReply)?;
        let can_trigger = match next()? {
            0 => false,
            1 => true,
            _ => return Err(Error::InvalidReply),
        };
        let reason = Reason::from_code(u8::try_from(next()?).map_err(|_| Error::InvalidReply)?);
        let clock = next()?;
        if !input.is_empty() || !(i32::MIN as i64..=u32::MAX as i64).contains(&clock) {
            return Err(Error::InvalidReply);
        }
        Ok(State {
            oid,
            can_trigger,
            reason,
            clock: clock as u32,
        })
    }

    fn request(&self, seq: u8, id: i32, arguments: &[u32]) -> Result<Vec<u8>, Error> {
        let mut bytes = Vec::with_capacity(2 + arguments.len() * 5);
        vlq::encode(&mut bytes, id);
        for &value in arguments {
            vlq::encode_unsigned(&mut bytes, value);
        }
        encode(seq, &bytes).map_err(Error::Frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derives_distinct_multi_peer_deadlines_and_handles_u64_wrap() {
        let one = Timing::for_peer(2_000_000, 1e6, 0, 1).unwrap();
        assert_eq!(
            (
                one.report_clock,
                one.report_ticks,
                one.expires,
                one.extend_threshold
            ),
            (2_000_000, 75_000, 2_250_000, 60_000)
        );
        let four = Timing::for_peer(2_000_000, 1e6, 3, 4).unwrap();
        assert_eq!(
            (
                four.report_clock,
                four.report_ticks,
                four.expires,
                four.extend_threshold
            ),
            (2_005_625, 7_500, 2_025_000, 6_000)
        );
        assert_eq!(
            Timing::for_peer(u64::MAX, 1e6, 0, 1),
            Err(Error::InvalidTiming)
        );
    }
}
