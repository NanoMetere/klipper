// SPDX-License-Identifier: GPL-3.0-only
//! Dictionary-selected clock/uptime messages. No scheduling or device I/O.

use crate::{encode, identify::Dictionary, vlq, Frame, FrameError};

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    MissingFormat(&'static str),
    InvalidInteger,
    UnexpectedMessage,
    TrailingData,
    Frame(FrameError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Response {
    Clock(u32),
    Uptime(u64),
}

pub struct ClockMessages {
    get_clock: i32,
    get_uptime: i32,
    clock: i32,
    uptime: i32,
}

impl ClockMessages {
    pub fn from_dictionary(dictionary: &Dictionary) -> Result<Self, Error> {
        Ok(Self {
            get_clock: *dictionary
                .commands
                .get("get_clock")
                .ok_or(Error::MissingFormat("get_clock"))?,
            get_uptime: *dictionary
                .commands
                .get("get_uptime")
                .ok_or(Error::MissingFormat("get_uptime"))?,
            clock: *dictionary
                .responses
                .get("clock clock=%u")
                .ok_or(Error::MissingFormat("clock clock=%u"))?,
            uptime: *dictionary
                .responses
                .get("uptime high=%u clock=%u")
                .ok_or(Error::MissingFormat("uptime high=%u clock=%u"))?,
        })
    }

    fn request(&self, sequence: u8, id: i32) -> Result<Vec<u8>, Error> {
        let mut payload = Vec::with_capacity(5);
        vlq::encode(&mut payload, id);
        encode(sequence, &payload).map_err(Error::Frame)
    }

    pub fn get_clock(&self, sequence: u8) -> Result<Vec<u8>, Error> {
        self.request(sequence, self.get_clock)
    }

    pub fn get_uptime(&self, sequence: u8) -> Result<Vec<u8>, Error> {
        self.request(sequence, self.get_uptime)
    }

    pub fn response(&self, frame: Frame<'_>) -> Result<Response, Error> {
        fn read_u32(input: &mut &[u8]) -> Result<u32, Error> {
            let value = vlq::read(input).ok_or(Error::InvalidInteger)?;
            if !(i32::MIN as i64..=u32::MAX as i64).contains(&value) {
                return Err(Error::InvalidInteger);
            }
            Ok(value as u32)
        }

        let mut input = frame.payload;
        let id = vlq::read(&mut input)
            .and_then(|id| i32::try_from(id).ok())
            .ok_or(Error::InvalidInteger)?;
        let response = if id == self.clock {
            Response::Clock(read_u32(&mut input)?)
        } else if id == self.uptime {
            let high = u64::from(read_u32(&mut input)?);
            let low = u64::from(read_u32(&mut input)?);
            Response::Uptime((high << 32) | low)
        } else {
            return Err(Error::UnexpectedMessage);
        };
        if !input.is_empty() {
            return Err(Error::TrailingData);
        }
        Ok(response)
    }
}
