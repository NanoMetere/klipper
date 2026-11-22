// SPDX-License-Identifier: GPL-3.0-only
//! Offline implementation of the bootstrap identify exchange used by Klippy.
//! Adapted from klippy/msgproto.py and klippy/serialhdl.py.
//! Upstream copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>.

use crate::{encode, vlq, Frame, FrameError};
use flate2::{Decompress, FlushDecompress, Status};
use serde::Deserialize;
use serde_json::Value;
use std::collections::BTreeMap;

pub const CHUNK_SIZE: usize = 40;
const MAX_COMPRESSED: usize = 256 * 1024;
const MAX_JSON: usize = 2 * 1024 * 1024;
const IDENTIFY_RESPONSE: u8 = 0;
const IDENTIFY: u8 = 1;

/// Firmware message formats are keyed by their complete format strings.
#[derive(Debug, Deserialize)]
pub struct Dictionary {
    pub commands: BTreeMap<String, i32>,
    pub responses: BTreeMap<String, i32>,
    #[serde(default)]
    pub output: BTreeMap<String, i32>,
    #[serde(default)]
    pub config: BTreeMap<String, Value>,
    #[serde(default)]
    pub enumerations: BTreeMap<String, Value>,
    pub kconfig: Option<Value>,
    #[serde(default)]
    pub version: String,
    #[serde(default)]
    pub build_versions: String,
}

#[derive(Debug)]
pub enum Error {
    Frame(FrameError),
    UnexpectedMessage,
    InvalidOffset,
    OffsetMismatch { expected: u32, received: u32 },
    InvalidChunk,
    CompressedLimit,
    JsonLimit,
    Incomplete,
    AlreadyComplete,
    InvalidCompression,
    InvalidJson(serde_json::Error),
}

/// Accumulates one dictionary. Rejected chunks never change the current offset.
#[derive(Debug, Default)]
pub struct Identify {
    compressed: Vec<u8>,
    done: bool,
}

impl Identify {
    pub fn request(&self, sequence: u8) -> Result<Vec<u8>, Error> {
        if self.done {
            return Err(Error::AlreadyComplete);
        }
        encode(
            sequence,
            &identify_payload(self.compressed.len() as u32, CHUNK_SIZE)?,
        )
        .map_err(Error::Frame)
    }

    /// Accepts an identify response; an empty chunk at the expected offset ends it.
    /// ACK frames should be handled by the caller, separately from this exchange.
    pub fn accept(&mut self, frame: Frame<'_>) -> Result<bool, Error> {
        if self.done {
            return Err(Error::AlreadyComplete);
        }
        let (received, chunk) = identify_response(frame)?;
        let expected = self.compressed.len() as u32;
        if received != expected {
            return Err(Error::OffsetMismatch { expected, received });
        }
        if self.compressed.len() + chunk.len() > MAX_COMPRESSED {
            return Err(Error::CompressedLimit);
        }
        self.compressed.extend_from_slice(chunk);
        self.done = chunk.is_empty();
        Ok(self.done)
    }

    /// The compressed bytes accepted during bootstrap, for read-only comparisons.
    pub fn compressed(&self) -> Option<&[u8]> {
        self.done.then_some(&self.compressed)
    }

    pub fn dictionary(&self) -> Result<Dictionary, Error> {
        if !self.done {
            return Err(Error::Incomplete);
        }
        let mut decoded = Vec::with_capacity(MAX_JSON + 1);
        let mut zlib = Decompress::new(true);
        let status = zlib
            .decompress_vec(&self.compressed, &mut decoded, FlushDecompress::Finish)
            .map_err(|_| Error::InvalidCompression)?;
        if decoded.len() > MAX_JSON {
            return Err(Error::JsonLimit);
        }
        if status != Status::StreamEnd || zlib.total_in() as usize != self.compressed.len() {
            return Err(Error::InvalidCompression);
        }
        serde_json::from_slice(&decoded).map_err(Error::InvalidJson)
    }
}

/// Encode a bounded identify read without a wire header.
pub fn identify_payload(offset: u32, len: usize) -> Result<Vec<u8>, Error> {
    if offset as usize > MAX_COMPRESSED {
        return Err(Error::InvalidOffset);
    }
    if !(1..=CHUNK_SIZE).contains(&len) {
        return Err(Error::InvalidChunk);
    }
    let mut payload = vec![IDENTIFY];
    vlq::encode(&mut payload, offset as i32);
    payload.push(len as u8);
    Ok(payload)
}

/// Decode the offset and body of one read-only identify response.
pub fn identify_response(frame: Frame<'_>) -> Result<(u32, &[u8]), Error> {
    let (id, mut body) = frame
        .payload
        .split_first()
        .ok_or(Error::UnexpectedMessage)?;
    if *id != IDENTIFY_RESPONSE {
        return Err(Error::UnexpectedMessage);
    }
    let offset = vlq::read(&mut body)
        .and_then(|n| u32::try_from(n).ok())
        .ok_or(Error::InvalidOffset)?;
    let (&len, chunk) = body.split_first().ok_or(Error::InvalidChunk)?;
    if usize::from(len) != chunk.len() || chunk.len() > CHUNK_SIZE {
        return Err(Error::InvalidChunk);
    }
    Ok((offset, chunk))
}
