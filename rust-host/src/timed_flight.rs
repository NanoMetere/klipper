// SPDX-License-Identifier: GPL-3.0-only
//! Compose established sequence tracking with stock adaptive retry timing.
//! No device is opened; caller controls successful writes and clock source.

use crate::{
    flight::{Flight, Progress, ReceiveError, Retry, SendError, FRAME_WINDOW},
    planner::Wire,
    retry_timing::{Error as TimingError, Timer, Timing},
    Frame, MAX_PAYLOAD, MIN_FRAME,
};
use std::{collections::VecDeque, io};

#[derive(Debug)]
pub enum Error {
    Send(SendError),
    Receive(ReceiveError),
    Timing(TimingError),
    Wire,
    NotDue,
}

/// Owns all transmission and ACK clock anchors for one established link.
/// Callers must close the old session after any failed/short write.
pub struct TimedFlight {
    flight: Flight,
    clock: Timing,
    wire: Wire,
    sent_end: VecDeque<(f64, usize)>,
}

impl TimedFlight {
    pub fn established(sequence: u64, wire: Wire) -> Result<Self, Error> {
        wire.seconds(1).map_err(|_| Error::Wire)?;
        Ok(Self {
            flight: Flight::established(sequence),
            clock: Timing::default(),
            wire,
            sent_end: VecDeque::with_capacity(FRAME_WINDOW),
        })
    }

    pub fn pending_frames(&self) -> usize {
        self.flight.pending_frames()
    }
    pub fn outstanding_bytes(&self) -> usize {
        self.flight.outstanding_bytes()
    }
    pub fn prior_ack_bytes(&self) -> usize {
        self.flight.prior_ack_bytes()
    }
    pub fn timer(&self) -> Timer {
        self.clock.timer()
    }

    pub fn send(
        &mut self,
        payload: &[u8],
        receive_window: usize,
        start: f64,
        write: impl FnOnce(&[u8]) -> io::Result<usize>,
    ) -> Result<u64, Error> {
        if payload.is_empty() || payload.len() > MAX_PAYLOAD {
            return Err(Error::Send(SendError::InvalidPayload));
        }
        let bytes = payload.len().checked_add(MIN_FRAME).ok_or(Error::Wire)?;
        let end = start + self.wire.seconds(bytes).map_err(|_| Error::Wire)?;
        validate(start)?;
        validate(end)?;
        let first = self.sent_end.is_empty();
        let seq = self
            .flight
            .send(payload, receive_window, write)
            .map_err(Error::Send)?;
        self.clock.sent(seq, end, first).map_err(Error::Timing)?;
        self.sent_end.push_back((end, bytes));
        Ok(seq)
    }

    pub fn observe(&mut self, frame: Frame<'_>, now: f64) -> Result<Progress, Error> {
        validate(now)?;
        let p = self.flight.observe_decoded(frame).map_err(Error::Receive)?;
        if p.retired != 0 {
            let end = self
                .sent_end
                .drain(..p.retired)
                .next_back()
                .map(|(end, _)| end);
            let next_wire = self
                .sent_end
                .front()
                .map(|(_, bytes)| self.wire.seconds(*bytes))
                .transpose()
                .map_err(|_| Error::Wire)?;
            self.clock
                .acknowledged(p.receive_sequence, now, end, next_wire)
                .map_err(Error::Timing)?;
        }
        if p.fast_retry {
            self.clock.arm_fast_retry().map_err(Error::Timing)?;
        }
        Ok(p)
    }

    pub fn retry_if_due(
        &mut self,
        now: f64,
        write: impl FnOnce(&[u8]) -> io::Result<usize>,
    ) -> Result<Retry, Error> {
        validate(now)?;
        let reason = match self.clock.timer() {
            Timer::Fast => Retry::DuplicateAck,
            Timer::At(at) if now >= at => Retry::Timeout,
            _ => return Err(Error::NotDue),
        };
        let (_, first) = self.sent_end.front().ok_or(Error::NotDue)?;
        let first_wire = self.wire.seconds(first + 1).map_err(|_| Error::Wire)?;
        self.flight.retry(reason, write).map_err(Error::Send)?;
        self.clock
            .retried(reason, now, first_wire)
            .map_err(Error::Timing)?;
        Ok(reason)
    }
}

fn validate(time: f64) -> Result<(), Error> {
    if (0.0..=1e12).contains(&time) {
        Ok(())
    } else {
        Err(Error::Timing(TimingError::InvalidTime))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeats_cached_frames_after_cumulative_ack_and_fast_retry() {
        let mut f = TimedFlight::established(14, Wire::Pty).unwrap();
        let write = |bytes: &[u8]| Ok(bytes.len());
        f.send(&[2], 0, 1.0, write).unwrap();
        f.send(&[3], 0, 1.0, write).unwrap();
        assert!(matches!(f.retry_if_due(1.0, write), Err(Error::NotDue)));
        f.observe(
            Frame {
                sequence: 15,
                payload: &[2],
            },
            1.1,
        )
        .unwrap();
        f.observe(
            Frame {
                sequence: 15,
                payload: &[],
            },
            1.1,
        )
        .unwrap();
        assert!(matches!(f.timer(), Timer::At(_)));
        f.observe(
            Frame {
                sequence: 15,
                payload: &[],
            },
            1.101,
        )
        .unwrap();
        assert_eq!(f.timer(), Timer::Fast);
        assert_eq!(
            f.retry_if_due(1.101, |bytes| {
                let frame = crate::encode(15, &[3]).unwrap();
                assert_eq!(bytes, [&[0x7e][..], &frame].concat());
                write(bytes)
            })
            .unwrap(),
            Retry::DuplicateAck
        );
        f.observe(
            Frame {
                sequence: 0,
                payload: &[],
            },
            1.3,
        )
        .unwrap();
        assert_eq!(f.timer(), Timer::Idle);
    }
}
