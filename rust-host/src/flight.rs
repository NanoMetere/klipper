// SPDX-License-Identifier: GPL-3.0-only
//! Established-session transmission ledger, adapted from klippy/chelper/serialqueue.c.
//! Upstream copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>.

use crate::{decode_prefix, encode, Frame, MAX_FRAME, SEQ_MASK, SYNC};
use std::{collections::VecDeque, io};

pub(crate) const FRAME_WINDOW: usize = 12;

#[derive(Debug)]
pub enum SendError {
    Closed,
    Backpressure,
    InvalidPayload,
    SequenceExhausted,
    SuppressedRetry,
    Write(io::Error),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReceiveError {
    Closed,
    InvalidFrame,
    OutsideWindow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Progress {
    /// Absolute sequence of the next frame expected by the MCU.
    pub receive_sequence: u64,
    pub retired: usize,
    pub fast_retry: bool,
    /// For a payload, the last frame eligible for a send-completion notification.
    /// The newest retired frame is not eligible until an empty ACK or later sequence.
    pub notify_through: Option<u64>,
    pub payload_timing_valid: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Retry {
    DuplicateAck,
    Timeout,
}

/// Starts at a known, synchronized MCU sequence. Startup adoption and adaptive
/// RTO belong to the session driver, not this established ledger.
pub struct Flight {
    pending: VecDeque<Vec<u8>>,
    next: u64,
    received: u64,
    empty_ack: u64,
    last_ack_bytes: usize,
    outstanding: usize,
    ignore_nak_through: Option<u64>,
    retransmit_through: Option<u64>,
    retry_requested: bool,
    closed: bool,
}

impl Flight {
    pub fn established(sequence: u64) -> Self {
        Self {
            pending: VecDeque::with_capacity(FRAME_WINDOW),
            next: sequence,
            received: sequence,
            empty_ack: sequence,
            last_ack_bytes: 0,
            outstanding: 0,
            ignore_nak_through: None,
            retransmit_through: None,
            retry_requested: false,
            closed: false,
        }
    }

    pub fn pending_frames(&self) -> usize {
        self.pending.len()
    }

    pub fn outstanding_bytes(&self) -> usize {
        self.outstanding
    }

    pub fn prior_ack_bytes(&self) -> usize {
        if self.empty_ack < self.received {
            self.last_ack_bytes
        } else {
            0
        }
    }

    pub fn is_closed(&self) -> bool {
        self.closed
    }

    pub fn can_send(&self, receive_window: usize) -> bool {
        !self.closed
            && self.pending.len() < FRAME_WINDOW
            && (self.pending.is_empty()
                || receive_window == 0
                || self.outstanding + self.prior_ack_bytes() + MAX_FRAME <= receive_window)
    }

    /// Send one nonempty frame. The callback returns the number of bytes written.
    /// Any failed/short write permanently closes this instance: it may already
    /// have reached the peer. The caller must close the old transport, discard
    /// its planner and bootstrap a new session, not retry with a fresh ledger.
    pub fn send(
        &mut self,
        payload: &[u8],
        receive_window: usize,
        write: impl FnOnce(&[u8]) -> io::Result<usize>,
    ) -> Result<u64, SendError> {
        if self.closed {
            return Err(SendError::Closed);
        }
        if !self.can_send(receive_window) {
            return Err(SendError::Backpressure);
        }
        if payload.is_empty() {
            return Err(SendError::InvalidPayload);
        }
        let seq = self.next;
        let next = seq.checked_add(1).ok_or(SendError::SequenceExhausted)?;
        let frame = encode(seq as u8, payload).map_err(|_| SendError::InvalidPayload)?;
        self.write_exact(&frame, write)?;
        self.next = next;
        self.outstanding += frame.len();
        self.pending.push_back(frame);
        Ok(seq)
    }

    /// Observe exactly one validated wire block. A future ACK outside the
    /// established window never changes state, even at absolute sequence 1.
    pub fn observe(&mut self, bytes: &[u8]) -> Result<Progress, ReceiveError> {
        if self.closed {
            return Err(ReceiveError::Closed);
        }
        let Some((frame, n)) = decode_prefix(bytes).map_err(|_| ReceiveError::InvalidFrame)? else {
            return Err(ReceiveError::InvalidFrame);
        };
        if n != bytes.len() {
            return Err(ReceiveError::InvalidFrame);
        }
        self.observe_decoded(frame)
    }

    /// Consume a frame whose wire header, length and CRC were already verified
    /// by a stream decoder. For unvalidated wire bytes call `observe` instead.
    pub fn observe_decoded(&mut self, frame: Frame<'_>) -> Result<Progress, ReceiveError> {
        if self.closed {
            return Err(ReceiveError::Closed);
        }
        if frame.sequence > SEQ_MASK || frame.payload.len() > crate::MAX_PAYLOAD {
            return Err(ReceiveError::InvalidFrame);
        }
        let delta = frame.sequence.wrapping_sub(self.received as u8) & SEQ_MASK;
        if usize::from(delta) > self.pending.len() {
            return Err(ReceiveError::OutsideWindow);
        }
        self.received += u64::from(delta);
        for _ in 0..delta {
            let retired = self.pending.pop_front().expect("validated ACK");
            self.last_ack_bytes = retired.len();
            self.outstanding -= retired.len();
        }
        if delta != 0 {
            // A cumulative advance replaces a previously queued fast-retry timer.
            self.retry_requested = false;
        }
        let empty = frame.payload.is_empty();
        let mut fast_retry = false;
        if empty {
            if self.empty_ack < self.received {
                self.empty_ack = self.received;
            } else if !self.pending.is_empty()
                && self
                    .ignore_nak_through
                    .is_none_or(|marker| self.received > marker)
            {
                self.retry_requested = true;
                fast_retry = true;
            }
        }
        Ok(Progress {
            receive_sequence: self.received,
            retired: usize::from(delta),
            fast_retry,
            notify_through: self.received.checked_sub(if empty { 1 } else { 2 }),
            payload_timing_valid: !empty
                && self.last_ack_bytes != 0
                && self
                    .retransmit_through
                    .is_none_or(|marker| self.received > marker),
        })
    }

    /// Write one sync byte followed by cached unacknowledged frames in order.
    /// The driver owns timeout selection; duplicate-ACK retries must be armed.
    pub fn retry(
        &mut self,
        reason: Retry,
        write: impl FnOnce(&[u8]) -> io::Result<usize>,
    ) -> Result<(), SendError> {
        if self.closed {
            return Err(SendError::Closed);
        }
        if self.pending.is_empty() || (reason == Retry::DuplicateAck && !self.retry_requested) {
            return Err(SendError::SuppressedRetry);
        }
        let mut bytes = [0; 1 + FRAME_WINDOW * MAX_FRAME];
        bytes[0] = SYNC;
        let mut len = 1;
        for frame in &self.pending {
            bytes[len..len + frame.len()].copy_from_slice(frame);
            len += frame.len();
        }
        self.write_exact(&bytes[..len], write)?;
        self.ignore_nak_through = Some(match reason {
            Retry::DuplicateAck => self.received.max(self.retransmit_through.unwrap_or(0)),
            Retry::Timeout => self.next,
        });
        self.retransmit_through = Some(self.next);
        self.retry_requested = false;
        Ok(())
    }

    fn write_exact(
        &mut self,
        bytes: &[u8],
        write: impl FnOnce(&[u8]) -> io::Result<usize>,
    ) -> Result<(), SendError> {
        // Leave the ledger closed even if a callback unwinds after writing.
        self.closed = true;
        match write(bytes) {
            Ok(n) if n == bytes.len() => {
                self.closed = false;
                Ok(())
            }
            Ok(_) => Err(SendError::Write(io::Error::new(
                io::ErrorKind::WriteZero,
                "incomplete frame write",
            ))),
            Err(error) => Err(SendError::Write(error)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MAX_PAYLOAD;

    fn write(bytes: &[u8]) -> io::Result<usize> {
        Ok(bytes.len())
    }

    #[test]
    fn wrap_payload_does_not_supply_empty_ack_credit() {
        let mut f = Flight::established(14);
        for (i, len) in [10, 17, 20].into_iter().enumerate() {
            assert_eq!(f.send(&vec![42; len], 0, write).unwrap(), 14 + i as u64);
        }
        let p = f.observe(&encode(0, &[21]).unwrap()).unwrap();
        assert_eq!(
            p,
            Progress {
                receive_sequence: 16,
                retired: 2,
                fast_retry: false,
                notify_through: Some(14),
                payload_timing_valid: true
            }
        );
        assert_eq!(f.outstanding_bytes(), 25);
        assert_eq!(f.prior_ack_bytes(), 22);
        assert!(!f.can_send(110));
        assert!(f.can_send(111));
        let ack = encode(0, &[]).unwrap();
        let p = f.observe(&ack).unwrap();
        assert_eq!(p.notify_through, Some(15));
        assert!(!p.fast_retry);
        assert_eq!(f.prior_ack_bytes(), 0);
        assert!(f.can_send(89));
        assert!(f.observe(&ack).unwrap().fast_retry);
        f.retry(Retry::DuplicateAck, |wire| {
            assert_eq!(wire, [&[SYNC][..], &encode(0, &[42; 20]).unwrap()].concat());
            write(wire)
        })
        .unwrap();
        assert!(!f.observe(&ack).unwrap().fast_retry);
        assert!(
            !f.observe(&encode(1, &[1]).unwrap())
                .unwrap()
                .payload_timing_valid
        );
        assert_eq!(
            f.observe(&encode(1, &[]).unwrap()).unwrap().notify_through,
            Some(16)
        );
        assert_eq!(f.pending_frames(), 0);
    }

    #[test]
    fn exhausted_flight_bounds_and_wrap_windows() {
        for sequence in 1..=64 {
            let mut f = Flight::established(sequence);
            for p in 0..FRAME_WINDOW {
                f.send(&[p as u8; MAX_PAYLOAD], 0, write).unwrap();
            }
            assert!(matches!(
                f.send(&[0], 0, |_| panic!()),
                Err(SendError::Backpressure)
            ));
            assert_eq!(
                f.observe(&encode((sequence + 13) as u8, &[]).unwrap()),
                Err(ReceiveError::OutsideWindow)
            );
            assert_eq!(
                f.observe(&encode((sequence - 1) as u8, &[]).unwrap()),
                Err(ReceiveError::OutsideWindow)
            );
            f.retry(Retry::Timeout, |bytes| {
                let expected = (0..FRAME_WINDOW).flat_map(|p| {
                    encode((sequence + p as u64) as u8, &[p as u8; MAX_PAYLOAD]).unwrap()
                });
                assert!(bytes
                    .iter()
                    .copied()
                    .eq(std::iter::once(SYNC).chain(expected)));
                write(bytes)
            })
            .unwrap();
            assert!(
                !f.observe(&encode(sequence as u8, &[]).unwrap())
                    .unwrap()
                    .fast_retry
            );
            assert_eq!(
                f.observe(&encode((sequence + 12) as u8, &[]).unwrap())
                    .unwrap()
                    .retired,
                12
            );
            assert_eq!(f.outstanding_bytes(), 0);
        }
    }

    #[test]
    fn bad_frames_and_failed_writes_are_isolated() {
        let mut f = Flight::established(1);
        assert!(matches!(
            f.send(&[], 0, |_| panic!()),
            Err(SendError::InvalidPayload)
        ));
        let mut bad = encode(1, &[]).unwrap();
        bad[2] ^= 1;
        assert_eq!(f.observe(&bad), Err(ReceiveError::InvalidFrame));
        assert_eq!(
            f.observe(&encode(2, &[]).unwrap()),
            Err(ReceiveError::OutsideWindow)
        );
        assert!(matches!(
            f.send(&[42], 0, |_| Ok(2)),
            Err(SendError::Write(_))
        ));
        assert!(f.is_closed());
        assert_eq!(f.pending_frames(), 0);
        assert!(matches!(
            f.send(&[42], 0, |_| panic!()),
            Err(SendError::Closed)
        ));
        assert_eq!(
            f.observe(&encode(2, &[]).unwrap()),
            Err(ReceiveError::Closed)
        );

        let mut f = Flight::established(u64::MAX - 1);
        assert!(f.send(&[42], 0, write).is_ok());
        assert!(matches!(
            f.send(&[42], 0, |_| panic!()),
            Err(SendError::SequenceExhausted)
        ));
        assert!(matches!(
            f.retry(Retry::Timeout, |_| Err(io::ErrorKind::BrokenPipe.into())),
            Err(SendError::Write(_))
        ));
        assert!(f.is_closed());
        assert_eq!(f.pending_frames(), 1);
        assert!(matches!(
            f.retry(Retry::Timeout, |_| panic!()),
            Err(SendError::Closed)
        ));
    }
}
