// SPDX-License-Identifier: GPL-3.0-only
//! Stop-and-wait state machine for idempotent MCU reads. Not a command scheduler.

use std::time::Duration;

const ACK_WAIT: Duration = Duration::from_millis(25);
const RESPONSE_WAIT: Duration = Duration::from_millis(10);
const OVERALL_WAIT: Duration = Duration::from_secs(1);
const MAX_FRAME_RETRIES: u32 = 3;
const MAX_RESPONSE_RETRIES: u32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Failure {
    NoAck,
    NoResponse,
    Deadline,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Prefix a sync marker before retrying an unacknowledged wire frame.
    Send {
        sequence: u8,
        prepend_sync: bool,
    },
    WaitUntil(Duration),
    Done {
        next_sequence: u8,
    },
    Failed(Failure),
}

enum State {
    Ready,
    Ack(Duration),
    Response(Duration),
    Done,
    Failed(Failure),
}

/// Query state for single-in-flight read-only commands. Times are monotonic
/// durations since this query began, supplied by the transport. The transport
/// must bound I/O by throughput/deadline and discard the session after failure.
/// Like stock Klippy, an unacknowledged frame is resent with the same sequence,
/// while a missing response to an acknowledged read is retried as a new frame.
/// The fixed timer bounds here are deliberately not Klippy's adaptive RTO.
pub struct ReadQuery {
    state: State,
    sequence: u8,
    frame_retries: u32,
    response_retries: u32,
}

impl ReadQuery {
    pub const fn new(sequence: u8) -> Self {
        Self {
            state: State::Ready,
            sequence: sequence & 15,
            frame_retries: 0,
            response_retries: 0,
        }
    }

    /// Only the MCU's next-expected sequence can acknowledge this request.
    pub fn expects(&self, sequence: u8) -> bool {
        matches!(self.state, State::Ack(_) | State::Response(_)) && sequence == self.next_sequence()
    }

    /// Observe an in-order frame. A matching reply completes the read; an ACK
    /// or other valid frame carrying the expected sequence proves consumption
    /// but does not supply the reply.
    pub fn observe(&mut self, now: Duration, sequence: u8, response_matches: bool) {
        if now >= OVERALL_WAIT || !self.expects(sequence) {
            return;
        }
        if response_matches {
            self.state = State::Done;
        } else if matches!(self.state, State::Ack(_)) {
            self.state = State::Response(
                (now + RESPONSE_WAIT * (1 << self.response_retries)).min(OVERALL_WAIT),
            );
        }
    }

    pub fn poll(&mut self, now: Duration) -> Action {
        match self.state {
            State::Done => {
                return Action::Done {
                    next_sequence: self.next_sequence(),
                }
            }
            State::Failed(failure) => return Action::Failed(failure),
            _ => {}
        }
        if now >= OVERALL_WAIT {
            return self.fail(Failure::Deadline);
        }
        match self.state {
            State::Ready => self.send(now, false),
            State::Ack(at) if now >= at => {
                if self.frame_retries == MAX_FRAME_RETRIES {
                    return self.fail(Failure::NoAck);
                }
                self.frame_retries += 1;
                self.send(now, true)
            }
            State::Response(at) if now >= at => {
                if self.response_retries == MAX_RESPONSE_RETRIES {
                    return self.fail(Failure::NoResponse);
                }
                self.response_retries += 1;
                self.frame_retries = 0;
                self.sequence = self.next_sequence();
                self.send(now, false)
            }
            State::Ack(at) | State::Response(at) => Action::WaitUntil(at),
            State::Done | State::Failed(_) => unreachable!(),
        }
    }

    const fn next_sequence(&self) -> u8 {
        (self.sequence + 1) & 15
    }

    fn fail(&mut self, failure: Failure) -> Action {
        self.state = State::Failed(failure);
        Action::Failed(failure)
    }

    fn send(&mut self, now: Duration, prepend_sync: bool) -> Action {
        self.state = State::Ack((now + ACK_WAIT * (1 << self.frame_retries)).min(OVERALL_WAIT));
        Action::Send {
            sequence: self.sequence,
            prepend_sync,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn retries_unacknowledged_frame_at_same_sequence() {
        let mut q = ReadQuery::new(15);
        assert_eq!(
            q.poll(ms(0)),
            Action::Send {
                sequence: 15,
                prepend_sync: false
            }
        );
        q.observe(ms(1), 15, false);
        assert_eq!(q.poll(ms(24)), Action::WaitUntil(ms(25)));
        assert_eq!(
            q.poll(ms(25)),
            Action::Send {
                sequence: 15,
                prepend_sync: true
            }
        );
        q.observe(ms(26), 0, false);
        assert_eq!(q.poll(ms(26)), Action::WaitUntil(ms(36)));
        q.observe(ms(28), 0, true);
        assert_eq!(q.poll(ms(28)), Action::Done { next_sequence: 0 });
    }

    #[test]
    fn retries_acknowledged_read_with_new_sequence_and_ignores_old_reply() {
        let mut q = ReadQuery::new(15);
        q.poll(ms(0));
        q.observe(ms(1), 0, false);
        assert_eq!(
            q.poll(ms(11)),
            Action::Send {
                sequence: 0,
                prepend_sync: false
            }
        );
        q.observe(ms(12), 0, true);
        assert_eq!(q.poll(ms(12)), Action::WaitUntil(ms(36)));
        q.observe(ms(13), 1, true);
        assert_eq!(q.poll(ms(13)), Action::Done { next_sequence: 1 });
    }

    #[test]
    fn bounds_link_and_response_retries() {
        let mut q = ReadQuery::new(0);
        q.poll(ms(0));
        for at in [25, 75, 175] {
            assert_eq!(
                q.poll(ms(at)),
                Action::Send {
                    sequence: 0,
                    prepend_sync: true
                }
            );
        }
        assert_eq!(q.poll(ms(375)), Action::Failed(Failure::NoAck));
        assert_eq!(q.poll(ms(500)), Action::Failed(Failure::NoAck));

        let mut q = ReadQuery::new(0);
        for (seq, at) in [0, 10, 30, 70, 150, 310].into_iter().enumerate() {
            let seq = seq as u8;
            assert_eq!(
                q.poll(ms(at)),
                Action::Send {
                    sequence: seq,
                    prepend_sync: false
                }
            );
            q.observe(ms(at), seq + 1, false);
        }
        assert_eq!(q.poll(ms(630)), Action::Failed(Failure::NoResponse));

        let mut q = ReadQuery::new(0);
        q.poll(ms(0));
        q.observe(ms(1000), 1, true);
        assert_eq!(q.poll(ms(1000)), Action::Failed(Failure::Deadline));
    }
}
