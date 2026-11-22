// SPDX-License-Identifier: GPL-3.0-only
//! Bounded receive-stream reassembly with Klippy's sync recovery.
//!
//! Recovery follows msgblock_check in klippy/chelper/msgblock.c.
//! Upstream copyright (C) 2016-2021 Kevin O'Connor <kevin@koconnor.net>.

use crate::{decode_prefix, Frame, MAX_FRAME, MIN_FRAME, SYNC};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Received {
    pub frames: usize,
    pub discarded: usize,
}

/// A stream decoder. Keeps at most 63 bytes between reads and allocates no heap
/// memory. Complete frames in a contiguous input slice do not need copying.
pub struct Receiver {
    spare: [u8; MAX_FRAME],
    used: usize,
    need_sync: bool,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

impl Receiver {
    pub const fn new() -> Self {
        Self {
            spare: [0; MAX_FRAME],
            used: 0,
            need_sync: false,
        }
    }

    pub fn pending(&self) -> &[u8] {
        &self.spare[..self.used]
    }

    pub const fn seeking_sync(&self) -> bool {
        self.need_sync
    }

    /// Process all available input, calling `on_frame` with each complete,
    /// validated frame in order (including ACKs). The callback must not
    /// panic if the receiver will be reused. Partial input remains pending.
    pub fn feed(&mut self, mut input: &[u8], mut on_frame: impl FnMut(Frame<'_>)) -> Received {
        let mut out = Received::default();
        let mut start = 0;
        while start < self.used {
            let n = input.len().min(self.spare.len() - self.used);
            self.spare[self.used..self.used + n].copy_from_slice(&input[..n]);
            self.used += n;
            input = &input[n..];
            let buffered = &self.spare[start..self.used];
            match classify(self.need_sync, buffered) {
                Action::Wait => {
                    if !input.is_empty() {
                        // The compacted tail can take another read immediately.
                        self.spare.copy_within(start..self.used, 0);
                        self.used -= start;
                        start = 0;
                        continue;
                    }
                    self.spare.copy_within(start..self.used, 0);
                    self.used -= start;
                    return out;
                }
                Action::Frame(frame, n) => {
                    out.frames += 1;
                    on_frame(frame);
                    start += n;
                }
                Action::Recover => {
                    if let Some(index) = sync_end(buffered) {
                        self.need_sync = false;
                        start += index;
                        out.discarded += index;
                    } else {
                        // A recovery step searches the whole logical read, not
                        // just the part that fitted in the temporary buffer.
                        out.discarded += buffered.len();
                        start = self.used;
                        self.skip_to_sync(&mut input, &mut out);
                    }
                }
            }
        }
        self.used = 0;
        while !input.is_empty() {
            match classify(self.need_sync, input) {
                Action::Wait => {
                    debug_assert!(input.len() < MAX_FRAME);
                    self.spare[..input.len()].copy_from_slice(input);
                    self.used = input.len();
                    break;
                }
                Action::Frame(frame, n) => {
                    out.frames += 1;
                    on_frame(frame);
                    input = &input[n..];
                }
                Action::Recover => self.skip_to_sync(&mut input, &mut out),
            }
        }
        out
    }

    fn skip_to_sync(&mut self, input: &mut &[u8], out: &mut Received) {
        let sync = sync_end(input);
        self.need_sync = sync.is_none();
        let n = sync.unwrap_or(input.len());
        out.discarded += n;
        *input = &input[n..];
    }
}

enum Action<'a> {
    Wait,
    Frame(Frame<'a>, usize),
    Recover,
}

fn classify(need_sync: bool, input: &[u8]) -> Action<'_> {
    if input.len() < MIN_FRAME {
        return Action::Wait;
    }
    if need_sync {
        return Action::Recover;
    }
    match decode_prefix(input) {
        Ok(Some((frame, n))) => Action::Frame(frame, n),
        Ok(None) => Action::Wait,
        Err(_) => Action::Recover,
    }
}

fn sync_end(input: &[u8]) -> Option<usize> {
    input.iter().position(|&b| b == SYNC).map(|i| i + 1)
}
