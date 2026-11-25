// SPDX-License-Identifier: GPL-3.0-only
//! Offline virtual SD byte checkpoints, adapted from Klipper's virtual_sdcard.py.
//! Upstream copyright (C) 2018-2025 Kevin O'Connor <kevin@koconnor.net>.
//! The caller supplies an immutable file snapshot and drives job progression.

const MAX_LINE_BYTES: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Selected,
    Running,
    Paused,
    Completed,
    Cancelled,
    Faulted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    State,
    Seek,
    Encoding,
    LineTooLong,
    StaleToken,
    TokenLimit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    pub state: State,
    pub position: usize,
    pub size: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct Line<'a> {
    pub start: usize,
    pub next: usize,
    pub token: u64,
    /// Exact text excluding LF; a preceding CR and UTF-8 remain intact.
    pub source: &'a str,
}

pub struct Job<'a> {
    source: &'a [u8],
    position: usize,
    state: State,
    generation: u64,
    waiting: Option<Line<'a>>,
}

impl<'a> Job<'a> {
    pub fn select(source: &'a [u8]) -> Self {
        Self {
            source,
            position: 0,
            state: State::Selected,
            generation: 0,
            waiting: None,
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            state: self.state,
            position: self.position,
            size: self.source.len(),
        }
    }

    /// Byte progress at the last commit. Issuing a line does not advance
    /// this value, and cancellation is exposed as an empty SD slot.
    pub fn committed_progress(&self) -> f64 {
        if self.state == State::Cancelled {
            0.0
        } else {
            fraction(self.position, self.source.len())
        }
    }

    /// Render a Klippy-compatible read-only status object for this in-memory
    /// job. The supplied path is metadata only; it is never opened.
    #[cfg(feature = "webhooks")]
    pub fn publish_status(&self, file_path: &str, output: &mut crate::webhooks::Snapshot) -> bool {
        if file_path.is_empty() || file_path.len() > 512 || file_path.contains('\0') {
            return false;
        }
        let cancelled = self.state == State::Cancelled;
        let position = if cancelled { 0 } else { self.position };
        let size = if cancelled { 0 } else { self.source.len() };
        let attached = !matches!(self.state, State::Completed | State::Cancelled);
        let active =
            self.state == State::Running || (self.state == State::Paused && self.waiting.is_some());
        project_status(
            output,
            attached.then_some(file_path),
            active,
            position,
            size,
        )
    }

    pub fn resume(&mut self) -> Result<(), Error> {
        if !matches!(self.state, State::Selected | State::Paused) || self.waiting.is_some() {
            return Err(Error::State);
        }
        self.state = State::Running;
        Ok(())
    }

    pub fn pause(&mut self) -> Result<(), Error> {
        if self.state != State::Running {
            return Err(Error::State);
        }
        // An already-dispatched line can still commit; no new lines will be issued.
        self.state = State::Paused;
        Ok(())
    }

    pub fn cancel(&mut self) -> Result<(), Error> {
        if !matches!(self.state, State::Selected | State::Paused | State::Running) {
            return Err(Error::State);
        }
        self.waiting = None;
        self.state = State::Cancelled;
        Ok(())
    }

    pub fn seek_paused(&mut self, position: usize) -> Result<(), Error> {
        if !matches!(self.state, State::Selected | State::Paused) || self.waiting.is_some() {
            return Err(Error::State);
        }
        self.check_boundary(position)?;
        self.position = position;
        Ok(())
    }

    fn check_boundary(&self, position: usize) -> Result<(), Error> {
        if position > self.source.len()
            || (position != 0
                && position != self.source.len()
                && self.source[position - 1] != b'\n')
        {
            return Err(Error::Seek);
        }
        Ok(())
    }

    pub fn issue(&mut self) -> Result<Option<Line<'a>>, Error> {
        if self.state != State::Running || self.waiting.is_some() {
            return Err(Error::State);
        }
        let rest = &self.source[self.position..];
        // Inspect no more than one bounded line plus its separator.
        let nul = rest
            .iter()
            .take(MAX_LINE_BYTES + 1)
            .position(|&v| v == b'\n');
        let Some(end) = nul else {
            if rest.len() > MAX_LINE_BYTES {
                self.state = State::Faulted;
                return Err(Error::LineTooLong);
            }
            // Stock dispatches LF-delimited commands only. Its trailing
            // unterminated buffer is dropped without advancing the checkpoint.
            self.state = State::Completed;
            return Ok(None);
        };
        let Ok(source) = std::str::from_utf8(&rest[..end]) else {
            self.state = State::Faulted;
            return Err(Error::Encoding);
        };
        if source.contains('\0') {
            self.state = State::Faulted;
            return Err(Error::Encoding);
        }
        let Some(generation) = self.generation.checked_add(1) else {
            self.state = State::Faulted;
            return Err(Error::TokenLimit);
        };
        self.generation = generation;
        let next = self.position + end + 1;
        let line = Line {
            start: self.position,
            next,
            token: generation,
            source,
        };
        self.waiting = Some(line);
        Ok(Some(line))
    }

    /// Advance a committed line, or accept an explicit loop/skip to another
    /// line boundary (as stock's set_file_position can be used by a macro).
    pub fn commit(&mut self, token: u64, seek: Option<usize>) -> Result<usize, Error> {
        let pending = self
            .waiting
            .filter(|line| line.token == token)
            .ok_or(Error::StaleToken)?;
        let target = seek.unwrap_or(pending.next);
        self.check_boundary(target)?;
        self.waiting = None;
        self.position = target;
        Ok(self.position)
    }

    pub fn fail(&mut self, token: u64) -> Result<(), Error> {
        if !self.waiting.is_some_and(|line| line.token == token) {
            return Err(Error::StaleToken);
        }
        self.waiting = None;
        self.state = State::Faulted;
        Ok(())
    }
}

/// Read-only reset/unselected shape. The caller owns clearing its selected
/// job; this function does not cancel it, open a file, or dispatch commands.
#[cfg(feature = "webhooks")]
pub fn publish_unselected_status(output: &mut crate::webhooks::Snapshot) -> bool {
    project_status(output, None, false, 0, 0)
}

#[cfg(feature = "webhooks")]
fn project_status(
    output: &mut crate::webhooks::Snapshot,
    path: Option<&str>,
    active: bool,
    position: usize,
    size: usize,
) -> bool {
    let fields = serde_json::json!({
        "file_path": path, "progress": fraction(position, size),
        "is_active": active, "file_position": position, "file_size": size,
    });
    output.set_object("virtual_sdcard".into(), fields.as_object().unwrap().clone())
}

fn fraction(position: usize, size: usize) -> f64 {
    if size == 0 {
        0.0
    } else {
        position as f64 / size as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracks_utf8_byte_positions_and_drops_the_unterminated_tail() {
        let mut sd = Job::select("; 한글\r\nG90\nM400".as_bytes());
        assert_eq!(sd.seek_paused(3), Err(Error::Seek));
        sd.resume().unwrap();
        let one = sd.issue().unwrap().unwrap();
        assert_eq!(one.source, "; 한글\r");
        assert_eq!(one.next, 10);
        assert_eq!(sd.snapshot().position, 0);
        sd.pause().unwrap();
        assert_eq!(sd.commit(one.token, None), Ok(10));
        sd.resume().unwrap();
        let two = sd.issue().unwrap().unwrap();
        assert_eq!(sd.commit(one.token, None), Err(Error::StaleToken));
        assert_eq!(sd.commit(two.token, None), Ok(14));
        assert!(sd.issue().unwrap().is_none());
        assert_eq!(sd.snapshot().state, State::Completed);
        assert_eq!(sd.snapshot().position, 14);
    }

    #[test]
    fn verifies_controlled_rewind_and_failures_remain_sticky() {
        let mut sd = Job::select(b"G90\nG91\n");
        sd.resume().unwrap();
        let one = sd.issue().unwrap().unwrap();
        sd.commit(one.token, None).unwrap();
        let two = sd.issue().unwrap().unwrap();
        assert_eq!(sd.commit(two.token, Some(1)), Err(Error::Seek));
        assert_eq!(sd.commit(two.token, Some(0)), Ok(0));
        let repeated = sd.issue().unwrap().unwrap();
        assert!(repeated.token > two.token);
        sd.fail(repeated.token).unwrap();
        assert_eq!(sd.snapshot().position, 0);
        assert_eq!(sd.resume(), Err(Error::State));
    }
}
