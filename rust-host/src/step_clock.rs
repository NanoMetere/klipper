// SPDX-License-Identifier: GPL-3.0-only
//! Offline step clock translation and reversal filter, adapted from stock
//! Klipper chelper/stepcompress.c. Copyright (C) 2016-2025 Kevin O'Connor.
//! The output is a trace only. This does not accept or open a transport.

use crate::{
    step_compress::{compress_each, Block},
    step_times::Event,
};

const REVERSAL_WINDOW: f64 = 0.000750;
const MAX_EVENTS: usize = 65_535;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidTime,
    Nonmonotonic,
    Capacity,
    Compress(crate::step_compress::Error),
}

#[derive(Debug, Clone, Copy)]
pub struct Clock {
    pub last_encoded: u64,
    pub time_origin: f64,
    pub hz: f64,
}

impl Clock {
    /// The order of operations and half-tick offset follow stock. Do not
    /// round absolute seconds before subtraction (loses boundary precision).
    pub fn to_tick(self, segment_at: f64, step_offset: f64) -> Result<u64, Error> {
        if !self.time_origin.is_finite()
            || !(1.0..=1e12).contains(&self.hz)
            || !segment_at.is_finite()
            || !step_offset.is_finite()
            || step_offset < 0.0
            || self.last_encoded >= (1 << 53)
        {
            return Err(Error::InvalidTime);
        }
        let last_time = self.time_origin + (self.last_encoded as f64 - 0.5) / self.hz;
        let relative = (step_offset + (segment_at - last_time)) * self.hz;
        if !(0.0..(1_u64 << 53) as f64).contains(&relative) {
            return Err(Error::InvalidTime);
        }
        let tick = self.last_encoded + relative as u64;
        if tick == 0 || tick >= (1 << 53) {
            return Err(Error::InvalidTime);
        }
        Ok(tick)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Step {
    pub clock: u64,
    pub positive: bool,
}

#[derive(Default)]
struct Filter {
    pending: Option<Step>,
    last_seen: Option<u64>,
}

impl Filter {
    fn append(&mut self, step: Step, hz: f64) -> Result<Option<Step>, Error> {
        if self.last_seen.is_some_and(|seen| step.clock <= seen) {
            return Err(Error::Nonmonotonic);
        }
        self.last_seen = Some(step.clock);
        let previous = self.pending.take();
        if previous.is_some_and(|p| {
            p.positive != step.positive && ((step.clock - p.clock) as f64) < REVERSAL_WINDOW * hz
        }) {
            return Ok(None);
        }
        self.pending = Some(step);
        Ok(previous)
    }

    fn commit(&mut self) -> Option<Step> {
        self.pending.take()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    SetDirection(bool),
    Steps(Block),
}

/// Encode a short, bounded raw-event trace. Committing a direction change
/// emits the preceding run and rebases the clock just like stock. This API
/// has no incremental scheduler, far-future path, or queue-flush deadlines.
pub fn encode_trace(
    mut clock: Clock,
    events: &[Event],
    max_error: u32,
    invert: bool,
) -> Result<Vec<Command>, Error> {
    if events.len() > MAX_EVENTS {
        return Err(Error::Capacity);
    }
    let mut filter = Filter::default();
    let mut output = Vec::new();
    let mut run = Vec::new();
    let mut direction = None;
    let flush = |clock: &mut Clock, output: &mut Vec<Command>, run: &mut Vec<u64>| {
        if !run.is_empty() {
            compress_each(clock.last_encoded, run, max_error, |block| {
                clock.last_encoded = block.last_clock;
                output.push(Command::Steps(block));
            })
            .map_err(Error::Compress)?;
            run.clear();
        }
        Ok(())
    };
    let commit = |s: Step,
                  clock: &mut Clock,
                  output: &mut Vec<Command>,
                  run: &mut Vec<u64>,
                  direction: &mut Option<bool>| {
        if *direction != Some(s.positive) {
            flush(clock, output, run)?;
            output.push(Command::SetDirection(s.positive ^ invert));
            *direction = Some(s.positive);
        }
        run.push(s.clock);
        Ok(())
    };
    for e in events {
        let send = match *e {
            Event::Commit => filter.commit(),
            Event::Crossing {
                direction_positive,
                segment_time,
                offset,
            } => filter.append(
                Step {
                    clock: clock.to_tick(segment_time, offset)?,
                    positive: direction_positive,
                },
                clock.hz,
            )?,
        };
        if let Some(step) = send {
            commit(step, &mut clock, &mut output, &mut run, &mut direction)?;
        }
    }
    if let Some(step) = filter.commit() {
        commit(step, &mut clock, &mut output, &mut run, &mut direction)?;
    }
    flush(&mut clock, &mut output, &mut run)?;
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step(tick: i32, positive: bool) -> Event {
        Event::Crossing {
            direction_positive: positive,
            segment_time: 0.,
            offset: f64::from(tick) / 1_000_000.,
        }
    }

    #[test]
    fn cancels_uncommitted_reverse_but_keeps_exact_cutoff_and_hints() {
        let clock = Clock {
            last_encoded: 0,
            time_origin: 0.0,
            hz: 1_000_000.0,
        };
        let events = [
            step(1_000, true),
            step(1_749, false), // cancel both
            step(2_000, true),
            Event::Commit,
            step(2_001, false), // can't cancel
            step(3_000, false),
            step(3_750, true),
        ]; // threshold equal, no cancel
        let output = encode_trace(clock, &events, 0, false).unwrap();
        let directions: Vec<_> = output
            .iter()
            .filter_map(|cmd| match *cmd {
                Command::SetDirection(v) => Some(v),
                _ => None,
            })
            .collect();
        assert_eq!(directions, [true, false, true]);
        assert_eq!(
            output
                .iter()
                .map(|cmd| match cmd {
                    Command::Steps(b) => usize::from(b.count),
                    _ => 0,
                })
                .sum::<usize>(),
            4
        );
    }

    #[test]
    fn preserves_high_epoch_and_rejects_invalid_input() {
        let clock = Clock {
            last_encoded: 1 << 32,
            time_origin: 10.25,
            hz: 1_000_000.0,
        };
        let representation = 10.25 + ((1u64 << 32) as f64 + 41.0) / 1_000_000.0;
        assert_eq!(clock.to_tick(representation, 0.), Ok((1 << 32) + 41));
        assert_eq!(clock.to_tick(representation, -0.1), Err(Error::InvalidTime));
    }
}
