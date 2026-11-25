// SPDX-License-Identifier: GPL-3.0-only
//! Replay bounded generation and transmission windows with fixed shaping.
use super::{Phases, Shaper, MAX_EVENTS};
use crate::{
    extrusion::prune_prepared,
    step_clock::{self, Clock, Command, TraceEncoder},
    step_times::{self, Axis, Event, Solver},
    trajectory::{Trajectory, Trapezoid},
};

pub(crate) enum Capture {
    Bind([f64; 3]),
    Profile(Trapezoid),
    Generate(f64),
    Transmit(f64),
    Retire(f64),
}

#[derive(Debug)]
pub(crate) enum ReplayError {
    Options,
    Trajectory(crate::trajectory::Error),
    Solver(step_times::Error),
    Clock(step_clock::Error),
}

struct WindowSolver {
    solver: Solver,
    last_flush: f64,
    last_move: f64,
    lane: Axis,
}

impl WindowSolver {
    fn window(
        &mut self,
        shaper: &Shaper,
        timeline: &Trajectory,
        until: f64,
        out: &mut Vec<Event>,
    ) -> Result<(), ReplayError> {
        if !until.is_finite() || until < self.last_flush {
            return Err(ReplayError::Options);
        }
        let start = self.last_flush;
        self.last_flush = until;
        let Some(phases) = Phases::new(timeline) else {
            return Ok(());
        };
        let (pre, post) = shaper.window(self.lane);
        let mut force = self.last_move + post;
        let mut skipped = 0;
        let first = phases.segments.partition_point(|s| s.end() <= start) as isize;
        let lane = self.lane;
        let mut solve = |index, lo: f64, hi: f64| {
            let phase = phases.pick(index);
            if hi.min(phase.end()) <= lo.max(phase.at) {
                return Ok(());
            }
            let room = MAX_EVENTS - out.len();
            if room == 0 {
                return Err(ReplayError::Solver(step_times::Error::EventLimit));
            }
            self.solver
                .solve_range_into(phase, lo..hi, room, out, |relative| {
                    shaper
                        .sample_relative(&phases, lane, index, relative)
                        .map_err(|_| step_times::Error::Invalid)
                })
                .map_err(ReplayError::Solver)
        };
        for i in first..=phases.segments.len() as isize {
            let phase = phases.pick(i);
            if super::selected(lane)
                .iter()
                .any(|&axis| phase.direction[axis] != 0.)
            {
                if skipped > 0 && pre > 0. {
                    let lead = (phase.at - pre).max(start).max(force);
                    let mut previous = i - 1;
                    while previous > i - skipped && phases.pick(previous).at > lead {
                        previous -= 1;
                    }
                    for j in previous..i {
                        solve(j, lead, until)?;
                    }
                }
                solve(i, start, until)?;
                if phase.end() >= until {
                    self.last_move = until;
                    break;
                }
                skipped = 0;
                self.last_move = phase.end();
                force = self.last_move + post;
            } else {
                if phase.at < force {
                    solve(i, start, force.min(until))?;
                    skipped = 1;
                } else {
                    skipped += 1;
                }
                if until + pre <= phase.end() {
                    break;
                }
            }
        }
        Ok(())
    }
}

#[cfg(feature = "runtime-shaper-preview")]
#[derive(Clone, Copy)]
pub(crate) struct ReplayUpdates<'a> {
    #[cfg(feature = "runtime-pickup-extended-preview")]
    pub(crate) extended_pickup: bool,
    pub(crate) positions: &'a [(usize, usize)],
    pub(crate) shapes: &'a [Shaper],
}

pub(crate) fn replay<const N: usize>(
    shaper: &Shaper,
    events: &[Capture],
    lanes: [(Axis, f64, bool); N],
    hz: f64,
    error: u32,
    #[cfg(feature = "runtime-shaper-preview")] updates: Option<ReplayUpdates<'_>>,
) -> Result<[Vec<Command>; N], ReplayError> {
    let mut out = std::array::from_fn(|_| Vec::new());
    let clock = Clock {
        last_encoded: 0,
        time_origin: 0.,
        hz,
    };
    for ((lane, distance, invert), result) in lanes.into_iter().zip(&mut out) {
        let mut timeline = Trajectory::new(1024).map_err(ReplayError::Trajectory)?;
        let mut state = WindowSolver {
            solver: Solver::new(lane, distance, [0.; 3]).map_err(ReplayError::Solver)?,
            last_flush: 0.,
            last_move: 0.,
            lane,
        };
        let mut encoder = TraceEncoder::new(clock, error, invert);
        #[cfg(feature = "runtime-pickup-extended-preview")]
        if updates.is_some_and(|u| u.extended_pickup) {
            encoder = encoder.extended_pickup();
        }
        let mut scratch = Vec::new();
        let mut prior = 0.;
        #[cfg(feature = "runtime-shaper-preview")]
        let mut shaper = shaper;
        #[cfg(feature = "runtime-shaper-preview")]
        let mut change = 0;
        #[cfg_attr(
            not(feature = "runtime-shaper-preview"),
            allow(clippy::unused_enumerate_index)
        )]
        for (_index, event) in events.iter().enumerate() {
            #[cfg(feature = "runtime-shaper-preview")]
            if let Some(ReplayUpdates {
                positions, shapes, ..
            }) = updates
            {
                while let Some(&(_, idx)) = positions.get(change).filter(|p| p.0 == _index) {
                    shaper = shapes.get(idx).ok_or(ReplayError::Options)?;
                    change += 1;
                }
            }
            match *event {
                Capture::Bind(position) => {
                    if !timeline.segments().is_empty() {
                        return Err(ReplayError::Options);
                    }
                    // Stock sets shaped commanded position by sampling a
                    // stationary synthetic move, not by bypassing the pulses.
                    let mut bound = position;
                    for &axis in super::selected(lane) {
                        let p = &shaper.axes[axis];
                        if p.len != 0 {
                            bound[axis] = p.values[..p.split]
                                .iter()
                                .rev()
                                .chain(&p.values[p.split..p.len])
                                .fold(0., |v, pulse| v + pulse.amplitude * position[axis]);
                        }
                    }
                    state
                        .solver
                        .rebase(lane.position(bound))
                        .map_err(ReplayError::Solver)?;
                }
                Capture::Profile(profile) => {
                    timeline.append(profile).map_err(ReplayError::Trajectory)?
                }
                Capture::Generate(until) => {
                    scratch.clear();
                    state.window(shaper, &timeline, until, &mut scratch)?;
                    encoder.push(&scratch).map_err(ReplayError::Clock)?;
                }
                Capture::Transmit(until) => {
                    if until < prior {
                        return Err(ReplayError::Options);
                    }
                    prior = until;
                    let clock =
                        step_clock::captured_cutoff(until, hz).map_err(ReplayError::Clock)?;
                    encoder.flush_before(clock).map_err(ReplayError::Clock)?;
                }
                Capture::Retire(until) => {
                    prune_prepared(&mut timeline, until).map_err(ReplayError::Trajectory)?;
                }
            }
        }
        *result = encoder.finish().map_err(ReplayError::Clock)?;
    }
    Ok(out)
}
