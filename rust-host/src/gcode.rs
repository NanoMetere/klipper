// SPDX-License-Identifier: GPL-3.0-only
//! Strict offline subset of klippy/gcode.py and klippy/extras/gcode_move.py.
//! Upstream copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>.
//! Not a command dispatcher or a hardware control interface.

use crate::lookahead::{self, Limits, LookAhead, Move, Profile};
use std::collections::BTreeMap;

#[path = "gcode/arcs.rs"]
mod arcs;
#[path = "gcode/extended.rs"]
mod extended;
#[path = "gcode/transform.rs"]
mod transform;
pub use transform::Owner as TransformOwner;

const MAX_LINE: usize = 4096;
const AXES: [u8; 4] = *b"XYZE";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidLine,
    UnsupportedCommand,
    InvalidParameter,
    DuplicateParameter,
    Nonfinite,
    InvalidSession,
    MoveOutsidePreview,
    UnknownState,
    StateLimit,
    TransformOwner,
    TransformLimit,
    ArcLimit,
    Motion(lookahead::Error),
}

impl From<lookahead::Error> for Error {
    fn from(error: lookahead::Error) -> Self {
        Self::Motion(error)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Effect {
    None,
    Linear { position: [f64; 4], speed: f64 },
    Barrier,
    Dwell(f64),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    base: [f64; 4],
    last: [f64; 4],
    homing: [f64; 4],
    absolute_xyz: bool,
    absolute_e_allowed: bool,
    speed: f64,
    speed_factor: f64,
    flow: f64,
}

impl Default for State {
    fn default() -> Self {
        Self {
            base: [0.0; 4],
            last: [0.0; 4],
            homing: [0.0; 4],
            absolute_xyz: true,
            absolute_e_allowed: true,
            speed: 25.0,
            speed_factor: 1.0 / 60.0,
            flow: 1.0,
        }
    }
}

impl State {
    pub fn position(&self) -> [f64; 4] {
        self.last
    }

    pub fn gcode_position(&self) -> [f64; 4] {
        let mut pos = std::array::from_fn(|i| self.last[i] - self.base[i]);
        pos[3] /= self.flow;
        pos
    }

    pub fn homing_origin(&self) -> [f64; 4] {
        self.homing
    }

    pub fn physical_feed_rate(&self) -> f64 {
        self.speed
    }

    /// Evaluate one traditional line. Reject unsupported/ambiguous input rather
    /// than silently ignoring it. Errors leave coordinate state untouched.
    pub fn apply(&mut self, line: &str) -> Result<Effect, Error> {
        let Some(words) = parse(line)? else {
            return Ok(Effect::None);
        };
        let mut next = *self;
        let result = match (words.family, words.code) {
            (b'G', 0 | 1) => {
                words.allow(b"XYZEF")?;
                next.linear(AXES.map(|axis| words.get(axis)), words.get(b'F'))?
            }
            (b'G', 92) => {
                words.allow(b"XYZE")?;
                if words.args.iter().all(Option::is_none) {
                    next.base = next.last;
                } else {
                    for (i, axis) in AXES.into_iter().enumerate() {
                        if let Some(value) = words.get(axis) {
                            next.base[i] =
                                next.last[i] - if i == 3 { value * next.flow } else { value };
                        }
                    }
                }
                Effect::None
            }
            (b'G', 21 | 90 | 91) => {
                words.allow(b"")?;
                if words.code != 21 {
                    next.absolute_xyz = words.code == 90;
                }
                Effect::None
            }
            (b'M', 82 | 83) => {
                words.allow(b"")?;
                next.absolute_e_allowed = words.code == 82;
                Effect::None
            }
            (b'M', 220 | 221) => {
                words.allow(b"S")?;
                let percentage = words.get(b'S').unwrap_or(100.0);
                if percentage <= 0.0 {
                    return Err(Error::InvalidParameter);
                }
                if words.code == 220 {
                    let factor = percentage / 6000.0;
                    next.speed = (next.speed / next.speed_factor) * factor;
                    next.speed_factor = factor;
                } else {
                    let factor = percentage / 100.0;
                    let e = (next.last[3] - next.base[3]) / next.flow;
                    next.base[3] = next.last[3] - e * factor;
                    next.flow = factor;
                }
                Effect::None
            }
            (b'G', 4) => {
                words.allow(b"P")?;
                let ms = words.get(b'P').unwrap_or(0.0);
                if ms < 0.0 {
                    return Err(Error::InvalidParameter);
                }
                Effect::Dwell(ms / 1000.0)
            }
            (b'M', 400) => {
                words.allow(b"")?;
                Effect::Barrier
            }
            _ => return Err(Error::UnsupportedCommand),
        };
        next.validate()?;
        *self = next;
        Ok(result)
    }

    fn linear(&mut self, xyze: [Option<f64>; 4], feed: Option<f64>) -> Result<Effect, Error> {
        for (i, value) in xyze.into_iter().enumerate() {
            if let Some(mut value) = value {
                if i == 3 {
                    value *= self.flow;
                }
                let absolute = self.absolute_xyz && (i != 3 || self.absolute_e_allowed);
                self.last[i] = value + if absolute { self.base[i] } else { self.last[i] };
            }
        }
        if let Some(feed) = feed {
            if feed <= 0.0 {
                return Err(Error::InvalidParameter);
            }
            self.speed = feed * self.speed_factor;
        }
        Ok(Effect::Linear {
            position: self.last,
            speed: self.speed,
        })
    }

    fn validate(&self) -> Result<(), Error> {
        if [self.speed, self.speed_factor, self.flow]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
            || self
                .last
                .iter()
                .chain(&self.base)
                .chain(&self.homing)
                .chain(&self.gcode_position())
                .any(|v| !v.is_finite())
        {
            return Err(Error::Nonfinite);
        }
        Ok(())
    }
}

struct Words {
    family: u8,
    code: u16,
    args: [Option<f64>; 26],
}

impl Words {
    fn get(&self, axis: u8) -> Option<f64> {
        self.args[usize::from(axis - b'A')]
    }
    fn allow(&self, axes: &[u8]) -> Result<(), Error> {
        for (i, value) in self.args.iter().enumerate() {
            if value.is_some() && !axes.contains(&(b'A' + i as u8)) {
                return Err(Error::InvalidParameter);
            }
        }
        Ok(())
    }
}

fn parse(line: &str) -> Result<Option<Words>, Error> {
    if line.len() > MAX_LINE {
        return Err(Error::InvalidLine);
    }
    let line = line
        .split_once(';')
        .map_or(line, |(prefix, _)| prefix)
        .trim_matches(|c: char| c.is_ascii_whitespace());
    if line.is_empty() {
        return Ok(None);
    }
    if !line.is_ascii()
        || line
            .bytes()
            .any(|b| b.is_ascii_control() && !b.is_ascii_whitespace())
        || line.contains(['\n', '\r', '*', '(', ')'])
    {
        return Err(Error::InvalidLine);
    }
    let bytes = line.as_bytes();
    let family = bytes[0].to_ascii_uppercase();
    if !matches!(family, b'G' | b'M') {
        return Err(Error::UnsupportedCommand);
    }
    let params = bytes
        .iter()
        .enumerate()
        .skip(1)
        .find(|(_, b)| b.is_ascii_alphabetic())
        .map_or(bytes.len(), |(i, _)| i);
    let code = &line[1..params].trim_end_matches(|c: char| c.is_ascii_whitespace());
    if code.is_empty() || !code.bytes().all(|b| b.is_ascii_digit()) {
        return Err(Error::InvalidLine);
    }
    // The stock dispatcher uses the literal command spelling, so G01 != G1.
    if code.len() > 1 && code.starts_with('0') {
        return Err(Error::UnsupportedCommand);
    }
    let mut words = Words {
        family,
        code: code.parse().map_err(|_| Error::UnsupportedCommand)?,
        args: [None; 26],
    };
    let mut at = params;
    while at < bytes.len() {
        let axis = bytes[at].to_ascii_uppercase();
        let end = bytes[at + 1..]
            .iter()
            .position(u8::is_ascii_alphabetic)
            .map_or(bytes.len(), |i| at + 1 + i);
        let val = &line[at + 1..end].trim_matches(|c: char| c.is_ascii_whitespace());
        let n: f64 = val.parse().map_err(|_| Error::InvalidParameter)?;
        if !n.is_finite() {
            return Err(Error::Nonfinite);
        }
        let slot = &mut words.args[usize::from(axis - b'A')];
        if slot.is_some() {
            return Err(Error::DuplicateParameter);
        }
        *slot = Some(n);
        at = end;
    }
    Ok(Some(words))
}

#[derive(Default)]
pub struct Output {
    pub moves: Vec<Profile>,
    pub dwell_seconds: Option<f64>,
}

/// Offline G-code to look-ahead profiles. Tracks effective toolhead position
/// separately from the requested G-code position. Never opens any device.
pub struct Preview {
    coords: State,
    saved: BTreeMap<String, State>,
    transform: transform::Stack,
    arcs: Option<arcs::Mode>,
    failed: bool,
    planner: LookAhead,
    limits: Limits,
    toolhead: [f64; 4],
}

impl Preview {
    pub fn new(
        limits: Limits,
        capacity: usize,
        extruder_corner: Option<f64>,
    ) -> Result<Self, Error> {
        Self::new_at(limits, capacity, extruder_corner, [0.0; 4])
    }

    pub(crate) fn new_at(
        limits: Limits,
        capacity: usize,
        extruder_corner: Option<f64>,
        position: [f64; 4],
    ) -> Result<Self, Error> {
        let mut planner = LookAhead::new(capacity, extruder_corner)?;
        planner.prime();
        Ok(Self {
            coords: State {
                last: position,
                ..State::default()
            },
            saved: BTreeMap::new(),
            transform: transform::Stack::default(),
            arcs: None,
            failed: false,
            planner,
            limits,
            toolhead: position,
        })
    }

    pub fn coords(&self) -> &State {
        &self.coords
    }

    pub fn physical_position(&self) -> [f64; 4] {
        self.toolhead
    }

    /// Install a top translation, optionally wrapping the existing one.
    /// It does not move the toolhead or reset reported coordinates.
    pub fn wrap_translation(
        &mut self,
        owner: TransformOwner,
        offset: [f64; 3],
        force: bool,
    ) -> Result<(), Error> {
        self.transform.add(owner, offset, force)
    }

    pub fn set_translation(
        &mut self,
        owner: TransformOwner,
        offset: [f64; 3],
    ) -> Result<(), Error> {
        self.transform.set(owner, offset)
    }

    pub fn unwrap_translation(&mut self, owner: TransformOwner) -> Result<(), Error> {
        self.transform.remove(owner)
    }

    /// Rebase the coordinates the parser sees from the commanded physical
    /// position, as on a stock transform change. Leaves offsets and saved
    /// state intact and emits no move.
    pub fn reset_last_position(&mut self) -> Result<(), Error> {
        let mut candidate = self.coords;
        candidate.last = self.transform.reported(self.toolhead)?;
        candidate.validate()?;
        self.coords = candidate;
        Ok(())
    }

    /// Enable the optional stock G2/G3 plane extension for this offline preview.
    pub fn enable_arcs(&mut self, resolution_mm: f64) -> Result<(), Error> {
        self.arcs = Some(arcs::Mode::new(resolution_mm)?);
        Ok(())
    }

    pub fn line(&mut self, line: &str) -> Result<Output, Error> {
        self.line_with(line, |_, _| Ok(()))
    }

    pub(crate) fn line_with(
        &mut self,
        line: &str,
        mut check: impl FnMut([f64; 4], &mut Move) -> Result<(), Error>,
    ) -> Result<Output, Error> {
        if self.failed {
            return Err(Error::InvalidSession);
        }
        if let Some(mode) = &mut self.arcs {
            if let Some(ops) = mode.commands(line, &self.coords)? {
                let mut output = Output::default();
                for (position, feed) in ops {
                    let result = (|| {
                        let mut next = self.coords;
                        let Effect::Linear { position, speed } = next.linear(position, feed)?
                        else {
                            unreachable!()
                        };
                        next.validate()?;
                        self.queue_move(position, speed, &mut check, &mut output)?;
                        self.coords = next;
                        Ok(())
                    })();
                    if let Err(error) = result {
                        // A partially planned arc is never exposed or replayed.
                        self.failed = true;
                        return Err(error);
                    }
                }
                return Ok(output);
            }
        }
        let mut candidate = self.coords;
        let effect = match extended::apply(&mut candidate, &mut self.saved, line)? {
            Some(effect) => effect,
            None => candidate.apply(line)?,
        };
        let mut out = Output::default();
        match effect {
            Effect::Linear { position, speed } => {
                self.queue_move(position, speed, &mut check, &mut out)?
            }
            Effect::Barrier => out.moves = self.finish()?,
            Effect::Dwell(sec) => {
                out.moves = self.finish()?;
                out.dwell_seconds = Some(sec);
            }
            Effect::None => {}
        }
        self.coords = candidate;
        Ok(out)
    }

    fn queue_move(
        &mut self,
        position: [f64; 4],
        speed: f64,
        check: &mut impl FnMut([f64; 4], &mut Move) -> Result<(), Error>,
        out: &mut Output,
    ) -> Result<(), Error> {
        let position = self.transform.physical(position)?;
        if let Some(mut move_) = Move::optional(self.limits, self.toolhead, position, speed)? {
            check(self.toolhead, &mut move_)?;
            let end = move_.end_position();
            let lazy = self.planner.push(move_)?;
            self.toolhead = end;
            if lazy {
                out.moves.extend(self.flush(true)?);
            }
        }
        Ok(())
    }

    pub fn finish(&mut self) -> Result<Vec<Profile>, Error> {
        let out = self.flush(false)?;
        self.planner.prime();
        Ok(out)
    }

    fn flush(&mut self, lazy: bool) -> Result<Vec<Profile>, Error> {
        if self.failed {
            return Err(Error::InvalidSession);
        }
        match self.planner.flush(lazy) {
            Ok(profiles) => Ok(profiles),
            Err(error) => {
                // A queued move may already have changed the modeled toolhead
                // position. Never continue interpreting after a planning failure.
                self.failed = true;
                Err(error.into())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changes_modes_without_moving_and_recovers_after_bad_input() {
        let mut state = State::default();
        assert_eq!(
            state.apply(" G1x2y+3.5e-.5F1500 ; 🚦").unwrap(),
            Effect::Linear {
                position: [2., 3.5, 0., -0.5],
                speed: 25.
            }
        );
        state.apply("G92 X0 E0").unwrap();
        state.apply("M221 S50").unwrap();
        state.apply("G91").unwrap();
        state.apply("M82").unwrap();
        let before = state;
        for line in [
            "G1 X1 F-1",
            "G1 X1 X2",
            "G20",
            "G1 N2",
            "N13 G1 X1*4",
            "G1 XNaN",
        ] {
            assert!(state.apply(line).is_err(), "{line}");
            assert_eq!(state, before);
        }
        assert_eq!(
            state.apply("G0 X1 E2").unwrap(),
            Effect::Linear {
                position: [3., 3.5, 0., 0.5],
                speed: 25.
            }
        );
        assert_eq!(state.gcode_position(), [1., 3.5, 0., 2.]);
    }

    #[test]
    fn saved_coordinate_modes_restore_xyz_but_not_physical_e() {
        let limits = Limits::new(160.0, 2000.0, 5.0, 0.5).unwrap();
        let mut p = Preview::new(limits, 128, None).unwrap();
        for line in [
            "G1 X2 E1",
            "SAVE_GCODE_STATE NAME='A Case'",
            "SET_GCODE_OFFSET X=3 X_ADJUST=9 MOVE=1 MOVE_SPEED=8",
            "G91",
            "M83",
            "G1 X4 E2",
            "RESTORE_GCODE_STATE NAME='A Case' MOVE=1",
        ] {
            p.line(line).unwrap();
        }
        assert_eq!(p.coords().position(), [2.0, 0.0, 0.0, 3.0]);
        assert_eq!(p.coords().gcode_position(), [2.0, 0.0, 0.0, 1.0]);
        assert_eq!(p.coords().homing_origin(), [0.0; 4]);
        let before = *p.coords();
        assert!(matches!(
            p.line("RESTORE_GCODE_STATE NAME='a case'"),
            Err(Error::UnknownState)
        ));
        assert!(matches!(
            p.line("SET_GCODE_OFFSET X=1 X=2"),
            Err(Error::DuplicateParameter)
        ));
        assert!(p.line("SET_GCODE_OFFSET X=8 MOVE=1 MOVE_SPEED=-1").is_err());
        assert_eq!(*p.coords(), before);
        p.line("G1 X1 E2").unwrap();
        assert_eq!(p.coords().position(), [1.0, 0.0, 0.0, 4.0]);
    }

    #[test]
    fn toolchanger_retract_restore_preserves_logical_extrusion() {
        let limits = Limits::new(160.0, 2000.0, 5.0, 0.5).unwrap();
        for move_ in [false, true] {
            let mut p = Preview::new(limits, 128, None).unwrap();
            for cmd in [
                "G1 X10 E10",
                "M220 S80",
                "M221 S125",
                "SAVE_GCODE_STATE NAME=tool_retract",
                "M220 S100",
                "M221 S100",
                "M83",
                "G1 X20 E-2",
            ] {
                p.line(cmd).unwrap();
            }
            let command = format!(
                "RESTORE_GCODE_STATE NAME=tool_retract MOVE={}",
                u8::from(move_)
            );
            p.line(&command).unwrap();
            assert_eq!(p.coords().position()[0], if move_ { 10.0 } else { 20.0 });
            assert_eq!(p.coords().position()[3], 8.0);
            assert_eq!(p.coords().gcode_position()[3], 10.0);
            p.line("G1 E11").unwrap();
            assert_eq!(p.coords().position()[3], 9.25);
        }
    }

    #[test]
    fn translation_wrap_preserves_machine_position_on_resync() {
        let limits = Limits::new(160.0, 2000.0, 5.0, 0.5).unwrap();
        let mut p = Preview::new(limits, 128, None).unwrap();
        p.line("G1 X10 Y12 E3").unwrap();
        p.wrap_translation(TransformOwner(1), [1.0, 2.0, 0.0], false)
            .unwrap();
        assert_eq!(
            p.wrap_translation(TransformOwner(2), [4.0; 3], false),
            Err(Error::TransformOwner)
        );
        p.wrap_translation(TransformOwner(2), [4.0, 0.0, 0.0], true)
            .unwrap();
        assert_eq!(
            p.set_translation(TransformOwner(1), [4.0; 3]),
            Err(Error::TransformOwner)
        );
        p.reset_last_position().unwrap();
        assert_eq!(p.physical_position(), [10.0, 12.0, 0.0, 3.0]);
        assert_eq!(p.coords().position(), [5.0, 10.0, 0.0, 3.0]);
        p.line("G1 X7").unwrap();
        assert_eq!(p.physical_position(), [12.0, 12.0, 0.0, 3.0]);
        p.unwrap_translation(TransformOwner(2)).unwrap();
        p.reset_last_position().unwrap();
        assert_eq!(p.coords().position(), [11.0, 10.0, 0.0, 3.0]);
    }

    #[test]
    fn arcs_require_opt_in_and_reject_unbounded_or_relative_requests() {
        let limits = Limits::new(160.0, 2000.0, 5.0, 0.5).unwrap();
        let mut p = Preview::new(limits, 96, None).unwrap();
        assert!(p.line("G3 X2 Y1 I1 J0").is_err());
        p.enable_arcs(0.2).unwrap();
        p.line("G1 X1 Y0 F900").unwrap();
        p.line("G17").unwrap();
        p.line("G3 X0 Y1 Z0.4 I-1 J0").unwrap();
        assert_eq!(p.coords().gcode_position()[..3], [0.0, 1.0, 0.4]);
        assert!(matches!(p.line("G3 I1000000 J0"), Err(Error::ArcLimit)));
        assert_eq!(p.coords().gcode_position()[..3], [0.0, 1.0, 0.4]);
        p.line("G91").unwrap();
        assert!(matches!(p.line("G2 X1 I1"), Err(Error::InvalidParameter)));
        p.line("G90").unwrap();
        assert!(p.line("G2 X1 R2").is_err());
    }

    #[test]
    fn preview_flushes_at_dwell_and_suppresses_zero_moves() {
        let limits = Limits::new(200.0, 3000.0, 5.0, 0.5).unwrap();
        let mut preview = Preview::new(limits, 48, None).unwrap();
        assert!(preview.line("G1 F1500").unwrap().moves.is_empty());
        preview.line("G1 X10").unwrap();
        preview.line("G1 X10").unwrap();
        let output = preview.line("G4 P250").unwrap();
        assert_eq!(output.moves.len(), 1);
        assert_eq!(output.dwell_seconds, Some(0.25));
        assert!(preview.finish().unwrap().is_empty());
    }
}
