// SPDX-License-Identifier: GPL-3.0-only
//! Stock toolchanger coordinate handoff, without docking motion or plugin I/O.
//! Adapted from viesturz/klipper-toolchanger extras/toolchanger.py.
//! Upstream copyright (C) 2023 Viesturs Zarins.

use crate::gcode::{self, Output, Preview, TransformOwner};
use std::fmt::Write;

const SAVED: &str = "__rust_virtual_dock";

/// Owns the preview while it is in dock coordinates. A failed step drops the
/// unfinished preview rather than yielding it as an ordinary motion session.
pub struct Dock {
    preview: Preview,
    owner: TransformOwner,
    nozzle: [f64; 3],
    restore: u8,
    outputs: Vec<Output>,
    lines_left: usize,
    in_dock: bool,
}

impl Dock {
    /// Enter docking coordinates without moving the nominal toolhead.
    /// The caller must have installed the top tool translation already.
    pub fn enter(
        preview: Preview,
        owner: TransformOwner,
        restore_axes: u8,
    ) -> Result<Self, gcode::Error> {
        Self::prepare(preview, owner, restore_axes)?.enter_coordinates()
    }

    pub(super) fn prepare(
        mut preview: Preview,
        owner: TransformOwner,
        restore_axes: u8,
    ) -> Result<Self, gcode::Error> {
        if restore_axes > 7 {
            return Err(gcode::Error::InvalidParameter);
        }
        let nozzle: [f64; 3] = preview.coords().gcode_position()[..3].try_into().unwrap();
        let barrier = preview.line("M400")?;
        preview.line(&format!("SAVE_GCODE_STATE NAME={SAVED}"))?;
        Ok(Self {
            preview,
            owner,
            nozzle,
            restore: restore_axes,
            outputs: vec![barrier],
            lines_left: 4096,
            in_dock: false,
        })
    }

    pub(super) fn enter_coordinates(mut self) -> Result<Self, gcode::Error> {
        if self.in_dock {
            return Err(gcode::Error::InvalidSession);
        }
        self.outputs.push(self.preview.line("M400")?);
        self.preview.set_translation(self.owner, [0.0; 3])?;
        self.preview.reset_last_position()?;
        self.preview.line("SET_GCODE_OFFSET X=0.0 Y=0.0 Z=0.0")?;
        self.in_dock = true;
        Ok(self)
    }

    /// Preview caller-supplied synthetic docking lines. This consumes the
    /// dock object so a failed path cannot be re-armed with partial geometry.
    pub fn preview_lines(mut self, instructions: &str) -> Result<Self, gcode::Error> {
        if instructions.len() > 128 * 1024 {
            return Err(gcode::Error::InvalidLine);
        }
        for line in instructions.lines() {
            self.lines_left = self
                .lines_left
                .checked_sub(1)
                .ok_or(gcode::Error::InvalidLine)?;
            let out = self.preview.line(line)?;
            if !out.moves.is_empty() || out.dwell_seconds.is_some() {
                self.outputs.push(out);
            }
        }
        Ok(self)
    }

    /// Restore the paused nozzle coordinates only on explicitly named axes.
    /// Physical E is preserved, and its logical extrusion base is restored
    /// by the stock-compatible SAVE/RESTORE state engine.
    pub fn finish(mut self, new_offset: [f64; 3]) -> Result<(Preview, Vec<Output>), gcode::Error> {
        if !self.in_dock {
            return Err(gcode::Error::InvalidSession);
        }
        self.outputs.push(self.preview.line("M400")?);
        self.preview.set_translation(self.owner, new_offset)?;
        self.preview.reset_last_position()?;
        let restore = format!("RESTORE_GCODE_STATE NAME={SAVED} MOVE=0");
        self.preview.line(&restore)?;
        if self.restore != 0 {
            self.preview.line("G90")?;
            let mut movement = String::from("G0");
            for (i, axis) in ['X', 'Y', 'Z'].into_iter().enumerate() {
                if self.restore & (1 << i) != 0 {
                    write!(&mut movement, " {axis}{:.17}", self.nozzle[i]).unwrap();
                }
            }
            self.outputs.push(self.preview.line(&movement)?);
            self.preview.line(&restore)?;
            self.outputs.push(self.preview.line("M400")?);
        }
        Ok((self.preview, self.outputs))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lookahead::Limits;

    #[test]
    fn dock_coordinates_restore_selected_axes_and_do_not_reextrude() {
        let limits = Limits::new(150.0, 1000.0, 5.0, 0.5).unwrap();
        let mut state = Preview::new(limits, 60, None).unwrap();
        let owner = TransformOwner(21);
        state
            .wrap_translation(owner, [1.0, 0.5, 0.0], true)
            .unwrap();
        state.reset_last_position().unwrap();
        state.line("G90").unwrap();
        state.line("G1 X10 Y12 Z.4 E3 F1200").unwrap();
        state.line("SET_GCODE_OFFSET X=.3").unwrap();
        let saved = state.coords().gcode_position();
        let mut dock = Dock::enter(state, owner, 5).unwrap();
        let p = dock.preview.physical_position();
        assert_eq!(&p[..3], &[11., 12.5, 0.4]);
        dock = dock
            .preview_lines("G90\nG1 X30 Y31 Z5\nG91\nM83\nG1 E-.3 F600")
            .unwrap();
        assert_eq!(dock.preview.physical_position(), [30., 31., 5., 2.7]);
        let (mut ready, _) = dock.finish([-0.6, 0.5, 0.25]).unwrap();
        let final_pos = ready.physical_position();
        for (got, want) in final_pos.into_iter().zip([9.4, 31., 0.65, 2.7]) {
            assert!((got - want).abs() < 1e-12);
        }
        let logical = ready.coords().gcode_position();
        assert!((logical[0] - saved[0]).abs() < 1e-12);
        assert_eq!(logical[3], saved[3]);
        ready.line("G1 E3.1").unwrap();
        assert!((ready.physical_position()[3] - 2.8).abs() < 1e-12);
    }
}
