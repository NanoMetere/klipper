// SPDX-License-Identifier: GPL-3.0-only
//! Keep independent inert extrusion coordinates for configured virtual tools.

use crate::gcode::{self, Output, Preview};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Slots,
    Unknown,
    Gcode(gcode::Error),
}

pub struct ExtruderPositions {
    active: u8,
    parked: BTreeMap<u8, f64>,
}

impl ExtruderPositions {
    pub fn new(slots: &[(u8, f64)], active: u8) -> Result<Self, Error> {
        if !(1..=64).contains(&slots.len()) || slots.iter().any(|&(_, e)| !e.is_finite()) {
            return Err(Error::Slots);
        }
        let parked: BTreeMap<_, _> = slots.iter().copied().collect();
        if parked.len() != slots.len() || !parked.contains_key(&active) {
            return Err(Error::Slots);
        }
        Ok(Self { active, parked })
    }

    pub fn active(&self) -> u8 {
        self.active
    }

    /// Return prior profiles, tagged by the outgoing extruder ID. No move
    /// is generated for the new ID. Caller owns allocation and chronology.
    pub fn switch(
        &mut self,
        next: u8,
        preview: &mut Preview,
    ) -> Result<Option<(u8, Output)>, Error> {
        let &at = self.parked.get(&next).ok_or(Error::Unknown)?;
        if next == self.active {
            return Ok(None);
        }
        let previous = self.active;
        let prev_pos = preview.physical_position()[3];
        let output = preview
            .activate_extruder_position(at)
            .map_err(Error::Gcode)?;
        self.parked.insert(previous, prev_pos);
        self.active = next;
        Ok(Some((previous, output)))
    }

    pub fn observed_position(&self, slot: u8, preview: &Preview) -> Option<f64> {
        if slot == self.active {
            Some(preview.physical_position()[3])
        } else {
            self.parked.get(&slot).copied()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lookahead::Limits;

    #[test]
    fn switching_resets_logical_e_and_does_not_move_parked_lanes() {
        let mut p =
            Preview::new(Limits::new(100., 1000., 5., 0.6).unwrap(), 128, Some(1.)).unwrap();
        let mut tools = ExtruderPositions::new(&[(0, 0.), (1, 1.3)], 0).unwrap();
        p.line("G1 X2 E5 F1200").unwrap();
        p.line("M221 S125").unwrap();
        p.line("SAVE_GCODE_STATE NAME=old").unwrap();
        let (old, out) = tools.switch(1, &mut p).unwrap().unwrap();
        assert_eq!(old, 0);
        assert!(!out.moves.is_empty());
        assert_eq!(p.coords().gcode_position()[3], 0.);
        p.line("G1 E2 F500").unwrap();
        assert_eq!(tools.observed_position(1, &p), Some(3.3));
        assert_eq!(tools.observed_position(0, &p), Some(5.));
        tools.switch(0, &mut p).unwrap();
        assert_eq!(p.coords().gcode_position()[3], 0.);
        p.line("RESTORE_GCODE_STATE NAME=old").unwrap();
        p.line("G1 E6").unwrap();
        assert_eq!(p.physical_position()[3], 6.25);
    }
}
