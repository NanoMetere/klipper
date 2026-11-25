// SPDX-License-Identifier: GPL-3.0-only
//! Bounded iterator for the pinned file-debug reactor's virtual cadence.

const MAX_PRINT_TIME: f64 = 120.25;

pub(crate) struct Cadence {
    need: f64,
    generation: f64,
    wire: f64,
    count: u16,
    done: bool,
}

impl Cadence {
    pub(crate) fn new(need: f64, generation: f64, wire: f64) -> Option<Self> {
        if ![need, generation, wire]
            .iter()
            .all(|v| (0.0..=MAX_PRINT_TIME).contains(v))
            || wire > generation
        {
            return None;
        }
        Some(Self {
            need,
            generation,
            wire,
            count: 0,
            done: false,
        })
    }

    pub(crate) fn more_armed(&self) -> bool {
        self.count != 0
    }
}

impl Iterator for Cadence {
    type Item = (f64, f64);

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        let faux = self.need - 1.5;
        let quantum = 0.700 - 0.450;
        if self.generation < faux {
            let mut target = self.generation + quantum;
            if self.count > 100 && faux > target {
                target += ((faux - target) / quantum).trunc() * quantum;
            }
            self.wire = self.wire.max(target - 0.050);
            self.generation = self.generation.max(target).max(self.wire);
            self.count += 1;
        } else {
            self.done = true;
            if self.count != 0 {
                return None;
            }
            self.wire = self.wire.max(self.need + 0.250);
            self.generation = self.generation.max(self.wire);
        }
        Some((self.wire, self.generation))
    }
}
