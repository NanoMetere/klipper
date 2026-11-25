// SPDX-License-Identifier: GPL-3.0-only
//! Offline binding/model state for one extrusion stepper and two logical queues.
//! The actual planner and physical step generation are not controlled here.
//!
//! Position rebasing follows Klipper's kinematics/extruder.py and stepper.py.
//! Upstream copyright (C) 2016-2025 Kevin O'Connor.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Queue {
    Primary,
    Secondary,
}

impl Queue {
    fn index(self) -> usize {
        match self {
            Self::Primary => 0,
            Self::Secondary => 1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Distance,
    Position,
    Pressure,
    InactiveQueue,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Snapshot {
    pub active: Queue,
    pub bound: Option<Queue>,
    pub logical_end: [f64; 2],
    pub commanded: f64,
    pub mcu_steps: i64,
    pub advance: f64,
}

/// Caller provides the committed logical moves and the *flushed* commanded
/// stepper positions from its own planner. No buffering, I/O, or hardware.
pub struct SharedStepper {
    active: Queue,
    bound: Option<Queue>,
    logical_end: [f64; 2],
    commanded: f64,
    mcu_steps: i64,
    offset: f64,
    distance: f64,
    advance: f64,
}

impl SharedStepper {
    pub fn new(distance: f64, positive_advance: f64) -> Result<Self, Error> {
        if !distance.is_finite() || distance <= 0.0 {
            return Err(Error::Distance);
        }
        let mut model = Self {
            active: Queue::Primary,
            bound: None,
            logical_end: [0.0; 2],
            commanded: 0.0,
            mcu_steps: 0,
            offset: 0.0,
            distance,
            advance: 0.0,
        };
        model.set_advance(positive_advance)?;
        Ok(model)
    }

    fn mcu_steps_at(&self, commanded: f64, offset: f64) -> Result<i64, Error> {
        let relative = (commanded + offset) / self.distance;
        let biased = if relative >= 0.0 {
            relative + 0.5
        } else {
            relative - 0.5
        };
        // Keep consecutive integer step values distinguishable in f64.
        if biased.abs() < (1_u64 << 52) as f64 {
            Ok(biased as i64)
        } else {
            Err(Error::Position)
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            active: self.active,
            bound: self.bound,
            logical_end: self.logical_end,
            commanded: self.commanded,
            mcu_steps: self.mcu_steps,
            advance: self.advance,
        }
    }

    /// Commit a logical move after planning, not a G-code request. An
    /// unbound queue may advance independently of the physical stepper.
    pub fn commit_move(&mut self, queue: Queue, end: f64) -> Result<(), Error> {
        if queue != self.active {
            return Err(Error::InactiveQueue);
        }
        if !end.is_finite() {
            return Err(Error::Position);
        }
        self.logical_end[queue.index()] = end;
        Ok(())
    }

    pub fn activate(&mut self, queue: Queue) {
        self.active = queue;
    }

    /// Store requested positive gain on the one physical stepper. This
    /// does not model when a pending lookahead callback makes it effective.
    pub fn set_advance(&mut self, advance: f64) -> Result<(), Error> {
        if !advance.is_finite() || advance <= 0.0 {
            return Err(Error::Pressure);
        }
        self.advance = advance;
        Ok(())
    }

    /// Bind or detach at a boundary already flushed by the caller. Binding
    /// does not replay another queue's history: it rebases the commanded
    /// position but preserves the signed physical integer position.
    pub fn sync(&mut self, queue: Option<Queue>, flushed: f64) -> Result<(), Error> {
        let mcu_steps = self.mcu_steps_at(flushed, self.offset)?;
        let (commanded, offset) = if let Some(q) = queue {
            let anchor = self.logical_end[q.index()];
            let offset = mcu_steps as f64 * self.distance - anchor;
            if self.mcu_steps_at(anchor, offset)? != mcu_steps {
                return Err(Error::Position);
            }
            (anchor, offset)
        } else {
            (flushed, self.offset)
        };
        self.commanded = commanded;
        self.mcu_steps = mcu_steps;
        self.offset = offset;
        self.bound = queue;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detach_and_rebase_preserve_signed_steps_and_ignore_invalid_updates() {
        use Queue::{Primary, Secondary};
        let mut model = SharedStepper::new(0.0625, 0.04).unwrap();
        model.sync(Some(Primary), 0.0).unwrap();
        model.commit_move(Primary, -0.15).unwrap();
        let before = model.snapshot();
        assert_eq!(model.commit_move(Secondary, 3.0), Err(Error::InactiveQueue));
        assert_eq!(model.sync(Some(Secondary), f64::NAN), Err(Error::Position));
        assert_eq!(model.set_advance(0.0), Err(Error::Pressure));
        assert_eq!(model.snapshot(), before);
        model.sync(Some(Secondary), -0.15625).unwrap();
        assert_eq!(model.snapshot().mcu_steps, -3);
        assert_eq!(model.snapshot().commanded, 0.0);
        assert_eq!(model.snapshot().active, Primary);
        model.commit_move(Primary, -0.2).unwrap();
        model.activate(Secondary);
        model.commit_move(Secondary, 0.28).unwrap();
        model.sync(None, 0.28125).unwrap();
        assert_eq!(model.snapshot().mcu_steps, 2);
        assert_eq!(model.snapshot().commanded, 0.28125);
        model.sync(Some(Primary), 0.28125).unwrap();
        let end = model.snapshot();
        assert_eq!(end.mcu_steps, 2);
        assert_eq!(end.commanded, -0.2);
        assert_eq!(end.logical_end, [-0.2, 0.28]);
    }
}
