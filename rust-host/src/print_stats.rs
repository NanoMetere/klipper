// SPDX-License-Identifier: GPL-3.0-only
//! Offline print-state and filament/time accounting.
//! Adapted from Klipper klippy/extras/print_stats.py.
//! Upstream copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>.
//! Clocks and extruder positions are exclusively caller-supplied observations.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum State {
    #[default]
    Standby,
    Printing,
    Paused,
    Complete,
    Error,
    Cancelled,
}

impl State {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Standby => "standby",
            Self::Printing => "printing",
            Self::Paused => "paused",
            Self::Complete => "complete",
            Self::Error => "error",
            Self::Cancelled => "cancelled",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Clock,
    Extrusion,
    Metadata,
}

#[derive(Debug, Clone, Copy)]
pub struct Extrusion {
    pub position: f64,
    pub factor: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Snapshot<'a> {
    pub filename: &'a str,
    pub total_duration: f64,
    pub print_duration: f64,
    pub filament_used: f64,
    pub state: State,
    pub message: &'a str,
    pub layers: [Option<u32>; 2],
}

#[derive(Default)]
pub struct Stats {
    filename: String,
    message: String,
    last_event: Option<f64>,
    state: State,
    filament_used: f64,
    prev_pause_duration: f64,
    last_epos: f64,
    total_duration: f64,
    init_duration: f64,
    print_start_time: Option<f64>,
    last_pause_time: Option<f64>,
    layers: [Option<u32>; 2],
}

impl Stats {
    pub fn select(&mut self, file: &str) -> Result<(), Error> {
        if file.len() > 512 || file.contains('\0') {
            return Err(Error::Metadata);
        }
        *self = Self {
            filename: file.into(),
            ..Self::default()
        };
        Ok(())
    }

    pub fn note_start(&mut self, at: f64, reading: Extrusion) -> Result<(), Error> {
        self.check(at)?;
        validate(reading)?;
        if self.print_start_time.is_none() {
            self.print_start_time = Some(at);
        } else if let Some(pause) = self.last_pause_time.take() {
            self.prev_pause_duration += at - pause;
        }
        self.last_epos = reading.position;
        self.state = State::Printing;
        self.message.clear();
        self.last_event = Some(at);
        Ok(())
    }

    pub fn note_pause(&mut self, at: f64, reading: Extrusion) -> Result<(), Error> {
        self.check(at)?;
        if self.last_pause_time.is_none() {
            self.sample_filament(reading)?;
            self.last_pause_time = Some(at);
        }
        if self.state != State::Error {
            self.state = State::Paused;
        }
        self.last_event = Some(at);
        Ok(())
    }

    pub fn switch_extruder(&mut self, at: f64, reading: Extrusion) -> Result<(), Error> {
        self.check(at)?;
        validate(reading)?;
        self.last_epos = reading.position;
        self.last_event = Some(at);
        Ok(())
    }

    pub fn note_finish(&mut self, at: f64, state: State, message: &str) -> Result<(), Error> {
        self.check(at)?;
        if !matches!(state, State::Complete | State::Cancelled | State::Error)
            || message.len() > 4096
        {
            return Err(Error::Metadata);
        }
        if let Some(start) = self.print_start_time.take() {
            self.total_duration = at - start;
            if self.filament_used < 1e-7 {
                self.init_duration = self.total_duration - self.prev_pause_duration;
            }
            self.state = state;
            self.message = message.into();
        }
        self.last_event = Some(at);
        Ok(())
    }

    pub fn note_layers(&mut self, total: Option<u32>, current: Option<u32>) {
        let next_total = total.or(self.layers[0]);
        let mut next_current = current.or(self.layers[1]);
        if next_total == Some(0) {
            self.layers = [None; 2];
            return;
        }
        if next_total != self.layers[0] {
            self.layers = [next_total, Some(0)];
        }
        if let (Some(max), Some(value)) = (self.layers[0], next_current.take()) {
            self.layers[1] = Some(value.min(max));
        }
    }

    pub fn observe(&mut self, at: f64, reading: Extrusion) -> Result<Snapshot<'_>, Error> {
        self.check(at)?;
        let mut paused = self.prev_pause_duration;
        if let Some(start) = self.print_start_time {
            if let Some(pause) = self.last_pause_time {
                paused += at - pause;
            } else {
                self.sample_filament(reading)?;
            }
            self.total_duration = at - start;
            if self.filament_used < 1e-7 {
                self.init_duration = self.total_duration - paused;
            }
        }
        self.last_event = Some(at);
        Ok(Snapshot {
            filename: &self.filename,
            total_duration: self.total_duration,
            print_duration: self.total_duration - self.init_duration - paused,
            filament_used: self.filament_used,
            state: self.state,
            message: &self.message,
            layers: self.layers,
        })
    }

    fn check(&self, time: f64) -> Result<(), Error> {
        if !(0.0..=1e12).contains(&time) || self.last_event.is_some_and(|last| last > time) {
            return Err(Error::Clock);
        }
        Ok(())
    }

    fn sample_filament(&mut self, reading: Extrusion) -> Result<(), Error> {
        validate(reading)?;
        let next = self.filament_used + (reading.position - self.last_epos) / reading.factor;
        if !next.is_finite() {
            return Err(Error::Extrusion);
        }
        self.filament_used = next;
        self.last_epos = reading.position;
        Ok(())
    }
}

fn validate(r: Extrusion) -> Result<(), Error> {
    if !r.position.is_finite() || !r.factor.is_finite() || r.factor <= 0.0 {
        return Err(Error::Extrusion);
    }
    Ok(())
}

impl Snapshot<'_> {
    #[cfg(feature = "webhooks")]
    pub fn publish(&self, out: &mut crate::webhooks::Snapshot) -> bool {
        let data = serde_json::json!({
            "filename": self.filename, "total_duration": self.total_duration,
            "print_duration": self.print_duration, "filament_used": self.filament_used,
            "state": self.state.as_str(), "message": self.message,
            "info": {"total_layer": self.layers[0], "current_layer": self.layers[1]},
        });
        out.set_object("print_stats".into(), data.as_object().unwrap().clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uses_only_active_motion_time_and_new_extruder_baseline() {
        let mut s = Stats::default();
        let extrude = |position| Extrusion {
            position,
            factor: 0.5,
        };
        s.select("test.gcode").unwrap();
        s.note_start(1., extrude(0.)).unwrap();
        s.observe(2., extrude(0.)).unwrap();
        s.note_pause(4., extrude(4.)).unwrap();
        assert_eq!(s.observe(6., extrude(20.)).unwrap().filament_used, 8.);
        s.note_start(10., extrude(8.)).unwrap();
        s.switch_extruder(11., extrude(100.)).unwrap();
        assert_eq!(s.observe(12., extrude(106.)).unwrap().filament_used, 20.);
        s.note_finish(13., State::Complete, "").unwrap();
        let snap = s.observe(20., extrude(500.)).unwrap();
        assert_eq!(
            (snap.total_duration, snap.print_duration, snap.filament_used),
            (12., 5., 20.)
        );
    }
}
