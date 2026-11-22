// SPDX-License-Identifier: GPL-3.0-only
//! Offline extrusion geometry and bounds, adapted from Klipper
//! klippy/kinematics/extruder.py. Copyright (C) 2016-2025 Kevin O'Connor.
//! No heater, pressure-advance, sensor or hardware access.

use crate::{
    lookahead::{self, Move, Profile},
    trajectory::{self, Trajectory, Trapezoid},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Limits,
    Cold,
    ExtrudeOnlyDistance,
    CrossSection,
    Move(lookahead::Error),
}

#[derive(Debug, Clone, Copy)]
pub struct Settings {
    pub nozzle: f64,
    pub filament: f64,
    pub max_area: f64,
    pub retraction_distance: f64,
    pub retraction_velocity: f64,
    pub retraction_accel: f64,
    pub corner_velocity: f64,
    pub min_simulated_temperature: f64,
}

pub struct Extruder {
    settings: Settings,
    max_ratio: f64,
}

impl Extruder {
    pub fn new(settings: Settings) -> Result<Self, Error> {
        if [
            settings.nozzle,
            settings.filament,
            settings.max_area,
            settings.retraction_velocity,
            settings.retraction_accel,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
            || settings.filament < settings.nozzle
            || [
                settings.retraction_distance,
                settings.corner_velocity,
                settings.min_simulated_temperature,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err(Error::Limits);
        }
        let max_ratio =
            settings.max_area / (std::f64::consts::PI * (settings.filament * 0.5).powi(2));
        if !max_ratio.is_finite() || max_ratio <= 0.0 {
            return Err(Error::Limits);
        }
        Ok(Self {
            settings,
            max_ratio,
        })
    }

    pub fn corner_velocity(&self) -> f64 {
        self.settings.corner_velocity
    }

    /// Apply the stock geometric rules with a caller-supplied simulated
    /// temperature. The supplied value is not accepted as physical evidence.
    pub fn check_move(&self, m: &mut Move, start: [f64; 4], temperature: f64) -> Result<(), Error> {
        let end = m.end_position();
        let displacement = end[3] - start[3];
        if displacement == 0.0 {
            return Ok(());
        }
        if !temperature.is_finite() || temperature < self.settings.min_simulated_temperature {
            return Err(Error::Cold);
        }
        let r = m.axes_ratio()[3];
        if (end[0] == start[0] && end[1] == start[1]) || r < 0.0 {
            if displacement.abs() > self.settings.retraction_distance {
                return Err(Error::ExtrudeOnlyDistance);
            }
            let scale = 1.0 / r.abs();
            m.limit(
                self.settings.retraction_velocity * scale,
                self.settings.retraction_accel * scale,
            )
            .map_err(Error::Move)?;
        } else if r > self.max_ratio && displacement > self.settings.nozzle * self.max_ratio {
            return Err(Error::CrossSection);
        }
        Ok(())
    }
}

/// Add signed extrusion phases to a dedicated timeline (X is the E axis).
/// Caller supplies the shared scheduled time and tracks dwells/XYZ-only
/// blocks. This intentionally models no pressure advance.
pub fn append_nominal(
    queue: &mut Trajectory,
    at: f64,
    p: &Profile,
) -> Result<(), trajectory::Error> {
    let r = p.direction[3];
    if r == 0.0 {
        return Ok(());
    }
    let cruise = if (-1e-12..0.0).contains(&p.cruise_t) {
        0.0
    } else {
        p.cruise_t
    };
    queue.append_signed(Trapezoid {
        at,
        phase_sec: [p.accel_t, cruise, p.decel_t],
        start: [p.start[3], 0.0, 0.0],
        direction: [
            1.0,
            f64::from(u8::from(
                r > 0.0 && (p.direction[0] != 0.0 || p.direction[1] != 0.0),
            )),
            0.0,
        ],
        start_velocity: p.start_v * r,
        cruise_velocity: p.cruise_v * r,
        accel: p.accel * r,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lookahead::{Limits, LookAhead};

    fn model() -> Extruder {
        Extruder::new(Settings {
            nozzle: 0.4,
            filament: 1.75,
            max_area: 0.64,
            retraction_distance: 15.,
            retraction_velocity: 20.,
            retraction_accel: 300.,
            corner_velocity: 1.,
            min_simulated_temperature: 170.,
        })
        .unwrap()
    }

    #[test]
    fn checks_virtual_temperature_and_geometry_before_producing_profiles() {
        let ex = model();
        let limits = Limits::new(160., 2000., 5., 0.5).unwrap();
        let pos = [0.; 4];
        let motion = [10., 0., 0., 1.];
        let mut m = Move::new(limits, pos, motion, 30.).unwrap();
        assert_eq!(ex.check_move(&mut m, pos, 169.), Err(Error::Cold));
        assert!(ex.check_move(&mut m, pos, 190.).is_ok());
        let mut m = Move::new(limits, pos, [1., 0., 0., 2.], 30.).unwrap();
        assert_eq!(ex.check_move(&mut m, pos, 190.), Err(Error::CrossSection));
        let mut m = Move::new(limits, pos, [0., 0., 0., -16.], 30.).unwrap();
        assert_eq!(
            ex.check_move(&mut m, pos, 190.),
            Err(Error::ExtrudeOnlyDistance)
        );
        let mut m = Move::new(limits, pos, [0., 0., 0., -2.], 40.).unwrap();
        ex.check_move(&mut m, pos, 190.).unwrap();
        let mut q = LookAhead::new(4, Some(ex.corner_velocity())).unwrap();
        q.push(m).unwrap();
        let p = q.flush(false).unwrap().remove(0);
        assert!(p.cruise_v <= 20.);
        let mut trajectory = Trajectory::new(8).unwrap();
        append_nominal(&mut trajectory, 2., &p).unwrap();
        assert!((trajectory.position(5.).unwrap().unwrap()[0] + 2.).abs() < 1e-12);
    }
}
