// SPDX-License-Identifier: GPL-3.0-only
//! Arc decomposition adapted from Klipper klippy/extras/gcode_arcs.py.
//! Copyright (C) 2019 Aleksej Vasiljkovic, with original Marlin work
//! copyright (C) 2011 Camiel Gubbels / Erik van der Zalm.

use super::{parse, Error, State};
use std::f64::consts::TAU;

const MAX_SEGMENTS: usize = 4096;
type Linear = ([Option<f64>; 4], Option<f64>);

pub(super) struct Mode {
    xyz: [usize; 3],
    resolution: f64,
}

impl Mode {
    pub(super) fn new(resolution: f64) -> Result<Self, Error> {
        if !resolution.is_finite() || resolution <= 0.0 {
            return Err(Error::InvalidParameter);
        }
        Ok(Self {
            xyz: [0, 1, 2],
            resolution,
        })
    }

    pub(super) fn commands(
        &mut self,
        line: &str,
        state: &State,
    ) -> Result<Option<Vec<Linear>>, Error> {
        // Avoid a second numeric parse for the common G0/G1 path.
        let trimmed = line.trim_start();
        let text = trimmed.as_bytes();
        let candidate = match text.get(1..3) {
            Some([b'1', b'7'..=b'9']) => true,
            Some([b'2' | b'3', next]) => !next.is_ascii_digit(),
            _ => matches!(text.get(1..), Some([b'2' | b'3'])),
        };
        if !matches!(text.first(), Some(b'G' | b'g')) || !candidate {
            return Ok(None);
        }
        let Some(words) = parse(line)? else {
            return Ok(None);
        };
        match words.code {
            17..=19 => {
                words.allow(b"")?;
                self.xyz = match words.code {
                    17 => [0, 1, 2],
                    18 => [0, 2, 1],
                    _ => [1, 2, 0],
                };
                return Ok(Some(Vec::new()));
            }
            2 | 3 => {}
            _ => return Ok(None),
        }
        words.allow(b"XYZEFIJK")?;
        if !state.absolute_xyz {
            return Err(Error::InvalidParameter);
        }
        let [alpha, beta, helix] = self.xyz;
        let offsets = *b"IJK";
        let ip = words.get(offsets[alpha]).unwrap_or(0.0);
        let iq = words.get(offsets[beta]).unwrap_or(0.0);
        if ip == 0.0 && iq == 0.0 {
            return Err(Error::InvalidParameter);
        }
        let here = state.gcode_position();
        let target: [f64; 3] = std::array::from_fn(|i| words.get(b"XYZ"[i]).unwrap_or(here[i]));
        let center_p = here[alpha] + ip;
        let center_q = here[beta] + iq;
        let dp = target[alpha] - center_p;
        let dq = target[beta] - center_q;
        let mut angle = (-ip * dq + iq * dp).atan2(-ip * dp - iq * dq);
        if angle < 0.0 {
            angle += TAU;
        }
        if words.code == 2 {
            angle -= TAU;
        }
        if angle == 0.0 && here[alpha] == target[alpha] && here[beta] == target[beta] {
            angle = TAU;
        }
        let linear = target[helix] - here[helix];
        let radial = ip.hypot(iq) * angle;
        let travel = if linear != 0.0 {
            radial.hypot(linear)
        } else {
            radial.abs()
        };
        let raw = (travel / self.resolution).floor();
        if !raw.is_finite() || raw > MAX_SEGMENTS as f64 {
            return Err(Error::ArcLimit);
        }
        let segments = raw.max(1.0) as usize;
        let step_angle = angle / segments as f64;
        let step_helix = linear / segments as f64;
        let feed = words.get(b'F');
        if feed.is_some_and(|f| f <= 0.0) {
            return Err(Error::InvalidParameter);
        }
        let mut e = if state.absolute_e_allowed {
            here[3]
        } else {
            0.0
        };
        let de = words
            .get(b'E')
            .map_or(0.0, |end| (end - e) / segments as f64);
        let mut ops = Vec::with_capacity(segments);
        for i in 1..=segments {
            let mut xyz = target;
            if i != segments {
                let theta = i as f64 * step_angle;
                let sin = theta.sin();
                let cos = theta.cos();
                xyz[alpha] = center_p + (-ip * cos + iq * sin);
                xyz[beta] = center_q + (-ip * sin - iq * cos);
                xyz[helix] = here[helix] + i as f64 * step_helix;
            }
            let e_pos = (de != 0.0).then_some(e + de);
            if state.absolute_e_allowed {
                e += de;
            }
            if xyz.iter().any(|v| !v.is_finite()) || e_pos.is_some_and(|v| !v.is_finite()) {
                return Err(Error::Nonfinite);
            }
            ops.push(([Some(xyz[0]), Some(xyz[1]), Some(xyz[2]), e_pos], feed));
        }
        Ok(Some(ops))
    }
}
