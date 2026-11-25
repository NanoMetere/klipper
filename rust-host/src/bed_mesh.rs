// SPDX-License-Identifier: GPL-3.0-only
//! Offline rectangular mesh interpolation adapted from Klipper extras/bed_mesh.py.
//! Upstream copyright (C) 2018-2019 Eric Callahan <arksine.code@gmail.com>.
//! No probing, config writes, sensor control, or device access.

pub mod profile;

const MAX_VALUES: usize = 65_536;
const MAX_CHECKS: f64 = 16_384.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Grid,
    Input,
    Fade,
    SplitBound,
}

#[derive(Debug, Clone, Copy)]
pub enum Algorithm {
    Direct,
    Lagrange {
        points_per_segment: [usize; 2],
    },
    Bicubic {
        points_per_segment: [usize; 2],
        tension: f64,
    },
}

#[derive(Clone)]
pub struct Mesh {
    min: [f64; 2],
    count: [usize; 2],
    spacing: [f64; 2],
    offset: [f64; 2],
    dense: Vec<f64>,
}

impl Mesh {
    /// Construct from a row-major rectangular matrix of offline measurements.
    pub fn new(
        bounds: [[f64; 2]; 2],
        size: [usize; 2],
        points: &[f64],
        algo: Algorithm,
    ) -> Result<Self, Error> {
        let [min, max] = bounds;
        if size.iter().any(|&n| !(2..=128).contains(&n))
            || points.len() != size[0] * size[1]
            || points.iter().any(|p| !p.is_finite() || p.abs() > 1e6)
            || (0..2).any(|i| {
                !min[i].is_finite()
                    || !max[i].is_finite()
                    || max[i] <= min[i]
                    || !(max[i] - min[i]).is_finite()
            })
        {
            return Err(Error::Grid);
        }
        let per = match algo {
            Algorithm::Direct => [0; 2],
            Algorithm::Lagrange { points_per_segment } => {
                if size[0].max(size[1]) > 6 {
                    return Err(Error::Grid);
                }
                points_per_segment
            }
            Algorithm::Bicubic {
                points_per_segment,
                tension,
            } => {
                if size[0].min(size[1]) < 3 || !(0.0..=2.0).contains(&tension) {
                    return Err(Error::Grid);
                }
                points_per_segment
            }
        };
        if per.iter().any(|&n| n > 64) {
            return Err(Error::Grid);
        }
        let count = std::array::from_fn(|i| (size[i] - 1) * per[i] + size[i]);
        if count[0] * count[1] > MAX_VALUES {
            return Err(Error::Grid);
        }
        let spacing = std::array::from_fn(|i| (max[i] - min[i]) / (count[i] - 1) as f64);
        if spacing.contains(&0.0) {
            return Err(Error::Grid);
        }
        let mult = [1 + per[0], 1 + per[1]];
        let mut dense = vec![0.0; count[0] * count[1]];
        for ix in 0..size[0] {
            for iy in 0..size[1] {
                dense[ix * mult[0] + iy * mult[1] * count[0]] = points[iy * size[0] + ix];
            }
        }
        for axis in 0..2 {
            let stride = if axis == 0 { 1 } else { count[0] };
            let other_stride = if axis == 0 { count[0] } else { 1 };
            for i in 0..count[1 - axis] {
                if axis == 0 && !i.is_multiple_of(mult[1]) {
                    continue;
                }
                let line = i * other_stride;
                for j in 0..count[axis] {
                    if j.is_multiple_of(mult[axis]) {
                        continue;
                    }
                    let control = |j: usize| dense[line + j * mult[axis] * stride];
                    let z = match algo {
                        Algorithm::Direct => unreachable!(),
                        Algorithm::Lagrange { .. } => {
                            let coord = |i: usize| min[axis] + spacing[axis] * i as f64;
                            let target = coord(j);
                            let mut sum = 0.0;
                            for k in 0..size[axis] {
                                let mut n = 1.0;
                                let mut d = 1.0;
                                let center = coord(k * mult[axis]);
                                for a in 0..size[axis] {
                                    if a != k {
                                        let reference = coord(a * mult[axis]);
                                        n *= target - reference;
                                        d *= center - reference;
                                    }
                                }
                                sum += control(k) * n / d;
                            }
                            sum
                        }
                        Algorithm::Bicubic { tension, .. } => {
                            let node = j / mult[axis];
                            let p0 = control(node.saturating_sub(1));
                            let p1 = control(node);
                            let p2 = control(node + 1);
                            let p3 = control((node + 2).min(size[axis] - 1));
                            let t = (j - node * mult[axis]) as f64 / mult[axis] as f64;
                            let t2 = t * t;
                            let t3 = t2 * t;
                            p1 * (2.0 * t3 - 3.0 * t2 + 1.0)
                                + p2 * (-2.0 * t3 + 3.0 * t2)
                                + (tension * (p2 - p0)) * (t3 - 2.0 * t2 + t)
                                + (tension * (p3 - p1)) * (t3 - t2)
                        }
                    };
                    if !z.is_finite() {
                        return Err(Error::Grid);
                    }
                    dense[line + j * stride] = z;
                }
            }
        }
        Ok(Self {
            min,
            count,
            spacing,
            offset: [0.0; 2],
            dense,
        })
    }

    pub fn set_offsets(&mut self, offset: [f64; 2]) -> Result<(), Error> {
        if offset.iter().any(|x| !x.is_finite()) {
            return Err(Error::Input);
        }
        self.offset = offset;
        Ok(())
    }

    pub fn range(&self) -> [f64; 2] {
        self.dense
            .iter()
            .fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], z| {
                [lo.min(*z), hi.max(*z)]
            })
    }

    fn average(&self) -> f64 {
        let total: f64 = self
            .dense
            .chunks(self.count[0])
            .map(|row| row.iter().sum::<f64>())
            .sum();
        let average = total / self.dense.len() as f64;
        // Stock's reported default target is rounded to hundredths.
        format!("{average:.2}")
            .parse()
            .expect("bounded finite mesh average")
    }

    pub fn sample(&self, xy: [f64; 2]) -> Result<f64, Error> {
        let mut blend = [0.0; 2];
        let mut index = [0; 2];
        for i in 0..2 {
            let p = xy[i] + self.offset[i];
            if !p.is_finite() {
                return Err(Error::Input);
            }
            let grid = ((p - self.min[i]) / self.spacing[i]).floor() as usize;
            index[i] = grid.min(self.count[i] - 2);
            blend[i] = ((p - (self.min[i] + self.spacing[i] * index[i] as f64)) / self.spacing[i])
                .clamp(0.0, 1.0);
        }
        let i = index[1] * self.count[0] + index[0];
        let a = lerp(blend[0], self.dense[i], self.dense[i + 1]);
        let b = lerp(
            blend[0],
            self.dense[i + self.count[0]],
            self.dense[i + self.count[0] + 1],
        );
        Ok(lerp(blend[1], a, b))
    }
}

fn lerp(t: f64, a: f64, b: f64) -> f64 {
    (1.0 - t) * a + t * b
}

#[derive(Clone)]
pub struct Transform {
    mesh: Mesh,
    fade: Option<(f64, f64, f64)>, // start, end, target
    tool_z: f64,
    split_z: f64,
    split_distance: f64,
}

impl Transform {
    /// `fade` contains start/end/explicit target, or None for no fade.
    pub fn new(
        mesh: Mesh,
        fade: Option<(f64, f64, f64)>,
        tool_z: f64,
        split_z: f64,
        split_distance: f64,
    ) -> Result<Self, Error> {
        Self::from_parts(mesh, fade, true, tool_z, split_z, split_distance)
    }

    fn from_parts(
        mesh: Mesh,
        fade: Option<(f64, f64, f64)>,
        explicit_target: bool,
        tool_z: f64,
        split_z: f64,
        split_distance: f64,
    ) -> Result<Self, Error> {
        if !tool_z.is_finite()
            || !split_z.is_finite()
            || split_z < 0.01
            || !split_distance.is_finite()
            || split_distance < 3.0
        {
            return Err(Error::Input);
        }
        if let Some((start, end, target)) = fade {
            let [lo, hi] = mesh.range();
            if [start, end, target].iter().any(|x| !x.is_finite())
                || !(end - start).is_finite()
                || end - start <= lo.abs().max(hi.abs())
                || (explicit_target && target != 0.0 && !(lo..=hi).contains(&target))
            {
                return Err(Error::Fade);
            }
        }
        Ok(Self {
            mesh,
            fade,
            tool_z,
            split_z,
            split_distance,
        })
    }

    /// Reinterpret the current physical point with updated lookup offsets.
    /// Do not copy the interpolation grid; roll back the offset if either
    /// transform or coordinate validation fails.
    pub(crate) fn with_lookup_offsets<T, E>(
        &mut self,
        updates: [Option<f64>; 2],
        map_error: impl FnOnce(Error) -> E,
        validate: impl FnOnce(&Self) -> Result<T, E>,
    ) -> Result<T, E> {
        let old = self.mesh.offset;
        self.mesh
            .set_offsets(std::array::from_fn(|i| updates[i].unwrap_or(old[i])))
            .map_err(map_error)?;
        let result = validate(self);
        if result.is_err() {
            self.mesh.offset = old;
        }
        result
    }

    fn z_offset(&self, xy: [f64; 2], factor: f64, target: f64) -> Result<f64, Error> {
        Ok(factor * (self.mesh.sample(xy)? - target) + target)
    }

    pub fn reported(&self, mut physical: [f64; 4]) -> Result<[f64; 4], Error> {
        if physical.iter().any(|x| !x.is_finite()) {
            return Err(Error::Input);
        }
        let adj = self.mesh.sample([physical[0], physical[1]])?;
        let z = physical[2] + self.tool_z;
        let correction = if let Some((start, end, target)) = self.fade {
            let factor = if z.min(z - adj) >= end {
                0.0
            } else if z.max(z - adj) >= start {
                ((end + target - z) / (end - start - (adj - target))).clamp(0.0, 1.0)
            } else {
                1.0
            };
            factor * (adj - target) + target
        } else {
            adj
        };
        physical[2] -= correction;
        Ok(physical)
    }

    /// Compute all physical split targets for one requested offline move,
    /// using the fade factor evaluated at the final requested Z (as stock).
    pub fn split(&self, from: [f64; 4], mut to: [f64; 4]) -> Result<Vec<[f64; 4]>, Error> {
        if from.iter().chain(&to).any(|x| !x.is_finite()) {
            return Err(Error::Input);
        }
        let (factor, target) = self.fade.map_or((1.0, 0.0), |(start, end, target)| {
            let fade_z = to[2] + self.tool_z;
            (((end - fade_z) / (end - start)).clamp(0.0, 1.0), target)
        });
        let mut out = Vec::new();
        if factor != 0.0 {
            let delta = std::array::from_fn::<_, 4, _>(|i| to[i] - from[i]);
            let len = crate::lookahead::squared_xyz(delta).sqrt();
            if !len.is_finite() || len / self.split_distance > MAX_CHECKS {
                return Err(Error::SplitBound);
            }
            let moved = delta.map(|x| x.abs() > 1e-10);
            if moved[0] || moved[1] {
                let mut z = self.z_offset([from[0], from[1]], factor, target)?;
                let mut walked = 0.0;
                while walked + self.split_distance < len {
                    walked += self.split_distance;
                    let t = walked / len;
                    let mut mid = std::array::from_fn(|i| {
                        if moved[i] {
                            lerp(t, from[i], to[i])
                        } else {
                            from[i]
                        }
                    });
                    let next = self.z_offset([mid[0], mid[1]], factor, target)?;
                    if (next - z).abs() >= self.split_z {
                        z = next;
                        mid[2] += z;
                        out.push(mid);
                    }
                }
            }
            to[2] += self.z_offset([to[0], to[1]], factor, target)?;
        } else {
            to[2] += target;
        }
        out.push(to);
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_edges_and_fades_to_target() {
        let mesh = Mesh::new(
            [[0., 0.], [10., 10.]],
            [2, 2],
            &[0., 0.2, 0.1, 0.3],
            Algorithm::Direct,
        )
        .unwrap();
        assert_eq!(mesh.sample([-100., -100.]), Ok(0.));
        assert_eq!(mesh.sample([100., 100.]), Ok(0.3));
        assert!((mesh.sample([5., 5.]).unwrap() - 0.15).abs() < 1e-12);
        let t = Transform::new(mesh, Some((1., 5., 0.05)), 0., 0.01, 3.).unwrap();
        let a = [1., 1., 9., 5.];
        let b = [9., 9., 9., 10.];
        let p = t.split(a, b).unwrap();
        assert_eq!(p, [[9., 9., 9.05, 10.]]);
        assert!((t.reported(p[0]).unwrap()[2] - 9.).abs() < 1e-12);
    }

    #[test]
    fn lookup_offsets_keep_omitted_axes_and_rollback_on_failed_rebase() {
        let mesh = Mesh::new(
            [[0., 0.], [10., 10.]],
            [2, 2],
            &[0., 0.2, 0.1, 0.3],
            Algorithm::Direct,
        )
        .unwrap();
        let mut transform = Transform::new(mesh, None, 0., 0.01, 3.).unwrap();
        let position = [1., 2., 0.4, 0.];
        let original = transform.reported(position).unwrap();
        let updated = transform
            .with_lookup_offsets([Some(3.), Some(4.)], |e| e, |t| t.reported(position))
            .unwrap();
        assert_ne!(updated, original);
        let failed = transform.with_lookup_offsets(
            [Some(9.), None],
            |e| e,
            |candidate| {
                assert_eq!(candidate.mesh.offset, [9., 4.]);
                assert_ne!(candidate.reported(position), Ok(updated));
                Err::<(), _>(Error::Input)
            },
        );
        assert_eq!(failed, Err(Error::Input));
        assert_eq!(transform.mesh.offset, [3., 4.]);
        assert_eq!(transform.reported(position), Ok(updated));
    }

    #[test]
    fn rejects_shape_and_invalid_fade() {
        assert!(Mesh::new([[0., 0.], [1., 1.]], [2, 2], &[0.], Algorithm::Direct).is_err());
        let mesh = Mesh::new(
            [[0., 0.], [1., 1.]],
            [2, 2],
            &[0., 1., 0., 1.],
            Algorithm::Direct,
        )
        .unwrap();
        assert!(Transform::new(mesh, Some((0., 0.5, 0.)), 0., 0.02, 4.).is_err());
    }
}
