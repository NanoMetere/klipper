// SPDX-License-Identifier: GPL-3.0-only
//! Bounded, offline XYZ translations for composing tool offsets.
//! The toolchanger's Python plugin and non-linear transforms are not executed.

use super::Error;

const MAX_LAYERS: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Owner(pub u32);

#[derive(Default)]
pub(super) struct Stack {
    layers: Vec<(Owner, [f64; 3])>,
}

impl Stack {
    pub(super) fn add(&mut self, owner: Owner, offset: [f64; 3], force: bool) -> Result<(), Error> {
        if (!self.layers.is_empty() && !force) || self.layers.iter().any(|l| l.0 == owner) {
            return Err(Error::TransformOwner);
        }
        if self.layers.len() == MAX_LAYERS {
            return Err(Error::TransformLimit);
        }
        if offset.iter().any(|v| !v.is_finite()) {
            return Err(Error::Nonfinite);
        }
        self.layers.push((owner, offset));
        Ok(())
    }

    pub(super) fn set(&mut self, owner: Owner, offset: [f64; 3]) -> Result<(), Error> {
        let Some(last) = self.layers.last_mut().filter(|l| l.0 == owner) else {
            return Err(Error::TransformOwner);
        };
        if offset.iter().any(|v| !v.is_finite()) {
            return Err(Error::Nonfinite);
        }
        last.1 = offset;
        Ok(())
    }

    pub(super) fn remove(&mut self, owner: Owner) -> Result<(), Error> {
        if self.layers.last().is_none_or(|l| l.0 != owner) {
            return Err(Error::TransformOwner);
        }
        self.layers.pop();
        Ok(())
    }

    pub(super) fn physical(&self, mut pos: [f64; 4]) -> Result<[f64; 4], Error> {
        for (_, offset) in self.layers.iter().rev() {
            for (v, d) in pos.iter_mut().zip(offset) {
                *v += d;
            }
        }
        if pos.iter().any(|v| !v.is_finite()) {
            return Err(Error::Nonfinite);
        }
        Ok(pos)
    }

    pub(super) fn reported(&self, mut pos: [f64; 4]) -> Result<[f64; 4], Error> {
        for (_, offset) in &self.layers {
            for (v, d) in pos.iter_mut().zip(offset) {
                *v -= d;
            }
        }
        if pos.iter().any(|v| !v.is_finite()) {
            return Err(Error::Nonfinite);
        }
        Ok(pos)
    }
}
