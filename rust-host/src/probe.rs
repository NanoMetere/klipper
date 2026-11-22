// SPDX-License-Identifier: GPL-3.0-only
//! Offline probe-coordinate arithmetic and bounded sample aggregation.
//! Adapted from Klipper extras/probe.py and manual_probe.py.
//! Upstream copyright (C) 2017-2026 Kevin O'Connor <kevin@koconnor.net>.
//! It never actuates a probe, homes an axis, or reads a pin.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResultPoint {
    pub bed: [f64; 3],
    pub tool: [f64; 3],
}

impl ResultPoint {
    pub fn from_trigger(tool: [f64; 3], offset: [f64; 3]) -> Result<Self, Error> {
        let bed = [
            tool[0] + offset[0],
            tool[1] + offset[1],
            tool[2] - offset[2],
        ];
        if tool.iter().chain(&bed).any(|v| !v.is_finite()) {
            return Err(Error::InvalidSample);
        }
        Ok(Self { bed, tool })
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Aggregate {
    Mean,
    Median,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Config,
    Unhomed,
    InvalidSample,
    Tolerance,
    TriggerTimeout,
    Closed,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Decision {
    Continue,
    Retry { remaining: u8 },
    Done(ResultPoint),
}

pub struct Sampler {
    capacity: usize,
    allowance: u8,
    accepted: Vec<ResultPoint>,
    mode: Aggregate,
    tolerance: f64,
    min_z: f64,
    max_z: f64,
    closed: bool,
}

impl Sampler {
    /// `virtually_homed` is a simulation precondition only, not evidence
    /// about the state of any physical printer.
    pub fn new(
        mode: Aggregate,
        count: usize,
        tolerance: f64,
        retries: u8,
        virtually_homed: bool,
    ) -> Result<Self, Error> {
        if !(1..=64).contains(&count) || retries > 16 || !tolerance.is_finite() || tolerance < 0.0 {
            return Err(Error::Config);
        }
        if !virtually_homed {
            return Err(Error::Unhomed);
        }
        Ok(Self {
            capacity: count,
            allowance: retries,
            accepted: Vec::with_capacity(count),
            mode,
            tolerance,
            min_z: f64::INFINITY,
            max_z: f64::NEG_INFINITY,
            closed: false,
        })
    }

    pub fn timeout(&mut self) -> Error {
        if self.closed {
            Error::Closed
        } else {
            self.closed = true;
            self.accepted.clear();
            Error::TriggerTimeout
        }
    }

    pub fn push(&mut self, p: ResultPoint) -> Result<Decision, Error> {
        if self.closed {
            return Err(Error::Closed);
        }
        if p.bed.iter().chain(&p.tool).any(|v| !v.is_finite()) {
            self.closed = true;
            self.accepted.clear();
            return Err(Error::InvalidSample);
        }
        self.min_z = self.min_z.min(p.bed[2]);
        self.max_z = self.max_z.max(p.bed[2]);
        if self.max_z - self.min_z > self.tolerance {
            self.accepted.clear();
            if self.allowance == 0 {
                self.closed = true;
                return Err(Error::Tolerance);
            }
            self.allowance -= 1;
            self.min_z = f64::INFINITY;
            self.max_z = f64::NEG_INFINITY;
            return Ok(Decision::Retry {
                remaining: self.allowance,
            });
        }
        self.accepted.push(p);
        if self.accepted.len() != self.capacity {
            return Ok(Decision::Continue);
        }
        self.closed = true;
        if matches!(self.mode, Aggregate::Median) {
            // Python uses a stable sort, so tied Z retains acquisition order.
            self.accepted
                .sort_by(|a, b| a.bed[2].partial_cmp(&b.bed[2]).unwrap());
            let mid = self.capacity / 2;
            if !self.capacity.is_multiple_of(2) {
                return Ok(Decision::Done(self.accepted[mid]));
            }
            return Ok(Decision::Done(mean(&self.accepted[mid - 1..=mid])?));
        }
        Ok(Decision::Done(mean(&self.accepted)?))
    }
}

fn mean(positions: &[ResultPoint]) -> Result<ResultPoint, Error> {
    let average_field = |which: fn(&ResultPoint) -> [f64; 3]| {
        let inverse = 1.0 / positions.len() as f64;
        std::array::from_fn(|i| {
            // Match compensated floating summation in the pinned Python runtime.
            let (mut hi, mut rest) = (0.0_f64, 0.0);
            for p in positions {
                let v = which(p)[i];
                let next = hi + v;
                rest += if hi.abs() >= v.abs() {
                    (hi - next) + v
                } else {
                    (v - next) + hi
                };
                hi = next;
            }
            (hi + rest) * inverse
        })
    };
    let (bed, tool) = (average_field(|p| p.bed), average_field(|p| p.tool));
    if bed.iter().chain(&tool).any(|v| !v.is_finite()) {
        return Err(Error::InvalidSample);
    }
    Ok(ResultPoint { bed, tool })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn p(z: f64) -> ResultPoint {
        ResultPoint::from_trigger([3., 4., z], [1., -2., 0.25]).unwrap()
    }
    #[test]
    fn retries_a_whole_set_instead_of_keeping_the_outlier() {
        let mut sampler = Sampler::new(Aggregate::Median, 2, 0.01, 1, true).unwrap();
        assert_eq!(sampler.push(p(1.0)), Ok(Decision::Continue));
        assert_eq!(sampler.push(p(1.1)), Ok(Decision::Retry { remaining: 0 }));
        assert_eq!(sampler.push(p(1.5)), Ok(Decision::Continue));
        assert_eq!(sampler.push(p(1.5)), Ok(Decision::Done(p(1.5))));
        assert_eq!(sampler.push(p(1.5)), Err(Error::Closed));
    }
    #[test]
    fn refuses_unhomed_and_latches_a_timeout() {
        assert!(matches!(
            Sampler::new(Aggregate::Mean, 1, 0., 0, false),
            Err(Error::Unhomed)
        ));
        let mut sampler = Sampler::new(Aggregate::Mean, 1, 0., 0, true).unwrap();
        assert_eq!(sampler.timeout(), Error::TriggerTimeout);
        assert_eq!(sampler.push(p(1.0)), Err(Error::Closed));
    }
}
