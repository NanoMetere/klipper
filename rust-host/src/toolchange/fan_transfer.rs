// SPDX-License-Identifier: GPL-3.0-only
//! Pure tool fan transfer adapted from viesturz/klipper-toolchanger FanSwitcher.
//! Upstream copyright (C) 2023 Viesturs Zarins.
//! Inert intents only; there is no callback to a fan or G-code interface.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FanIntent {
    pub fan: u8,
    pub speed: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FanError {
    MissingObservation,
    Speed,
}

pub struct FanTransfer {
    active: Option<u8>,
    pending: Option<f64>,
    enabled: bool,
}

impl FanTransfer {
    pub const fn new(enabled: bool) -> Self {
        Self {
            active: None,
            pending: None,
            enabled,
        }
    }

    pub fn active(&self) -> Option<u8> {
        self.active
    }

    pub fn pending(&self) -> Option<f64> {
        self.pending
    }

    pub fn set(&mut self, tool_fan: Option<u8>, speed: f64) -> Result<Option<FanIntent>, FanError> {
        valid(speed)?;
        if let Some(fan) = tool_fan {
            Ok(Some(FanIntent { fan, speed }))
        } else {
            self.pending = Some(speed);
            Ok(None)
        }
    }

    /// Transfer already observed effective speed; do not guess an old fan's
    /// speed or issue a real device command if that observation is absent.
    pub fn activate(
        &mut self,
        incoming: Option<u8>,
        old_speed: Option<f64>,
    ) -> Result<Vec<FanIntent>, FanError> {
        if self.active == incoming || !self.enabled {
            return Ok(Vec::new());
        }
        let mut speed = self.pending;
        let mut intents = Vec::new();
        if let Some(old) = self.active {
            let declared = old_speed.ok_or(FanError::MissingObservation)?;
            valid(declared)?;
            speed = Some(declared);
            intents.push(FanIntent {
                fan: old,
                speed: 0.0,
            });
        }
        self.active = incoming;
        if let Some(value) = speed {
            if let Some(fan) = incoming {
                self.pending = None;
                intents.push(FanIntent { fan, speed: value });
            } else {
                self.pending = Some(value);
            }
        }
        Ok(intents)
    }
}

fn valid(speed: f64) -> Result<(), FanError> {
    if !(0.0..=1e6).contains(&speed) {
        return Err(FanError::Speed);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn carries_speed_across_unmount_and_refuses_unknown_old_speed() {
        let mut router = FanTransfer::new(true);
        assert_eq!(router.set(None, 0.78), Ok(None));
        assert_eq!(
            router.activate(Some(2), None).unwrap(),
            [FanIntent {
                fan: 2,
                speed: 0.78
            }]
        );
        assert_eq!(
            router.activate(Some(1), None),
            Err(FanError::MissingObservation)
        );
        assert_eq!(router.active(), Some(2));
        assert_eq!(
            router.activate(None, Some(0.32)).unwrap(),
            [FanIntent { fan: 2, speed: 0.0 }]
        );
        assert_eq!(
            router.activate(Some(3), None).unwrap(),
            [FanIntent {
                fan: 3,
                speed: 0.32
            }]
        );
    }
}
