// SPDX-License-Identifier: GPL-3.0-only
//! Compose an inert scripted tool exchange. No configured plugin macros run.

use super::{Action, Catalog, Changer, Completion, Dock, Error};
use crate::gcode::{self, Output, Preview, TransformOwner};

#[derive(Debug)]
pub enum Failure {
    Change(Error),
    Coordinates(gcode::Error),
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Scripts<'a> {
    pub before: &'a str,
    pub dropoff: &'a str,
    pub pickup: &'a str,
    pub after: &'a str,
}

#[derive(Debug, Clone, Copy)]
pub struct ChangeRequest<'a> {
    pub target: Option<u8>,
    pub homed_axes: u8,
    pub detected_after_pickup: Option<u8>,
    pub scripts: Scripts<'a>,
}

impl Catalog {
    /// Offline, all-or-nothing selection. The returned preview and profiles
    /// are released only after the pickup proof and restoration both finish.
    /// Scripts are explicitly supplied fixtures, not Jinja templates from a
    /// printer config. Outputs are inert and never sent to firmware.
    pub fn preview_change(
        &self,
        changer: &mut Changer,
        preview: Preview,
        owner: TransformOwner,
        change: ChangeRequest<'_>,
    ) -> Result<(Preview, Vec<Output>), Failure> {
        let to = change
            .target
            .map(|target| self.numbered(target).ok_or(Failure::Change(Error::Tool)))
            .transpose()?;
        let from = changer
            .active()
            .map(|active| self.numbered(active).ok_or(Failure::Change(Error::Tool)))
            .transpose()?;
        let Some(request) = changer
            .select(change.target, change.homed_axes)
            .map_err(Failure::Change)?
        else {
            return Ok((preview, Vec::new()));
        };
        let axes = to.or(from).map_or(0, |tool| tool.restore_axes);
        let offset = to.map_or([0.; 3], |tool| tool.nozzle_offset);
        let result = (|| {
            let mut current = Some(request);
            let mut draft = None;
            let mut saved = Some(preview);
            let mut done = None;
            while let Some(req) = current {
                let mut observed = Completion::Completed;
                let coord = |result: Result<Dock, gcode::Error>| {
                    result.map(Some).map_err(Failure::Coordinates)
                };
                let missing = Failure::Change(Error::State);
                match req.action {
                    Action::Save => {
                        draft = coord(Dock::prepare(saved.take().ok_or(missing)?, owner, axes))?;
                    }
                    Action::Before(_) => {
                        draft = coord(
                            draft
                                .take()
                                .ok_or(missing)?
                                .preview_lines(change.scripts.before),
                        )?;
                    }
                    Action::SelectDockTransform => {
                        draft = coord(draft.take().ok_or(missing)?.enter_coordinates())?;
                    }
                    Action::Dropoff(_) => {
                        draft = coord(
                            draft
                                .take()
                                .ok_or(missing)?
                                .preview_lines(change.scripts.dropoff),
                        )?;
                    }
                    Action::Pickup(_) => {
                        draft = coord(
                            draft
                                .take()
                                .ok_or(missing)?
                                .preview_lines(change.scripts.pickup),
                        )?;
                    }
                    Action::Verify(_) => {
                        observed = Completion::Detected(change.detected_after_pickup)
                    }
                    Action::After(_) => {
                        draft = coord(
                            draft
                                .take()
                                .ok_or(missing)?
                                .preview_lines(change.scripts.after),
                        )?;
                    }
                    Action::Restore => {
                        done = Some(
                            draft
                                .take()
                                .ok_or(missing)?
                                .finish(offset)
                                .map_err(Failure::Coordinates)?,
                        );
                    }
                    Action::ApplyToolConfig(_)
                    | Action::SuspendMissingWatch
                    | Action::ResumeMissingWatch(_) => {}
                }
                current = changer
                    .finish_step(req.ticket, observed)
                    .map_err(Failure::Change)?;
            }
            done.ok_or(Failure::Change(Error::State))
        })();
        if result.is_err() {
            if let Some(req) = changer.current() {
                let _ = changer.finish_step(req.ticket, Completion::Failed);
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{config, lookahead::Limits, toolchange::State};
    use std::collections::BTreeMap;

    fn catalog() -> Catalog {
        let cfg = config::parse(&BTreeMap::from([("main.cfg".into(),
            "[toolchanger]\n[tool A]\ntool_number: 0\ndetection_pin: TEST_1\n[tool B]\ntool_number: 3\ndetection_pin: TEST_2\ngcode_x_offset: 1.25\nt_command_restore_axis: XZ\n".into()
        )]), "main.cfg").unwrap();
        Catalog::from_config(&cfg).unwrap()
    }

    fn preview() -> (Preview, TransformOwner) {
        let limits = Limits::new(150.0, 1000.0, 5.0, 0.5).unwrap();
        let mut preview = Preview::new(limits, 256, None).unwrap();
        let owner = TransformOwner(1);
        preview.wrap_translation(owner, [0.; 3], true).unwrap();
        preview.line("G90").unwrap();
        preview.line("G1 X12 Y15 Z1 E2 F900").unwrap();
        (preview, owner)
    }

    #[test]
    fn commits_a_dock_exchange_only_after_observed_pickup() {
        let config = catalog();
        let mut changer = config.build().unwrap();
        changer.initialize_from_detection(Some(0)).unwrap();
        let (p, owner) = preview();
        let mut next = ChangeRequest {
            target: Some(3),
            homed_axes: 7,
            detected_after_pickup: Some(2),
            scripts: Scripts {
                before: "G0 X14",
                dropoff: "G0 X40 Y80 Z5",
                pickup: "G0 X50 Y70 Z4",
                after: "G1 E1.7",
            },
        };
        assert!(matches!(
            config.preview_change(&mut changer, p, owner, next),
            Err(Failure::Change(Error::Detection))
        ));
        assert_eq!(changer.state(), State::Faulted);
        assert_eq!(changer.active(), None);
        changer.initialize_from_detection(Some(0)).unwrap();
        next.detected_after_pickup = Some(3);
        let (p, owner) = preview();
        let (p, out) = config.preview_change(&mut changer, p, owner, next).unwrap();
        assert_eq!(changer.active(), Some(3));
        assert_eq!(p.physical_position(), [13.25, 70., 1., 1.7]);
        assert!(out.iter().any(|o| !o.moves.is_empty()));
    }
}
