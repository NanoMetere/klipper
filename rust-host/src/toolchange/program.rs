// SPDX-License-Identifier: GPL-3.0-only
//! Compose an inert scripted tool exchange. No configured plugin macros run.

use super::{
    Action, Catalog, Changer, Completion, Dock, Error, ExtruderPositionError, ExtruderPositions,
};
use crate::gcode::{self, Output, Preview, TransformOwner};

#[derive(Debug)]
pub enum Failure {
    Change(Error),
    Coordinates(gcode::Error),
    Extruder(ExtruderPositionError),
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

/// Each held output retains the modeled extruder active when it was queued.
pub struct LaneOutput {
    pub tool: u8,
    pub phases: Vec<Output>,
}

pub struct LanePreview {
    pub preview: Preview,
    pub positions: ExtruderPositions,
    pub groups: Vec<LaneOutput>,
}

struct Handoff {
    positions: ExtruderPositions,
    corner_velocity: Option<f64>,
    outgoing: Option<LaneOutput>,
}

struct Exchange {
    preview: Preview,
    phases: Vec<Output>,
    handoff: Option<Handoff>,
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
        let result = self.exchange(changer, preview, owner, change, None)?;
        Ok((result.preview, result.phases))
    }

    /// Tag synthetic before/dropoff E moves with the old lane and pickup/
    /// after moves with the rebased incoming lane, in plugin action order.
    /// A failed pickup yields no drafts, state or partial motor profiles.
    /// This does not validate physical extrusion safety or run product macros.
    pub fn preview_change_lanes(
        &self,
        changer: &mut Changer,
        preview: Preview,
        owner: TransformOwner,
        change: ChangeRequest<'_>,
        positions: ExtruderPositions,
        incoming_corner_velocity: Option<f64>,
    ) -> Result<LanePreview, Failure> {
        let target = change.target.ok_or(Failure::Change(Error::Tool))?;
        if changer.active() != Some(positions.active())
            || positions.observed_position(target, &preview).is_none()
        {
            return Err(Failure::Extruder(ExtruderPositionError::Unknown));
        }
        let result = self.exchange(
            changer,
            preview,
            owner,
            change,
            Some(Handoff {
                positions,
                corner_velocity: incoming_corner_velocity,
                outgoing: None,
            }),
        )?;
        let Handoff {
            positions,
            outgoing,
            ..
        } = result.handoff.expect("lane-aware exchange");
        let mut groups = outgoing.into_iter().collect::<Vec<_>>();
        if !result.phases.is_empty() {
            groups.push(LaneOutput {
                tool: positions.active(),
                phases: result.phases,
            });
        }
        Ok(LanePreview {
            preview: result.preview,
            positions,
            groups,
        })
    }

    fn exchange(
        &self,
        changer: &mut Changer,
        preview: Preview,
        owner: TransformOwner,
        change: ChangeRequest<'_>,
        mut handoff: Option<Handoff>,
    ) -> Result<Exchange, Failure> {
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
            return Ok(Exchange {
                preview,
                phases: Vec::new(),
                handoff,
            });
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
                    Action::ApplyToolConfig(Some(incoming)) => {
                        if let Some(state) = &mut handoff {
                            let tool = state.positions.active();
                            let phases = draft.as_mut().ok_or(missing)?.handoff_extrusion(
                                &mut state.positions,
                                incoming,
                                state.corner_velocity,
                            )?;
                            state.outgoing = Some(LaneOutput { tool, phases });
                        }
                    }
                    Action::ApplyToolConfig(None)
                    | Action::SuspendMissingWatch
                    | Action::ResumeMissingWatch(_) => {}
                }
                current = changer
                    .finish_step(req.ticket, observed)
                    .map_err(Failure::Change)?;
            }
            let (preview, phases) = done.ok_or(Failure::Change(Error::State))?;
            Ok(Exchange {
                preview,
                phases,
                handoff,
            })
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

    #[test]
    fn lane_change_never_yields_a_draft_after_bad_detection_or_bad_pickup_script() {
        let cfg = catalog();
        let mut changer = cfg.build().unwrap();
        let slots = || ExtruderPositions::new(&[(0, 0.), (3, 1.3)], 0).unwrap();
        let run = |changer: &mut Changer, observed, pickup| {
            changer.initialize_from_detection(Some(0)).unwrap();
            let (p, owner) = preview();
            cfg.preview_change_lanes(
                changer,
                p,
                owner,
                ChangeRequest {
                    target: Some(3),
                    homed_axes: 7,
                    detected_after_pickup: observed,
                    scripts: Scripts {
                        before: "M83\nG1 E-.1 F600",
                        dropoff: "G0 X40 Y80 Z5\nG1 E-.3 F600",
                        pickup,
                        after: "G1 E.2 F600",
                    },
                },
                slots(),
                Some(0.75),
            )
        };
        assert!(matches!(
            run(&mut changer, Some(2), "G1 E.1 F600"),
            Err(Failure::Change(Error::Detection))
        ));
        assert_eq!(changer.state(), State::Faulted);
        assert_eq!(changer.active(), None);
        assert!(matches!(
            run(&mut changer, Some(3), "UNKNOWN_COMMAND"),
            Err(Failure::Coordinates(_))
        ));
        assert_eq!(changer.state(), State::Faulted);
        let ready = run(&mut changer, Some(3), "G1 E.1 F600").unwrap();
        let measured = ready
            .groups
            .iter()
            .map(|g| {
                (
                    g.tool,
                    g.phases
                        .iter()
                        .flat_map(|p| &p.moves)
                        .map(|m| m.end[3] - m.start[3])
                        .sum::<f64>(),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(measured.len(), 2);
        assert_eq!(measured[0].0, 0);
        assert_eq!(measured[1].0, 3);
        assert!((measured[0].1 - 1.6).abs() < 1e-12);
        assert!((measured[1].1 - 0.3).abs() < 1e-12);
        assert_eq!(changer.active(), Some(3));
    }
}
