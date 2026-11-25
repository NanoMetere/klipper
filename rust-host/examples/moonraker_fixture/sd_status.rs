// SPDX-License-Identifier: GPL-3.0-only
//! Two independent in-memory read-only projections. No command is dispatched.
use klipper_host_protocol::{
    print_stats::{Extrusion, State, Stats},
    virtual_sd::{publish_unselected_status, Job},
    webhooks::Snapshot,
};
use std::io;

const FILE: &str = "상태.gcode";
const PATH: &str = "/virtual/상태.gcode";
const SOURCE: &[u8] = "; 한글\r\nG90\n".as_bytes();

fn err(error: impl std::fmt::Debug) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, format!("{error:?}"))
}

pub struct SdStatus {
    sd: Job<'static>,
    stats: Stats,
    token: Option<u64>,
}

impl SdStatus {
    pub fn new() -> io::Result<Self> {
        let mut stats = Stats::default();
        stats.select(FILE).map_err(err)?;
        Ok(Self {
            sd: Job::select(SOURCE),
            stats,
            token: None,
        })
    }

    pub fn advance(&mut self, step: u8) -> io::Result<()> {
        let at = f64::from(step);
        let e = Extrusion {
            position: 0.0,
            factor: 1.0,
        };
        match step {
            1 => {
                self.sd.resume().map_err(err)?;
                self.stats.note_start(at, e).map_err(err)?;
            }
            2 => {
                let line = self
                    .sd
                    .issue()
                    .map_err(err)?
                    .ok_or_else(|| err("missing SD line"))?;
                self.token = Some(line.token);
                // Like stock, a requested pause cannot undo an already-issued line.
                self.sd.pause().map_err(err)?;
            }
            3 => {
                let token = self.token.take().ok_or_else(|| err("missing SD token"))?;
                self.sd.commit(token, None).map_err(err)?;
                self.stats.note_pause(at, e).map_err(err)?;
            }
            4 => {}
            5 => {
                self.sd.resume().map_err(err)?;
                self.stats.note_start(at, e).map_err(err)?;
            }
            6 => {
                let line = self
                    .sd
                    .issue()
                    .map_err(err)?
                    .ok_or_else(|| err("missing final SD line"))?;
                self.sd.commit(line.token, None).map_err(err)?;
                if self.sd.issue().map_err(err)?.is_some() {
                    return Err(err("expected EOF"));
                }
                self.stats
                    .note_finish(at, State::Complete, "")
                    .map_err(err)?;
            }
            7 => *self = Self::new()?,
            9 => self.stats = Stats::default(),
            8 => {
                // Cancel a freshly selected file after the start. No job endpoint
                // is exposed; only this local deterministic fixture drives it.
                self.sd.resume().map_err(err)?;
                self.stats.note_start(at, e).map_err(err)?;
                self.sd.cancel().map_err(err)?;
                self.stats
                    .note_finish(at, State::Cancelled, "")
                    .map_err(err)?;
            }
            _ => return Err(err("unknown SD fixture step")),
        }
        Ok(())
    }

    pub fn publish(&mut self, snapshot: &mut Snapshot, step: u8) {
        if step == 9 {
            assert!(publish_unselected_status(snapshot));
        } else {
            assert!(self.sd.publish_status(PATH, snapshot));
        }
        let sampled = self
            .stats
            .observe(
                f64::from(step),
                Extrusion {
                    position: 0.0,
                    factor: 1.0,
                },
            )
            .unwrap();
        assert!(sampled.publish_status(snapshot));
    }
}
