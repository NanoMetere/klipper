// SPDX-License-Identifier: GPL-3.0-only
//! In-memory fixed fixture; stdin/stdout only, no socket, printer or execution.
use klipper_host_protocol::webhooks::{Session, Snapshot};
use serde_json::json;
use std::io::{self, Read, Write};

fn main() -> io::Result<()> {
    let mut status = Snapshot::default();
    status.info =
        json!({"state":"ready","state_message":"Offline fixture","software_version":"fixture-v1"})
            .as_object()
            .unwrap()
            .clone();
    status.help = json!({"G90":"Absolute XYZ", "M83":"Relative extrusion"})
        .as_object()
        .unwrap()
        .clone();
    for (name, value) in [
        (
            "toolhead",
            json!({"position":[1.,2.,3.,0.],"homed_axes":"xyz"}),
        ),
        (
            "gcode_move",
            json!({"speed":30.,"absolute_coordinates":true}),
        ),
    ] {
        assert!(status.set_object(name.into(), value.as_object().unwrap().clone()));
    }
    let mut input = io::stdin().lock();
    let mut output = io::stdout().lock();
    let mut bytes = [0; 13]; // deliberately fragment requests
    let mut session = Session::default();
    loop {
        let read = input.read(&mut bytes)?;
        if read == 0 {
            break;
        }
        let mut result = Ok(());
        session.feed(&bytes[..read], &status, 7.0, |line| {
            if result.is_ok() {
                result = output.write_all(line);
            }
        });
        result?;
    }
    output.flush()
}
