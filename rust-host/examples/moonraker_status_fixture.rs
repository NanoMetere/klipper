// SPDX-License-Identifier: GPL-3.0-only
//! Software-only status fixture using an inherited socketpair end on stdin.
#[cfg(unix)]
#[path = "moonraker_fixture/program.rs"]
mod program;

#[cfg(unix)]
fn main() -> std::io::Result<()> {
    use klipper_host_protocol::{status_socket::StatusConnection, webhooks::Snapshot};
    use serde_json::json;
    use std::{
        env, fs,
        io::{self, ErrorKind},
        os::{fd::AsFd, unix::net::UnixStream},
        thread,
        time::{Duration, Instant},
    };
    let mut argv = env::args_os().skip(1);
    let path = argv
        .next()
        .ok_or_else(|| io::Error::new(ErrorKind::InvalidInput, "expected a fixture state file"))?;
    let control = argv.next();
    if argv.next().is_some() {
        return Err(io::Error::new(
            ErrorKind::InvalidInput,
            "unexpected fixture argument",
        ));
    }
    let mut program = control
        .as_ref()
        .map(|_| program::FixtureProgram::new())
        .transpose()?;
    let stream = UnixStream::from(io::stdin().as_fd().try_clone_to_owned()?);
    stream.peer_addr()?;
    let mut client = StatusConnection::from_stream(stream)?;
    let mut snapshot = Snapshot::default();
    let started = Instant::now();
    let mut observed = None;
    loop {
        if started.elapsed() > Duration::from_secs(20) {
            return Err(io::Error::new(
                ErrorKind::TimedOut,
                "fixture connection expired",
            ));
        }
        let data = fs::read_to_string(&path)?;
        let (state, position) = data
            .trim()
            .split_once(' ')
            .ok_or_else(|| io::Error::new(ErrorKind::InvalidData, "invalid fixture state"))?;
        let x = position
            .parse::<u8>()
            .map_err(|_| io::Error::new(ErrorKind::InvalidData, "invalid fixture position"))?;
        if !matches!(state, "startup" | "ready" | "shutdown") {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                "invalid fixture state",
            ));
        }
        if observed
            .as_ref()
            .is_none_or(|(old, p)| old != state || *p != x)
        {
            observed = Some((state.to_owned(), x));
            snapshot.info = json!({
                "state": state, "state_message":"Offline Moonraker fixture",
                "software_version":"rust-status-fixture",
                "klipper_path":"/unused/klipper", "python_path":"/unused/python"
            })
            .as_object()
            .unwrap()
            .clone();
            for (name, value) in [
                (
                    "webhooks",
                    json!({"state":state, "state_message":"Offline Moonraker fixture"}),
                ),
                (
                    "toolhead",
                    json!({"position":[x, 2, 3, 0], "homed_axes":"xyz"}),
                ),
            ] {
                assert!(snapshot.set_object(name.into(), value.as_object().unwrap().clone()));
            }
        }
        if let (Some(path), Some(driver)) = (&control, &mut program) {
            let next = fs::read_to_string(path)?
                .trim()
                .parse::<u8>()
                .map_err(|error| io::Error::new(ErrorKind::InvalidData, error))?;
            driver.advance(next)?;
            driver.publish(&mut snapshot);
        }
        match client.poll(&snapshot, started.elapsed().as_secs_f64()) {
            Ok(_) => thread::sleep(Duration::from_millis(5)),
            Err(e) if matches!(e.kind(), ErrorKind::UnexpectedEof | ErrorKind::BrokenPipe) => {
                return Ok(());
            }
            Err(e) => return Err(e),
        }
    }
}

#[cfg(not(unix))]
fn main() {}
