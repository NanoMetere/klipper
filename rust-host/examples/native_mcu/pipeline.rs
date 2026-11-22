// SPDX-License-Identifier: GPL-3.0-only
//! Disposable native Linux MCU exercise. No movement or heating commands.

use crate::linux::{wait_readable, Link};
use klipper_host_protocol::{
    flight::{Flight, ReceiveError, Retry},
    identify::{identify_payload, identify_response},
    planner::{Decision, Planner, Priority, Snapshot, Wire},
};
use std::{
    io::{self, Read, Write},
    os::fd::AsFd,
    time::{Duration, Instant},
};

const CHUNK: usize = 16;
const ACK_TIMEOUT: Duration = Duration::from_millis(25);
const WAVE_TIMEOUT: Duration = Duration::from_secs(1);

pub(super) fn run(
    mut link: Link,
    reference: &[u8],
    count: usize,
    width: usize,
    receive_window: usize,
) -> Result<(), String> {
    if !(1..=128).contains(&width) || reference.len() < width * CHUNK || !count.is_multiple_of(width) {
        return Err("invalid pipeline width or count".into());
    }
    let mut flight = Flight::established(16 + u64::from(link.sequence));
    let mut planner = Planner::new(1, width).map_err(|e| format!("{e:?}"))?;
    let (mut peak, mut retries, mut largest_frame_count) = (0, 0, 0);
    let (mut discarded, mut invalid) = (0, 0);
    let mut wave_ns = Vec::new();
    let mut read_buf = [0; 4096];
    let commands = (0..width)
        .map(|i| identify_payload((i * CHUNK) as u32, CHUNK))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| format!("{e:?}"))?;
    let mut seen = [false; 128];
    for _ in 0..count / width {
        seen.fill(false);
        let mut missing = width;
        let mut retry_at = None;
        let mut frames_this_wave = 0;
        let begin = Instant::now();
        let expires = begin + WAVE_TIMEOUT;
        for payload in &commands {
            planner
                .push(0, 0, Priority::Deadline(0), payload)
                .map_err(|e| format!("{e:?}"))?;
        }
        loop {
            if Instant::now() >= expires {
                return Err(format!(
                    "pipeline deadline: {missing} replies missing, {} frames pending",
                    flight.pending_frames()
                ));
            }
            if missing == 0 && flight.pending_frames() == 0 {
                break;
            }
            let now = 1.0 + link.epoch.elapsed().as_secs_f64();
            match planner
                .plan(
                    Wire::Pty,
                    None,
                    Snapshot {
                        now,
                        link_idle: now,
                        burst_bytes: 0,
                        in_flight: flight.pending_frames(),
                        outstanding_bytes: flight.outstanding_bytes(),
                        prior_ack_bytes: flight.prior_ack_bytes(),
                        receive_window,
                    },
                )
                .map_err(|e| format!("{e:?}"))?
            {
                Decision::Send(payload) => {
                    flight
                        .send(payload.as_slice(), receive_window, |bytes| {
                            link.writer.write(bytes)
                        })
                        .map_err(|e| format!("{e:?}"))?;
                    frames_this_wave += 1;
                    peak = peak.max(flight.pending_frames());
                    retry_at.get_or_insert_with(|| Instant::now() + ACK_TIMEOUT);
                    continue;
                }
                Decision::Idle | Decision::AwaitAck { .. } => {}
                other => return Err(format!("unexpected PTY decision: {other:?}")),
            }
            if retry_at.is_some_and(|at| Instant::now() >= at) {
                flight
                    .retry(Retry::Timeout, |bytes| link.writer.write(bytes))
                    .map_err(|e| format!("{e:?}"))?;
                retries += 1;
                retry_at = Some(Instant::now() + ACK_TIMEOUT);
            }
            let until = retry_at.unwrap_or(expires).min(expires);
            match wait_readable(
                link.reader.as_fd(),
                until.saturating_duration_since(Instant::now()),
            ) {
                Ok(false) => continue,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.to_string()),
                Ok(true) => {}
            }
            let n = match link.reader.read(&mut read_buf) {
                Ok(0) => return Err("unexpected PTY EOF".into()),
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.to_string()),
            };
            let mut error = None;
            let mut fast = false;
            let stats = link.rx.feed(&read_buf[..n], |frame| {
                if error.is_some() {
                    return;
                }
                let p = match flight.observe_decoded(frame) {
                    Ok(p) => p,
                    Err(ReceiveError::OutsideWindow) => {
                        invalid += 1;
                        return;
                    }
                    Err(e) => {
                        error = Some(format!("{e:?}"));
                        return;
                    }
                };
                if p.retired != 0 {
                    fast = false;
                    retry_at = (flight.pending_frames() != 0).then(|| Instant::now() + ACK_TIMEOUT);
                }
                fast |= p.fast_retry;
                if let Ok((offset, bytes)) = identify_response(frame) {
                    // Ignore late bootstrap replies (size 40 or a terminal empty);
                    // a measured response must match the exact bytes and offset.
                    let offset = offset as usize;
                    if bytes.len() != CHUNK || !offset.is_multiple_of(CHUNK) || offset >= width * CHUNK {
                        return;
                    }
                    if seen[offset / CHUNK] || bytes != &reference[offset..offset + CHUNK] {
                        error = Some(format!("duplicate or changed pipeline reply at {offset}"));
                        return;
                    }
                    seen[offset / CHUNK] = true;
                    missing -= 1;
                }
            });
            if let Some(error) = error {
                return Err(error);
            }
            discarded += stats.discarded;
            if fast {
                flight
                    .retry(Retry::DuplicateAck, |bytes| link.writer.write(bytes))
                    .map_err(|e| format!("{e:?}"))?;
                retries += 1;
                retry_at = Some(Instant::now() + ACK_TIMEOUT);
            }
        }
        largest_frame_count = largest_frame_count.max(frames_this_wave);
        wave_ns.push(begin.elapsed().as_nanos());
    }
    let prefix: String = reference[..width * CHUNK]
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    eprintln!(
        "PIPE {peak} {largest_frame_count} {retries} {discarded} {invalid} {prefix} {wave_ns:?}"
    );
    Ok(())
}
