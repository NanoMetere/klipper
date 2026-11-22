// SPDX-License-Identifier: GPL-3.0-only
//! Test helper: read and write a disposable Linux MCU PTY inherited on stdin/stdout.
use klipper_host_protocol::{
    clock::{ClockMessages, Error as ClockError, Response},
    identify::Identify,
    stream::Receiver,
    Frame,
};
use std::io::{self, Read, Write};
use std::time::Instant;

fn exchange<T>(
    reader: &mut impl Read,
    writer: &mut impl Write,
    rx: &mut Receiver,
    request: &[u8],
    discarded: &mut usize,
    mut accept: impl FnMut(Frame<'_>) -> Result<Option<T>, String>,
) -> Result<T, String> {
    writer.write_all(request).map_err(|e| e.to_string())?;
    writer.flush().map_err(|e| e.to_string())?;
    let expected = (request[1] + 1) & 15;
    let mut bytes = [0; 256];
    loop {
        let count = match reader.read(&mut bytes) {
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e.to_string()),
            Ok(0) => return Err("unexpected PTY EOF".into()),
            Ok(n) => n,
        };
        let mut result = Ok(None);
        let stats = rx.feed(&bytes[..count], |frame| {
            if frame.payload.is_empty() || frame.sequence != expected {
                return;
            }
            if matches!(result, Ok(None)) {
                result = accept(frame);
            }
        });
        *discarded += stats.discarded;
        if let Some(value) = result? {
            return Ok(value);
        }
    }
}

fn main() -> Result<(), String> {
    let count: usize = std::env::args()
        .nth(1)
        .ok_or("missing sample count")?
        .parse::<usize>()
        .map_err(|e| e.to_string())?;
    if !(1..=100_000).contains(&count) {
        return Err("sample count out of bounds".into());
    }
    let (mut reader, mut writer) = (io::stdin().lock(), io::stdout().lock());
    let mut rx = Receiver::new();
    let mut identify = Identify::default();
    let mut seq = 0u8;
    let mut discarded = 0;
    let init = Instant::now();
    loop {
        let request = identify.request(seq).map_err(|e| format!("{e:?}"))?;
        let done = exchange(
            &mut reader,
            &mut writer,
            &mut rx,
            &request,
            &mut discarded,
            |frame| {
                if frame.payload[0] != 0 {
                    return Ok(None);
                }
                identify
                    .accept(frame)
                    .map(Some)
                    .map_err(|e| format!("{e:?}"))
            },
        )?;
        seq = (seq + 1) & 15;
        if done {
            break;
        }
    }
    let dictionary = identify.dictionary().map_err(|e| format!("{e:?}"))?;
    let codec = ClockMessages::from_dictionary(&dictionary).map_err(|e| format!("{e:?}"))?;
    let init_ns = init.elapsed().as_nanos();
    let mut last_uptime = 0;
    let mut timings = Vec::with_capacity(count);
    for sample in 0..count {
        let uptime = sample % 8 == 0;
        let start = Instant::now();
        let request = if uptime {
            codec.get_uptime(seq)
        } else {
            codec.get_clock(seq)
        }
        .map_err(|e| format!("{e:?}"))?;
        let response = exchange(
            &mut reader,
            &mut writer,
            &mut rx,
            &request,
            &mut discarded,
            |frame| match codec.response(frame) {
                Ok(value) => Ok(Some(value)),
                Err(ClockError::UnexpectedMessage) => Ok(None),
                Err(e) => Err(format!("{e:?}")),
            },
        )?;
        timings.push(start.elapsed().as_nanos());
        match response {
            Response::Uptime(ticks) if uptime && ticks >= last_uptime => last_uptime = ticks,
            Response::Clock(_) if !uptime => {}
            _ => return Err(format!("unexpected response: {response:?}")),
        }
        seq = (seq + 1) & 15;
    }
    eprintln!("{init_ns} {discarded} {timings:?}");
    Ok(())
}
