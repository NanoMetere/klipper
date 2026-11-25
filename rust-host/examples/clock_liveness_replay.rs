// SPDX-License-Identifier: GPL-3.0-only
//! Stock-style clock liveness policy, no I/O or job dispatch.
use std::io::{self, Read, Write};

use klipper_host_protocol::clock_liveness::{ClockLiveness, TimeoutAction};

fn main() -> io::Result<()> {
    let mut out = io::BufWriter::new(io::stdout().lock());
    let mut input = String::new();
    io::stdin().lock().take(65_537).read_to_string(&mut input)?;
    if input.len() > 65_536 {
        return Err(io::ErrorKind::InvalidData.into());
    }
    for line in input.lines() {
        let Some((mode, program)) = line.split_once(' ') else {
            return Err(io::ErrorKind::InvalidData.into());
        };
        if !matches!(mode, "0" | "1") || program.is_empty() || program.len() > 512 {
            return Err(io::ErrorKind::InvalidData.into());
        }
        let mut watch = ClockLiveness::default();
        let mut sent = 0;
        let mut aborts = 0;
        for (i, opcode) in program.bytes().enumerate() {
            let mut due = 0;
            match opcode {
                b'q' => {
                    due = (watch
                        .query_sent(i as f64 * 0.125)
                        .map_err(|_| io::ErrorKind::InvalidData)?
                        * 10000.)
                        .round() as u64;
                    sent += 1;
                }
                b'r' => watch.reply_received(),
                b'c' => {
                    aborts += u8::from(
                        watch.check_timeout(mode == "1") == TimeoutAction::LostCommunication,
                    )
                }
                _ => return Err(io::ErrorKind::InvalidData.into()),
            }
            writeln!(
                out,
                "{} {} {} {} {}",
                u8::from(watch.is_active()),
                u8::from(watch.timeout_reported()),
                aborts,
                sent,
                due
            )?;
        }
    }
    out.flush()
}
