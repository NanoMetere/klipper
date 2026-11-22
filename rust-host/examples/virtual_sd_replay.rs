// SPDX-License-Identifier: GPL-3.0-only
//! Replays file-position observations without dispatching any printer commands.

use klipper_host_protocol::virtual_sd::Job;
use std::{
    env, fs,
    io::{self, Write},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args().skip(1);
    let source = fs::read(args.next().ok_or("missing fixture path")?)?;
    let start = args.next().ok_or("missing start")?.parse()?;
    let pause: usize = args.next().ok_or("missing pause")?.parse()?;
    let rewind: usize = args.next().ok_or("missing rewind")?.parse()?;
    let fail: usize = args.next().ok_or("missing fail")?.parse()?;
    assert!(args.next().is_none());
    let mut job = Job::select(&source);
    job.seek_paused(start).unwrap();
    job.resume().unwrap();
    let mut out = io::BufWriter::new(io::stdout().lock());
    let mut index = 0;
    while let Some(line) = job.issue().unwrap() {
        index += 1;
        write!(out, "{} {} ", line.start, line.next)?;
        for byte in line.source.as_bytes() {
            write!(out, "{byte:02x}")?;
        }
        writeln!(out)?;
        if index == fail {
            job.fail(line.token).unwrap();
            break;
        }
        if index == pause {
            job.pause().unwrap();
        }
        job.commit(line.token, (index == rewind).then_some(start))
            .unwrap();
        if index == pause {
            writeln!(out, "PAUSE {}", job.snapshot().position)?;
            job.resume().unwrap();
        }
    }
    writeln!(
        out,
        "{:?} {}",
        job.snapshot().state,
        job.snapshot().position
    )?;
    Ok(())
}
