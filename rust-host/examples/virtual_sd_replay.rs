// SPDX-License-Identifier: GPL-3.0-only
//! Replays file-position observations without dispatching any printer commands.

use klipper_host_protocol::virtual_sd::Job;
use std::{
    env, fs,
    io::{self, Write},
};

#[cfg(feature = "webhooks")]
fn status(out: &mut impl Write, job: Option<&Job<'_>>) -> io::Result<()> {
    use klipper_host_protocol::webhooks::{Session, Snapshot};
    let mut snapshot = Snapshot::default();
    assert!(match job {
        Some(job) => job.publish_status("/virtual/input.gcode", &mut snapshot),
        None => klipper_host_protocol::virtual_sd::publish_unselected_status(&mut snapshot),
    });
    let mut session = Session::default();
    let mut reply = None;
    session.feed(
        b"{\"id\":1,\"method\":\"objects/query\",\"params\":{\"objects\":{\"virtual_sdcard\":null}}}\x03",
        &snapshot, 1.0, &mut |raw: &[u8]| {
            reply = Some(serde_json::from_slice::<serde_json::Value>(&raw[..raw.len()-1]).unwrap());
        },
    );
    writeln!(
        out,
        "STATUS {}",
        reply.unwrap()["result"]["status"]["virtual_sdcard"]
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args().skip(1);
    let source = fs::read(args.next().ok_or("missing fixture path")?)?;
    let start = args.next().ok_or("missing start")?.parse()?;
    let pause: usize = args.next().ok_or("missing pause")?.parse()?;
    let rewind: usize = args.next().ok_or("missing rewind")?.parse()?;
    let fail: usize = args.next().ok_or("missing fail")?.parse()?;
    #[cfg(feature = "webhooks")]
    let (with_status, with_cancel) = match args.next().as_deref() {
        Some("--status") => (true, false),
        Some("--status-cancel") => (true, true),
        None => (false, false),
        _ => return Err("unknown fixture option".into()),
    };
    assert!(args.next().is_none());
    let mut job = Job::select(&source);
    job.seek_paused(start).unwrap();
    let mut out = io::BufWriter::new(io::stdout().lock());
    #[cfg(feature = "webhooks")]
    if with_status {
        status(&mut out, Some(&job))?;
    }
    job.resume().unwrap();
    let mut index = 0;
    while let Some(line) = job.issue().unwrap() {
        index += 1;
        write!(out, "{} {} ", line.start, line.next)?;
        for byte in line.source.as_bytes() {
            write!(out, "{byte:02x}")?;
        }
        writeln!(out)?;
        #[cfg(feature = "webhooks")]
        if with_status {
            status(&mut out, Some(&job))?;
        }
        if index == fail {
            job.fail(line.token).unwrap();
            break;
        }
        if index == pause {
            job.pause().unwrap();
            #[cfg(feature = "webhooks")]
            if with_status {
                status(&mut out, Some(&job))?;
            }
        }
        job.commit(line.token, (index == rewind).then_some(start))
            .unwrap();
        if index == pause {
            writeln!(out, "PAUSE {}", job.snapshot().position)?;
            #[cfg(feature = "webhooks")]
            if with_status {
                status(&mut out, Some(&job))?;
            }
            #[cfg(feature = "webhooks")]
            if with_cancel {
                job.cancel().unwrap();
                status(&mut out, Some(&job))?;
                // In-memory accounting retains the diagnostic checkpoint;
                // cancellation and reset both expose an empty SD slot.
                assert!(job.snapshot().position > 0);
                status(&mut out, None)?;
                return Ok(());
            }
            job.resume().unwrap();
        }
    }
    writeln!(
        out,
        "{:?} {}",
        job.snapshot().state,
        job.snapshot().position
    )?;
    #[cfg(feature = "webhooks")]
    if with_status {
        status(&mut out, Some(&job))?;
    }
    Ok(())
}
