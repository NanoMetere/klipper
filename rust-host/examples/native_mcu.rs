// SPDX-License-Identifier: GPL-3.0-only
//! Test helper: read and write a disposable Linux MCU PTY inherited on stdin/stdout.
#[cfg(target_os = "linux")]
mod linux {
    use klipper_host_protocol::{
        clock::{ClockMessages, Error as ClockError, Response},
        identify::Identify,
        read_query::{Action, ReadQuery},
        stream::Receiver,
        Frame,
    };
    use std::fs::File;
    use std::io::{self, Read, Write};
    use std::os::fd::{AsRawFd, BorrowedFd};
    use std::time::{Duration, Instant};

    #[repr(C)]
    struct PollFd {
        fd: i32,
        events: i16,
        revents: i16,
    }

    unsafe extern "C" {
        fn poll(fds: *mut PollFd, nfds: std::ffi::c_ulong, timeout: i32) -> i32;
    }

    fn wait_readable(fd: BorrowedFd<'_>, time: Duration) -> io::Result<bool> {
        const POLLIN: i16 = 0x001;
        let mut event = PollFd {
            fd: fd.as_raw_fd(),
            events: POLLIN,
            revents: 0,
        };
        let ms = time.as_nanos().div_ceil(1_000_000);
        // Safety: event is one initialized Linux pollfd, live until poll returns;
        // the borrowed file descriptor cannot be closed during this call.
        let rc = unsafe { poll(&mut event, 1, ms.min(i32::MAX as u128) as i32) };
        if rc < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(rc > 0)
    }

    struct Link {
        rx: Receiver,
        reader: File,
        writer: File,
        sequence: u8,
        discarded: usize,
        frame_retries: usize,
        response_retries: usize,
    }

    impl Link {
        fn exchange<T>(
            &mut self,
            request: impl Fn(u8) -> Result<Vec<u8>, String>,
            mut accept: impl FnMut(Frame<'_>) -> Result<Option<T>, String>,
        ) -> Result<T, String> {
            let mut query = ReadQuery::new(self.sequence);
            let start = Instant::now();
            let mut result = None;
            let mut started = false;
            let mut bytes = [0; 256];
            loop {
                match query.poll(start.elapsed()) {
                    Action::Send {
                        sequence,
                        prepend_sync,
                    } => {
                        let frame = request(sequence)?;
                        if prepend_sync {
                            self.frame_retries += 1;
                            let mut wire = [0x7e; 65];
                            wire[1..=frame.len()].copy_from_slice(&frame);
                            self.writer
                                .write_all(&wire[..=frame.len()])
                                .map_err(|e| e.to_string())?;
                        } else {
                            self.response_retries += usize::from(started);
                            self.writer.write_all(&frame).map_err(|e| e.to_string())?;
                        }
                        started = true;
                    }
                    Action::WaitUntil(at) => {
                        let remaining = at.saturating_sub(start.elapsed());
                        match wait_readable(std::os::fd::AsFd::as_fd(&self.reader), remaining) {
                            Ok(false) => continue,
                            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                            Err(e) => return Err(e.to_string()),
                            Ok(true) => {}
                        }
                        let count = match self.reader.read(&mut bytes) {
                            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                            Err(e) => return Err(e.to_string()),
                            Ok(0) => return Err("unexpected PTY EOF".into()),
                            Ok(n) => n,
                        };
                        let now = start.elapsed();
                        let stats = self.rx.feed(&bytes[..count], |frame| {
                            if result.is_some() || !query.expects(frame.sequence) {
                                return;
                            }
                            if !frame.payload.is_empty() {
                                result = accept(frame).transpose();
                            }
                            query.observe(now, frame.sequence, result.is_some());
                        });
                        self.discarded += stats.discarded;
                    }
                    Action::Done { next_sequence } => {
                        self.sequence = next_sequence;
                        return result.ok_or("read completed without a response")?;
                    }
                    Action::Failed(err) => return Err(format!("read failed: {err:?}")),
                }
            }
        }
    }

    pub fn run() -> Result<(), String> {
        let count = std::env::args()
            .nth(1)
            .ok_or("missing sample count")?
            .parse::<usize>()
            .map_err(|e| e.to_string())?;
        if !(1..=100_000).contains(&count) {
            return Err("sample count out of bounds".into());
        }
        // Unbuffered reads ensure poll sees all unread input. The harness supplies
        // the disposable PTY descriptors and enforces a process-wide timeout.
        let mut link = Link {
            rx: Receiver::new(),
            reader: File::open("/proc/self/fd/0").map_err(|e| e.to_string())?,
            writer: File::options()
                .write(true)
                .open("/proc/self/fd/1")
                .map_err(|e| e.to_string())?,
            sequence: 0,
            discarded: 0,
            frame_retries: 0,
            response_retries: 0,
        };
        let mut identify = Identify::default();
        let init = Instant::now();
        // Encode from the same state used to construct the request. The reply is
        // accepted separately, after the request has finished, so retries reuse the offset.
        loop {
            let payload = link.exchange(
                |seq| identify.request(seq).map_err(|e| format!("{e:?}")),
                |frame| Ok((frame.payload[0] == 0).then(|| frame.payload.to_vec())),
            )?;
            if identify
                .accept(Frame {
                    sequence: link.sequence,
                    payload: &payload,
                })
                .map_err(|e| format!("{e:?}"))?
            {
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
            let response = link.exchange(
                |seq| {
                    if uptime {
                        codec.get_uptime(seq)
                    } else {
                        codec.get_clock(seq)
                    }
                    .map_err(|e| format!("{e:?}"))
                },
                |frame| match codec.response(frame) {
                    Ok(value @ Response::Uptime(_)) if uptime => Ok(Some(value)),
                    Ok(value @ Response::Clock(_)) if !uptime => Ok(Some(value)),
                    Ok(_) | Err(ClockError::UnexpectedMessage) => Ok(None),
                    Err(e) => Err(format!("{e:?}")),
                },
            )?;
            timings.push(start.elapsed().as_nanos());
            match response {
                Response::Uptime(ticks) if ticks >= last_uptime => last_uptime = ticks,
                Response::Clock(_) => {}
                _ => return Err(format!("unexpected response: {response:?}")),
            }
        }
        eprintln!(
            "{init_ns} {} {} {} {timings:?}",
            link.discarded, link.frame_retries, link.response_retries
        );
        Ok(())
    }
}

#[cfg(target_os = "linux")]
fn main() -> Result<(), String> {
    linux::run()
}

#[cfg(not(target_os = "linux"))]
fn main() -> Result<(), &'static str> {
    Err("the disposable PTY example requires Linux")
}
