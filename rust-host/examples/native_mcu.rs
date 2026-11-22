// SPDX-License-Identifier: GPL-3.0-only
//! Test helper: read and write a disposable Linux MCU PTY inherited on stdin/stdout.
#[cfg(target_os = "linux")]
mod linux {
    use klipper_host_protocol::{
        clock::{ClockMessages, Error as ClockError, Response},
        clock_sync::{PrimaryClock, Sample, Update},
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

    struct Reply<T> {
        value: T,
        sent: Option<f64>,
        received: f64,
    }

    struct Link {
        rx: Receiver,
        epoch: Instant,
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
        ) -> Result<Reply<T>, String> {
            let mut query = ReadQuery::new(self.sequence);
            let start = Instant::now();
            let mut result = None;
            let mut started = false;
            let mut sent = None;
            let mut received = 0.0;
            let mut bytes = [0; 256];
            loop {
                match query.poll(start.elapsed()) {
                    Action::Send {
                        sequence,
                        prepend_sync,
                    } => {
                        let frame = request(sequence)?;
                        if prepend_sync {
                            sent = None;
                            self.frame_retries += 1;
                            let mut wire = [0x7e; 65];
                            wire[1..=frame.len()].copy_from_slice(&frame);
                            self.writer
                                .write_all(&wire[..=frame.len()])
                                .map_err(|e| e.to_string())?;
                        } else {
                            self.response_retries += usize::from(started);
                            // PTY has zero modeled wire time and no other outgoing
                            // flight. A fresh app retry receives a fresh boundary.
                            sent = Some(1.0 + self.epoch.elapsed().as_secs_f64());
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
                        let read_end = Instant::now();
                        let now = read_end.duration_since(start);
                        let stats = self.rx.feed(&bytes[..count], |frame| {
                            if result.is_some() || !query.expects(frame.sequence) {
                                return;
                            }
                            if !frame.payload.is_empty() {
                                result = accept(frame).transpose();
                            }
                            query.observe(now, frame.sequence, result.is_some());
                        });
                        if result.is_some() {
                            received = 1.0 + read_end.duration_since(self.epoch).as_secs_f64();
                        }
                        self.discarded += stats.discarded;
                    }
                    Action::Done { next_sequence } => {
                        self.sequence = next_sequence;
                        let value = result.ok_or("read completed without a response")??;
                        return Ok(Reply {
                            value,
                            sent,
                            received,
                        });
                    }
                    Action::Failed(err) => return Err(format!("read failed: {err:?}")),
                }
            }
        }
    }

    fn read_clock(
        link: &mut Link,
        codec: &ClockMessages,
        uptime: bool,
    ) -> Result<Reply<Response>, String> {
        link.exchange(
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
        )
    }

    fn clock_trace(
        link: &mut Link,
        codec: &ClockMessages,
        hz: f64,
        count: usize,
    ) -> Result<(), String> {
        let uptime = read_clock(link, codec, true)?;
        let Response::Uptime(ticks) = uptime.value else {
            return Err("missing uptime".into());
        };
        let sent = uptime
            .sent
            .ok_or("untimed uptime cannot seed the clock model")?;
        let mut clock = PrimaryClock::from_uptime(hz, ticks, sent).map_err(|e| format!("{e:?}"))?;
        eprintln!("SEED {hz} {ticks} {sent:.17}");
        let mut release = None;
        for i in 0..count {
            let warm = i < 8;
            std::thread::sleep(if warm {
                Duration::from_millis(50)
            } else {
                Duration::from_micros(983_900)
            });
            let prev = (clock.last_clock(), clock.estimate());
            let reply = read_clock(link, codec, false)?;
            let Response::Clock(raw) = reply.value else {
                return Err("missing clock".into());
            };
            let sample = reply.sent.map_or(Sample::Untimed, |sent| Sample::Timed {
                sent,
                received: reply.received,
            });
            let update = clock
                .observe(raw, sample, warm)
                .map_err(|e| format!("{e:?}"))?;
            let state = clock.estimate();
            let kind = match update {
                Update::Untimed => {
                    if clock.last_clock() <= prev.0 || state != prev.1 {
                        return Err(
                            "untimed clock should advance ticks but preserve estimates".into()
                        );
                    }
                    "untimed"
                }
                Update::Outlier => "outlier",
                Update::Updated(next) => {
                    release = Some(next);
                    "updated"
                }
            };
            let low_sent = reply.sent.unwrap_or(0.0);
            let warm = usize::from(warm);
            let last = clock.last_clock();
            eprint!(
                "CLOCK {raw} {low_sent:.17} {:.17} {warm} {last} {kind} {:.17} {:.17} {:.17}",
                reply.received, state.at, state.ticks, state.hz
            );
            if let Some(next) = release {
                eprint!(" {:.17} {} {:.17}", next.at, next.ticks, next.hz);
            }
            eprintln!();
        }
        eprintln!("DONE {} {}", link.frame_retries, link.response_retries);
        Ok(())
    }

    pub fn run() -> Result<(), String> {
        let mut argv = std::env::args().skip(1);
        let arg = argv.next().ok_or("missing sample count")?;
        let trace = arg == "--clock-trace";
        let number = if trace {
            argv.next().ok_or("missing trace size")?
        } else {
            arg
        };
        let count = number.parse::<usize>().map_err(|e| e.to_string())?;
        let bounds = if trace { 8..=25 } else { 1..=100_000 };
        if !bounds.contains(&count) || argv.next().is_some() {
            return Err("sample count out of bounds".into());
        }
        // Unbuffered reads ensure poll sees all unread input. The harness supplies
        // the disposable PTY descriptors and enforces a process-wide timeout.
        let mut link = Link {
            rx: Receiver::new(),
            epoch: Instant::now(),
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
                    payload: &payload.value,
                })
                .map_err(|e| format!("{e:?}"))?
            {
                break;
            }
        }
        let dictionary = identify.dictionary().map_err(|e| format!("{e:?}"))?;
        let codec = ClockMessages::from_dictionary(&dictionary).map_err(|e| format!("{e:?}"))?;
        if trace {
            let hz = dictionary
                .config
                .get("CLOCK_FREQ")
                .and_then(serde_json::Value::as_f64)
                .ok_or("CLOCK_FREQ is missing or not numeric")?;
            return clock_trace(&mut link, &codec, hz, count);
        }
        let init_ns = init.elapsed().as_nanos();
        let mut last_uptime = 0;
        let mut timings = Vec::with_capacity(count);
        for sample in 0..count {
            let uptime = sample % 8 == 0;
            let start = Instant::now();
            let response = read_clock(&mut link, &codec, uptime)?.value;
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
