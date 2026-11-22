// SPDX-License-Identifier: GPL-3.0-only
//! Test helper: read and write a disposable Linux MCU PTY inherited on stdin/stdout.
#[cfg(target_os = "linux")]
#[path = "native_mcu/pipeline.rs"]
mod pipeline;

#[cfg(target_os = "linux")]
#[path = "native_mcu/gpio_motion.rs"]
mod gpio_motion;

#[cfg(target_os = "linux")]
#[path = "native_mcu/gpio_dual.rs"]
mod gpio_dual;

#[cfg(target_os = "linux")]
#[path = "native_mcu/gpio_mixed.rs"]
mod gpio_mixed;

#[cfg(target_os = "linux")]
mod linux {
    use klipper_host_protocol::{
        clock::{ClockMessages, Error as ClockError, Response},
        clock_sync::{PrimaryClock, Sample, Update},
        encode,
        fixture_mcu::ShutdownWatch,
        identify::{Dictionary, Identify},
        mcu_config::{Error as ConfigError, Messages, Profile, Report, Session, Start},
        read_query::{Action, ReadQuery},
        stream::Receiver,
        trigger_sync::{
            Error as TriggerError, Messages as TriggerMessages, Reason, State as TriggerState,
        },
        Frame,
    };
    use std::fs::File;
    use std::io::{self, Read, Write};
    use std::os::fd::{AsFd, AsRawFd, BorrowedFd};
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

    pub(super) fn wait_readable(fd: BorrowedFd<'_>, time: Duration) -> io::Result<bool> {
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

    pub(super) struct Reply<T> {
        pub(super) value: T,
        sent: Option<f64>,
        received: f64,
    }

    pub(super) struct Link {
        pub(super) rx: Receiver,
        pub(super) epoch: Instant,
        pub(super) reader: File,
        pub(super) writer: File,
        pub(super) sequence: u8,
        pub(super) shutdown_watch: Option<ShutdownWatch>,
        discarded: usize,
        frame_retries: usize,
        response_retries: usize,
    }

    impl Link {
        pub(super) fn exchange<T>(
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
                            if let Some(watch) = &mut self.shutdown_watch {
                                watch.observe(frame.payload);
                            }
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

    pub(super) fn identify_fixture(link: &mut Link) -> Result<Dictionary, String> {
        let mut identify = Identify::default();
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
                return identify.dictionary().map_err(|e| format!("{e:?}"));
            }
        }
    }

    fn inherited_pty(read: u32, write: u32) -> Result<(Link, std::path::PathBuf), String> {
        let r = format!("/proc/self/fd/{read}");
        let w = format!("/proc/self/fd/{write}");
        let target = std::fs::read_link(&r).map_err(|e| e.to_string())?;
        if target != std::fs::read_link(&w).map_err(|e| e.to_string())?
            || !target.starts_with("/dev/pts")
        {
            return Err("fixture requires a disposable PTY descriptor".into());
        }
        Ok((
            Link {
                rx: Receiver::new(),
                epoch: Instant::now(),
                reader: File::open(r).map_err(|e| e.to_string())?,
                writer: File::options()
                    .write(true)
                    .open(w)
                    .map_err(|e| e.to_string())?,
                sequence: 0,
                shutdown_watch: None,
                discarded: 0,
                frame_retries: 0,
                response_retries: 0,
            },
            target,
        ))
    }

    pub(super) fn read_clock(
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

    fn receive_fixture<T>(
        link: &mut Link,
        expected_sequence: u8,
        timeout: Duration,
        timeout_error: &str,
        mut accept: impl FnMut(Frame<'_>) -> Result<Option<T>, String>,
    ) -> Result<T, String> {
        let deadline = Instant::now() + timeout;
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Err(timeout_error.into());
            }
            match wait_readable(link.reader.as_fd(), deadline - now) {
                Ok(false) => return Err(timeout_error.into()),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.to_string()),
                Ok(true) => {}
            }
            let mut buf = [0; 256];
            let read = match link.reader.read(&mut buf) {
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e.to_string()),
                Ok(0) => return Err("fixture peer closed".into()),
                Ok(n) => n,
            };
            let mut received = None;
            link.rx.feed(&buf[..read], |f| {
                if let Some(watch) = &mut link.shutdown_watch {
                    watch.observe(f.payload);
                }
                if f.sequence == expected_sequence && received.is_none() {
                    received = accept(f).transpose();
                }
            });
            if let Some(value) = received {
                return value;
            }
        }
    }

    fn fixture_write<T>(
        link: &mut Link,
        frame: &[u8],
        accept: impl FnMut(Frame<'_>) -> Result<Option<T>, String>,
    ) -> Result<T, String> {
        let next = (link.sequence + 1) & 15;
        // No message is ever resent after an unknown write outcome.
        link.writer.write_all(frame).map_err(|e| e.to_string())?;
        let value = receive_fixture(
            link,
            next,
            Duration::from_millis(700),
            "unknown config write outcome (no ACK or expected response)",
            accept,
        )?;
        link.sequence = next;
        Ok(value)
    }

    pub(super) fn fixture_ack(link: &mut Link, frame: &[u8]) -> Result<(), String> {
        fixture_write(link, frame, |f| {
            if f.payload.is_empty() {
                Ok(Some(()))
            } else {
                Err("MCU sent a response instead of a fixture ACK".into())
            }
        })
    }

    pub(super) fn read_config(link: &mut Link, codec: &Messages) -> Result<Report, String> {
        link.exchange(
            |seq| codec.get_config(seq).map_err(|e| format!("{e:?}")),
            |frame| match codec.report(frame) {
                Ok(report) => Ok(Some(report)),
                Err(ConfigError::UnexpectedMessage) => Ok(None),
                Err(e) => Err(format!("{e:?}")),
            },
        )
        .map(|reply| reply.value)
    }

    fn configure_fixture(
        link: &mut Link,
        codec: &Messages,
        trsync_count: u8,
    ) -> Result<(), String> {
        let profile = Profile::trsync_only(trsync_count, 3);
        let mut session = Session::new(&profile);
        let start = session
            .first(read_config(link, codec)?, false)
            .map_err(|e| format!("{e:?}"))?;
        let sends = if start == Start::NeedsConfiguration {
            let items = codec.payloads(&profile);
            for bytes in &items {
                let frame = encode(link.sequence, bytes).map_err(|e| format!("{e:?}"))?;
                fixture_ack(link, &frame)?;
            }
            items.len()
        } else {
            0
        };
        let final_report = read_config(link, codec)?;
        let slots = session.verify(final_report).map_err(|e| format!("{e:?}"))?;
        eprintln!(
            "CONFIG {start:?} {} {} {slots} {sends}",
            final_report.crc, final_report.move_count
        );
        // On the same live sequence, a different profile must never send.
        let wrong = Profile::trsync_only(trsync_count + 1, 3);
        let mut mismatch = Session::new(&wrong);
        if !matches!(
            mismatch.first(read_config(link, codec)?, false),
            Err(ConfigError::CrcMismatch { .. })
        ) {
            return Err("expected a local CRC mismatch".into());
        }
        let mut reuse = Session::new(&profile);
        if reuse.first(read_config(link, codec)?, false) != Ok(Start::ReuseMatching) {
            return Err("expected to reuse configured state".into());
        }
        let confirmed = read_config(link, codec)?;
        if reuse.verify(confirmed) != Ok(slots) {
            return Err("reused move slot count changed".into());
        }
        eprintln!("REUSE {} {slots} 0", confirmed.crc);
        Ok(())
    }

    fn trigger_report(
        codec: &TriggerMessages,
        frame: Frame<'_>,
    ) -> Result<Option<TriggerState>, String> {
        if frame.payload.is_empty() {
            return Ok(None);
        }
        match codec.response(frame) {
            Ok(state) if state.oid == 0 => Ok(Some(state)),
            Ok(_) | Err(TriggerError::UnexpectedMessage) => Ok(None),
            Err(e) => Err(format!("{e:?}")),
        }
    }

    fn trigger_fixture(
        link: &mut Link,
        codec: &TriggerMessages,
        clock: &ClockMessages,
        hz: f64,
    ) -> Result<(), String> {
        let err = |e| format!("{e:?}");
        let begin_arm = |link: &mut Link| {
            let arm = codec
                .arm(link.sequence, 0, 0, 0, Reason::CommsTimeout)
                .map_err(err)?;
            fixture_ack(link, &arm)
        };
        begin_arm(link)?;
        let stop = codec
            .trigger(link.sequence, 0, Reason::HostRequest)
            .map_err(err)?;
        let report = fixture_write(link, &stop, |f| trigger_report(codec, f))?;
        if report.can_trigger || report.reason != Reason::HostRequest || report.clock != 0 {
            return Err(format!("unexpected host-trigger state: {report:?}"));
        }
        eprintln!("TRIGGER host {}", report.reason.code());

        begin_arm(link)?;
        let read = read_clock(link, clock, false)?;
        let Response::Clock(current) = read.value else {
            return Err("expected fixture clock".into());
        };
        let expire = current.wrapping_add((hz * 0.15) as u32);
        let arm_timeout = codec.expire(link.sequence, 0, expire).map_err(err)?;
        fixture_ack(link, &arm_timeout)?;
        let sequence = link.sequence;
        let timeout = receive_fixture(
            link,
            sequence,
            Duration::from_secs(1),
            "no fixture timeout response",
            |f| trigger_report(codec, f),
        )?;
        let lateness = timeout.clock.wrapping_sub(expire);
        if timeout.can_trigger || timeout.reason != Reason::CommsTimeout || lateness > hz as u32 {
            return Err(format!("unexpected timeout state: {timeout:?}"));
        }
        let stop = codec
            .trigger(link.sequence, 0, Reason::HostRequest)
            .map_err(err)?;
        let stopped = fixture_write(link, &stop, |f| trigger_report(codec, f))?;
        if stopped.can_trigger || stopped.reason != Reason::CommsTimeout || stopped.clock != 0 {
            return Err("firmware changed the terminal trigger reason".into());
        }
        eprintln!("TRIGGER timeout {} {}", stopped.reason.code(), lateness);
        Ok(())
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
        if matches!(arg.as_str(), "--gpio-dual" | "--gpio-dual-packed") {
            let fd = argv
                .next()
                .ok_or("secondary simulator descriptor required")?
                .parse::<u32>()
                .map_err(|e| e.to_string())?;
            if !(3..=1024).contains(&fd) || argv.next().is_some() {
                return Err("invalid secondary simulator descriptor".into());
            }
            let (primary, a) = inherited_pty(0, 1)?;
            let (secondary, b) = inherited_pty(fd, fd)?;
            if a == b {
                return Err("the two fixture PTYs must be independent".into());
            }
            return crate::gpio_dual::run([primary, secondary], arg == "--gpio-dual-packed");
        }

        let trace = arg == "--clock-trace";
        let pipe = arg == "--pipeline";
        let trigger = arg == "--trigger-fixture";
        let gpio_multitool = match arg.as_str() {
            "--gpio-multitool" => 2,
            "--gpio-sparse" => 3,
            _ => 0,
        };
        let gpio_dock = arg == "--gpio-dock";
        let gpio_shutdown = arg == "--gpio-mixed-shutdown";
        let gpio_mixed = arg == "--gpio-mixed" || gpio_shutdown;
        let gpio_batched = arg == "--gpio-batched" || gpio_multitool != 0 || gpio_dock;
        let gpio_extrusion = arg == "--gpio-extrusion" || gpio_batched;
        let gpio_fixture = arg == "--gpio-motion" || gpio_extrusion || gpio_mixed;
        let config_fixture = arg == "--config-fixture" || trigger || gpio_fixture;
        let number = if trace || pipe || config_fixture {
            argv.next().ok_or("missing trace size")?
        } else {
            arg
        };
        let count = number.parse::<usize>().map_err(|e| e.to_string())?;
        let bounds = if trace {
            8..=25
        } else if config_fixture {
            1..=16
        } else {
            1..=100_000
        };
        let width = if pipe {
            argv.next()
                .unwrap_or_default()
                .parse::<usize>()
                .map_err(|e| e.to_string())?
        } else {
            0
        };
        if !bounds.contains(&count) || argv.next().is_some() {
            return Err("sample count out of bounds".into());
        }
        if config_fixture {
            let stdin = std::fs::read_link("/proc/self/fd/0").map_err(|e| e.to_string())?;
            let stdout = std::fs::read_link("/proc/self/fd/1").map_err(|e| e.to_string())?;
            if stdin != stdout || !stdin.starts_with("/dev/pts") {
                return Err("config fixture requires a single disposable PTY".into());
            }
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
            shutdown_watch: None,
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
        if gpio_mixed {
            return crate::gpio_mixed::run(link, &dictionary, gpio_shutdown);
        }
        if gpio_fixture {
            return crate::gpio_motion::run(
                link,
                &dictionary,
                gpio_extrusion,
                gpio_batched,
                gpio_multitool,
                gpio_dock,
            );
        }
        if config_fixture {
            let messages = Messages::from_dictionary(&dictionary).map_err(|e| format!("{e:?}"))?;
            configure_fixture(&mut link, &messages, count as u8)?;
            if trigger {
                let codec =
                    TriggerMessages::from_dictionary(&dictionary).map_err(|e| format!("{e:?}"))?;
                let clock =
                    ClockMessages::from_dictionary(&dictionary).map_err(|e| format!("{e:?}"))?;
                let hz = dictionary
                    .config
                    .get("CLOCK_FREQ")
                    .and_then(serde_json::Value::as_f64)
                    .filter(|v| (1e3..=1e9).contains(v))
                    .ok_or("invalid fixture clock rate")?;
                return trigger_fixture(&mut link, &codec, &clock, hz);
            }
            return Ok(());
        }
        if pipe {
            let receive_window = dictionary
                .config
                .get("RECEIVE_WINDOW")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(0) as usize;
            return crate::pipeline::run(
                link,
                identify.compressed().ok_or("incomplete identify")?,
                count,
                width,
                receive_window,
            );
        }
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
