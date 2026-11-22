# Klipper Rust host port

This is an experimental host-side port, not a replacement for Klippy yet. Existing
Python host code, MCU firmware, and printer configuration are unchanged. Do not
use this crate to control a printer.

Phase 1 implements the Klipper transport envelope in Rust without third-party
dependencies: length, destination and sequence, payload, CRC, and sync marker.
It supports ACK frames, partial input, and decoding one frame from a concatenated
buffer. Tests use byte vectors from Klippy's Python implementation.

```sh
cargo test
```

The bootstrap stage now decodes the compressed MCU dictionary and uses its
message IDs for offline clock and uptime exchanges. Serial/CAN transport, scheduling,
G-code, and the Klippy socket API for Moonraker are separate later milestones.
Toolchanger and other plugin compatibility is not claimed by this initial port.

The protocol implementation is based on Klipper's GPLv3 host code. See the
repository's [COPYING](../COPYING).

## Offline dictionary and clock replay

Using a native Linux firmware dictionary built from upstream revision
`c9559fa04ec6b11fb1a9a1610903546319fcf310`, the tests replay 105 request/reply
pairs encoded by Klippy, including the terminating empty response. The
dictionary contains 121 command formats and 26 response formats. Clock tests
use IDs from that dictionary, including negative response IDs and unsigned
values near rollover. Build timestamp/host and compiler metadata were
normalized in the published fixture; no printer or live device was involved.

The bootstrap requests 40-byte chunks and caps compressed and expanded
dictionary sizes at 256 KiB and 2 MiB, respectively. Short, oversized, or
misaligned chunks do not advance the exchange. Clock synchronization and runtime session behavior are not part of this offline API. A separate stream receiver and disposable PTY test are described
below. It does not interpret movement or heater commands.

```sh
cargo test --locked --all-targets
cargo test --locked --no-default-features  # dependency-free frame codec
python3 -c 'import sys; sys.stdout.buffer.write(b"".join(bytes.fromhex(l.split()[1]) for l in open("tests/fixtures/identify_wire.txt")))' \
  | cargo run --locked --release --example identify_replay
```

To regenerate the fixture, build the pinned upstream tree with
`CONFIG_MACH_LINUX=y` (run `make olddefconfig` and `make`), then:

```sh
python3 tests/fixtures/generate_identify.py UPSTREAM UPSTREAM/out/klipper.dict \
  tests/fixtures/clock_wire.txt > tests/fixtures/identify_wire.txt
```

## Receive stream and disposable Linux MCU

`stream::Receiver` accepts arbitrary read boundaries, yields validated frames
and ACKs, and preserves Klippy's discard-through-sync behavior. It uses no heap
and retains at most 63 incomplete bytes between calls. The upstream C
differential test compared each read's exact decoded payloads and sequences,
discard counts, pending bytes and sync state across 1,769,288 reads in 4,096
deterministic streams, including corruption and short reads.

Build the pinned upstream Linux firmware as described above. Install stock
Klippy's Python dependencies (from that revision's requirements). Then:

```sh
sh bench/check_stream.sh /path/to/pinned/klipper
cargo build --locked --release --example native_mcu
python3 bench/native_mcu.py /path/to/pinned/klipper \
  --binary target/release/examples/native_mcu --samples 1024 --rounds 8 --json /tmp/pty.json
```

The PTY runner creates its own temporary endpoint with the native firmware;
it does not take a real serial device path. Separate PTY fault tests inject invalid bytes, drop the first request entirely,
drop the first identify response while forwarding its ACK, or silence the
peer. The first three tests must finish identify and 32 read-only exchanges
with the expected counters. The silent case must exit promptly with `NoAck`. The measured runs use a direct PTY with no injected data, a fresh MCU
per run, one request in flight, and one uptime per eight requests. Stock uses
upstream Python `SerialReader` with the upstream C serial queue; the Rust
example uses the bounded frame receiver. The connection figure measures
identify from each host's connect/initialization entry point, excluding
process launch and firmware startup. Stock includes opening the endpoint and
initializing its serial queue; Rust receives an already open descriptor.

On Linux x86-64 (AMD EPYC 9V74, affinity limited to two logical CPUs, Rust
1.100.0, Python 3.12.14, GCC 14.2.0), after one warmup per host and alternating
host order over eight runs:

| Host | Identify (median) | Clock/uptime RTT (median) | RTT (p95) |
| --- | ---: | ---: | ---: |
| Rust PTY example | 1.553 ms | 7.070 µs | 10.085 µs |
| Stock Klippy serial reader | 17.311 ms | 50.196 µs | 94.924 µs |

Each host completed 8,192 measured requests. This compares a minimal,
single-in-flight Rust loop with stock's full transport path. It does not
measure feature-equivalent scheduler performance, physical links, G-code,
or printing time. The end-to-end harness has an independent process timeout. The Rust example
uses a limited `ReadQuery` state machine with a one-second per-query deadline:
unacknowledged wire frames retry their existing sequence with a sync prefix;
an acknowledged read without a reply is reissued with a new sequence. Both
retry counts are bounded. It is not stock's adaptive RTO, multi-command flow
control, or a general policy for retrying state-changing operations. For the
pre-retry benchmark, see commit `a922819c`.

## Primary and secondary MCU clock models

`clock_sync::PrimaryClock` initializes from firmware uptime, extends ordered
32-bit samples, tracks drift and minimum RTT, and estimates a conservative
release anchor separately from the host/MCU clock mapping.
`SecondaryClock` aligns another MCU to the primary print timeline without
requiring identical oscillator rates. These are pure models: the runtime
still must supply valid timestamps, periodic samples, reset/reconnect events
and an independently verified command scheduler. A counter wrap is not
distinguishable from a reboot using only a wrapped counter. Do not use these
models to drive a printer.

```sh
python3 bench/clock_sync.py /path/to/pinned/klipper --bench 300
```

The differential script runs stock `klippy/clocksync.py` from the same pinned
checkout. Across 6,000 synthetic primary samples (drift, jitter, multiple
wraps, missing send timestamps, outliers and a reinitialized epoch), release
ticks and sample disposition agreed. Across 1,493 two-MCU calibration points,
horizon tick predictions agreed. The largest primary frequency difference
observed was 2.33e-10 Hz. These are differences against the Python model, not
claims about physical oscillator accuracy.

On the same AMD EPYC 9V74 host, pinned to one logical CPU, the primary
model microbenchmark used pre-parsed sample inputs, seven measured alternating
rounds and 450,000 samples per round: 12.17 ns/sample median in Rust vs
896.20 ns/sample for stock Python with a no-I/O serial callback. This excludes
I/O, clock-query scheduling, message parsing and physical printing.

### Live disposable clock trace

```sh
cargo build --locked --release --example native_mcu
python3 bench/native_mcu.py /path/to/pinned/klipper \
  --binary target/release/examples/native_mcu --live-clock --json /tmp/clock-traces.json
```

The Linux PTY test seeds from a timed uptime reply, makes eight warmup
reads 50 ms apart, then makes two more reads at approximately 0.9839 s
intervals. For the zero-wire-delay, single-in-flight PTY case, it timestamps
the fresh write and the read that supplies the response. These are PTY-only
boundaries, not a physical wire-time model. It replays the exact captured
boundaries and tick values through stock Klippy's Python model instead of
comparing the clocks of separately timed runs.

Test cases cover a clean link, a lost request (the transport-retried reply
extends raw ticks without updating calibration), a lost response after
ACK (the fresh-sequence application retry supplies a new send timestamp),
and a disconnect with a query pending. The disconnected host exits; a
new host process and a freshly started MCU establish a new uptime epoch.
This verifies teardown and fresh startup, not automatic live recovery in a
long-running printer process. It does not send motion or heater commands.

## Offline command release planner

`planner::Planner` keeps bounded FIFO command lanes and batches ready
commands into protocol-sized payloads. It models the pinned C scheduler's
release clock, deadline priority, background priority, bounded flight and
receive-byte windows, and minimum UART / classic CAN wire-time estimates.
It does not drive a device. The separate flight ledger below handles ACK
progression and cached retransmission; the caller still owns burst writes,
session teardown, and clock-epoch validation.

```sh
python3 bench/planner.py /path/to/pinned/klipper
```

The offline comparison invokes unchanged pinned C queue functions without
starting their worker or opening a device. Across 22,504 mixed snapshots,
Rust matched the stock action and wake time. The 5,755 emitted frames also
matched byte-for-byte (including sequence, packing and CRC). Inputs covered
PTY / UART / classic CAN timing models, multiple FIFO lanes, delayed release,
background deadlines and backpressure. Empty waits are normalized to an
idle result. These cases do not establish physical bus timing or print
scheduler safety.

## Established-session send ledger

`flight::Flight` holds at most 12 outstanding blocks. Payload replies may retire
blocks cumulatively, but only an empty ACK releases the separate byte-window
charge for the last retired block. A repeated empty ACK can trigger fast
retransmission. Retransmission emits one sync byte followed by the exact
still-pending frames in original order. Received frames outside the active
sequence window are rejected; the ledger is not a startup resynchronizer.

A failed or short write permanently poisons the ledger, including a retry
write. The caller must close the old transport and discard that session's
planner and pending commands. It must not replay those commands in a new
ledger. Startup sequence adoption, adaptive retry timing and burst I/O are not implemented by the ledger. The
disposable PTY integration below composes it with the planner.

```sh
python3 bench/flight.py /path/to/pinned/klipper
```

The offline harness exercised 30,009 events against the pinned C receive and
retransmit routines, including sequence wrap, partial cumulative ACKs, byte
window pressure, rejected sequences and duplicate ACKs. The state and wire
outputs matched, including 2,089 exact retransmissions to an isolated pipe.
Rust tests separately cover send-completion notification cutoff, corruption,
and fatal write failures. This is a protocol equivalence check, not a speed
benchmark or a test on a printer.

## Pipelined read-only PTY comparison

```sh
cargo build --locked --release --example native_mcu
python3 bench/pipeline_pty.py /path/to/pinned/klipper target/release/examples/native_mcu
```

Both hosts first bootstrap on disposable native Linux MCU PTYs. The timed
section submits a wave of 128 pre-encoded identify reads and receives the
128 distinct dictionary offsets, 16 bytes per reply. Rust batches them using
the planner and flight ledger; the stock baseline uses pre-encoded raw
serialqueue sends and an asynchronous Python response callback. Both validate
contents; Rust matches the exact bytes from bootstrap, and the harness also
compares those bytes against stock. Connection setup and command encoding
are excluded. Each wave waits for completion before the next one.

Eight alternating rounds of 64 waves per host (65,536 responses each),
pinned to two logical CPUs on AMD EPYC 9V74:

| Host | Median per wave | p95 per wave |
| --- | ---: | ---: |
| Rust example | 127.026 µs | 222.685 µs |
| Stock Klippy (Python + C) | 816.561 µs | 1699.083 µs |

Rust reached nine simultaneous outstanding frames. Dropping one request
frame caused one cached retransmission and recovered all expected replies.
Dropping only a reply after the MCU received the request hit the wave
deadline with exactly one missing reply and zero outstanding frames; the
process discarded the session rather than blindly replaying commands.

This measures a matched, pre-encoded read-only workload over PTY, not an
individual request RTT, print time, CPU load, or full Klippy feature parity.
The example uses a fixed retry timer and does not exercise physical I/O.

## Offline motion look-ahead

`lookahead` implements stock XYZ plus one optional E axis geometry, move and
junction speed limits, minimum cruise ratio, and full or lazy flush. It
returns velocity and timing profiles only. Homing, travel limits, extrusion
temperature and kinematics validation are outside this module; do not drive
a device from this API.

```sh
taskset -c <CPU> python3 bench/lookahead.py /path/to/pinned/klipper
```

The pinned Python 3.12 baseline uses `Move`, `LookAheadQueue`, and the stock
extruder junction method directly; no device or reactor is started.
25,836 moves and 6,366 flushes agreed in queue decisions. Profile fields
matched within the harness's 2e-12 tolerance (largest observed absolute
difference 3.47e-18); cases include reversals, diagonal paths, extrude-only
moves, externally limited motion, zero junction caps, and lazy flush tails.
The Rust XYZ norm follows Python 3.12's compensated sum, which matters near
degenerate corners.

Seven alternating single-CPU rounds had medians of 110.4 ns/move Rust and
3997.3 ns/move stock Python on AMD EPYC 9V74; fixture parsing is untimed.
This measures host geometry and look-ahead only, not motor steps or
print duration.

## Offline G-code motion preview

The preview accepts a strict subset of ordinary G-code: G0/G1 (XYZEF),
G21, G90/G91, G92, M82/M83, M220/M221, G4 and M400. It handles the
coordinate and feed/flow state, no-op moves, lazy flush and full-stop
boundaries. It deliberately rejects macros, unknown words, duplicate
parameters, line numbers/checksums, and inches. There is no printer
configuration, homing, step generation, heater/fan control, or device I/O.

```sh
taskset -c <CPU> python3 bench/gcode_preview.py /path/to/pinned/klipper
```

7,020 input lines and the final flush matched the pinned stock dispatcher,
coordinate handlers and look-ahead using an in-memory test toolhead.
5,370 move profiles matched to within 2e-12 absolute/relative tolerance
(maximum absolute difference 1.39e-17 on this run). Python equipment and
kinematics checks were stubbed; this does not establish machine safety.

On the same single logical CPU, seven alternating rounds had median host
processing cost of 0.329 µs/line Rust and 6.745 µs/line Python. Inputs
were already in memory, with G-code parsing, coordinate handling, and
look-ahead timed; startup, file I/O, step generation and hardware are
excluded. The compared algorithm produces the same planned timing, not
faster physical motion.

## Offline XYZ trajectory

`trajectory::Trajectory` turns look-ahead profiles into bounded XYZ
acceleration, cruise, deceleration and hold segments. It supports querying
coordinates at arbitrary times, including stationary gaps; the caller owns
the timeline for dwells and extrusion-only moves. Position resets, history
pruning, kinematic transforms, and motor step generation are not included.

```sh
python3 bench/trajectory.py /path/to/pinned/klipper
```

Against the pinned C `trapq_append` / `move_get_coord`, all 5,356 segments
and 14,953 coordinate samples matched exactly in the offline fixture. Inputs
included zero phases, short phases, idle gaps and samples at boundaries.
A Rust unit test also builds a timeline from in-memory G-code profiles.
No performance or hardware claim is made from this comparison.

## Frame validation microbenchmark

On Linux (AMD EPYC 9V74) with Python 3.12.14, Rust 1.100.0 and GCC 14.2.0, pinned to one
logical CPU, the median of seven measured runs was:

| Implementation | ns / valid frame | Min–max (ns) |
| --- | ---: | ---: |
| Python `MessageParser.check_packet` | 3638.97 | 3610.86–3899.33 |
| Klippy C `msgblock_check` | 37.20 | 36.74–41.33 |
| Rust `decode_prefix` | 44.48 | 43.69–52.74 |

Rust used about 1/82 of the Python reference's time, but took about 20% more
time per frame than the C helper that Klippy actually uses in its serial
receive path. This is not a speedup over production Klippy. The comparison
uses a repeated mix of valid 5-, 8-, 11- and 64-byte frames; it excludes
device I/O, C resynchronization on invalid data, host scheduling, and motion.

From this directory, run `python3 bench/run.py` with Python 3, a Rust
compiler, and a Linux C toolchain. For the recorded upstream baseline, use
checkout `c9559fa04ec6b11fb1a9a1610903546319fcf310` as the optional argument.

### Optimized CRC comparison

The production C receive check and both Rust versions were rerun on the same
pinned x86-64 logical CPU. The median of nine rounds per profile (lower is
better):

| Valid-frame mix | Klippy C | First Rust version | Optimized Rust |
| --- | ---: | ---: | ---: |
| Short (5, 6, 8, 11 bytes) | 8.43 ns | 11.16 ns | 7.61 ns |
| Mixed (5, 8, 11, 64 bytes) | 37.69 ns | 45.18 ns | 19.12 ns |
| Long (32, 48, 63, 64 bytes) | 104.73 ns | 113.99 ns | 43.82 ns |

The new CRC uses four compile-time tables (2 KiB) to consume four bytes at a
time, then consumes up to three trailing bytes using the first table. It
produces identical wire results for all 60 payload lengths and 16 sequence
numbers across eight deterministic payload draws (7,680 frame cases), as
well as the upstream vectors. The mixed median is 58% lower than the first
Rust version and 49% lower than Klippy C. The short-frame results overlap in
some individual runs. This remains a synthetic test, not an estimate of
performance on a Raspberry Pi or a full printer.

To rerun the three-way comparison, from `rust-host/`:

```sh
git show 3cebfed61eea618d676283d2c7d577ffd3a942d3:rust-host/src/lib.rs > /tmp/klipper-rust-baseline.rs
python3 bench/run.py --baseline /tmp/klipper-rust-baseline.rs
```

Use `taskset -c <CPU>` to pin the benchmark. The current script reports
the two native implementations and can optionally include a prior Rust source;
the original Python measurement is retained above as historical context.

After adding the offline bootstrap and clock codecs, a validation rerun
measured the mixed profile at 18.02 ns/frame for the optimized Rust checker,
36.45 for C and 43.32 for the original Rust implementation. The frame checker
itself was not changed, and this rerun does not benchmark dictionary loading.
