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
boundaries. The preview also accepts coordinate-only `SET_GCODE_OFFSET`
and named `SAVE_GCODE_STATE` / `RESTORE_GCODE_STATE`. Saving and restoring
state does not execute arbitrary macros. The preview rejects unknown words, duplicate
parameters, line numbers/checksums, and inches. There is no printer
configuration, homing, step generation, heater/fan control, or device I/O.

```sh
taskset -c <CPU> python3 bench/gcode_preview.py /path/to/pinned/klipper
```

7,065 input lines and the final flush matched the pinned stock dispatcher,
coordinate handlers and look-ahead using an in-memory test toolhead.
5,096 move profiles, requested/gcode coordinates, and homing offsets
matched exactly in this run. The corpus includes offset adjustment,
quoted case-sensitive state names, restore with and without movement,
and preserving E's relative coordinate after a restore. Python equipment
and kinematic safety checks were stubbed.

On the same single logical CPU, seven alternating rounds had median host
processing cost of 0.395 µs/line Rust and 8.274 µs/line Python. Inputs
were already in memory, with parsing, coordinate handling, and motion
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

## Offline raw step crossings

`step_times` applies Cartesian and CoreXY motor transforms and the stock
iterative crossing algorithm to bounded portions of trajectory segments.
It emits relative step times, direction, and commit markers; this is before
tick conversion, short-reversal cancellation, step compression, and MCU I/O.

```sh
python3 bench/step_times.py /path/to/pinned/klipper
```

The unchanged C solver (with an in-memory sink before compression) and Rust
matched all 34,805 crossings, 64,048 commit markers, and commanded positions
at 5,000 segment boundaries. Cases include direction changes, inactive
axes, and separately flushed segment portions. This does not validate
actual step pulses or compressed firmware commands.

## Offline step command trace

`step_clock::encode_trace` converts raw crossings to clock ticks, applies
stock's sub-750 µs uncommitted reversal rollback, and batches same-direction
runs through `step_compress`. Near-reversal commit markers can preserve a
step. The trace yields direction changes and step block fields only; it
does not emit frames, schedule I/O, handle far-future steps or expose a
production streaming interface.

```sh
python3 bench/step_compress.py /path/to/pinned/klipper
python3 bench/step_clock.py /path/to/pinned/klipper
```

The compression benchmark fixture matched 31,482 blocks exactly across
1,801 runs / 278,249 clock inputs, including 32-bit rollover and a
65,535-step constant-rate run. This fixture checks correctness only.
Through the actual stock C step-clock/filter/compressor and decoded queue
messages, the second fixture matched all 6,593 direction commands and
51,878 step blocks across 641 traces, including direction inversion,
half-tick rounding, and rollbacks at the reversal boundary. Nothing
was sent to a device.

## Offline configuration resolution

`config::parse` reads only a caller-supplied map of relative file names and
UTF-8 text. It applies nested includes, sorted nonrecursive glob matches,
later value overrides, multiline values, and default options. It rejects
root escapes and active include cycles. This layer does not read the
filesystem, load plugins, execute macros, or apply/write SAVE_CONFIG data.

```sh
python3 bench/config.py /path/to/pinned/klipper
```

The pinned `ConfigFileReader` comparison covered 228 upstream sample
configs and one nested synthetic config: 2,696 section entries in their
original order and 16,075 effective options matched byte-for-byte. Unit
tests additionally cover absent literals, empty wildcard matches, escaped
paths and cycles. This is read-only parsing parity for the fixture, not
validation that all configured hardware/modules are supported.

## Config-to-step offline preview

`machine_preview::Machine` combines supported printer and rail settings with
G-code coordinates, travel checks, Z velocity/acceleration limits, XYZ
trajectory, and clocked step traces. It accepts one motor per Cartesian or
CoreXY rail and begins from a caller-supplied virtual position. An optional
offline extruder lane requires explicit geometry, step distance and
simulated temperature, with optional pressure advance. No physical heater
is used. It does not implement real temperature verification, homing, bed
transforms, tool changes or I/O; the result is not a live controller.

```sh
python3 bench/machine_trace.py /path/to/pinned/klipper
```

The same config and G-code, including dwell, feed override, reversal,
coordinate reset and Z movement, were processed through stock Python config,
G-code and kinematics and pinned C trapq/itersolve/stepcompress. All 337
Cartesian and 378 CoreXY direction/step commands (including cumulative
ticks) matched exactly. Both planned the same 0.728039557 simulated seconds.
The optional extruder fixture matches 278 Cartesian and 320 CoreXY
commands across all four lanes, including E-only moves and retracts, XY+E
moves, flow overrides, and Z limits. Both versions planned 0.824010918
simulated seconds on that fixture. Stock's real extrusion geometry,
nominal trapq, compression and step routines are used with a simulated
temperature gate. This does not exercise pressure advance.

These are deterministic traces in memory with no peripherals or hardware.

## Offline read-only Klippy requests

The `webhooks` feature implements bounded JSON/ETX stream reassembly and
the read-only `info`, `gcode/help`, `objects/list`, `objects/query` and
`objects/subscribe` request subset. The caller supplies an in-memory
status snapshot, event timestamp, and update cadence. Subscriptions
report changed fields. Malformed and oversized input is dropped without
losing subsequent valid frames. No socket or command execution is
opened; this is not yet a Moonraker endpoint.

```sh
cargo build --locked --release --example webhook_replay
python3 bench/webhooks.py /path/to/pinned/klipper target/release/examples/webhook_replay
```

22 requests generated 17 responses identical to isolated stock
Klippy handlers (with only offline endpoints exposed and portable
`info` fields compared). Unit tests also exercise subscription changes
and stream recovery. No production status providers are loaded.

### Run the preview CLI

From `rust-host/`:

```sh
cargo run --locked --release --bin klipper-preview -- \
  examples/preview printer.cfg examples/preview/path.gcode 20000000 1 1 0.2 rails.cfg
```

The inputs are the configuration root, entry path, G-code path, clock Hz,
explicit virtual X/Y/Z start, and any included config files. Include
only the files the configuration needs; paths are resolved under the
specified root. The synthetic fixture sends nothing to a printer. The
JSON shows duration and step/direction blocks per motor. The CLI uses
zero compression error and logical motor directions, without physical pin
inversion. Errors do not yield a partial trace.

## Offline tool-coordinate translation lifecycle

The G-code preview can wrap a prior translation with another layer, update
or detach the top layer by its owner, and reset the reported position from
commanded physical coordinates without emitting a move. XYZ transforms
leave E unchanged. This models the mechanics needed for a tool-offset
handover only; no Python extension, tool pickup sequence, nonlinear
transform, or probe is implemented.

```sh
python3 bench/tool_transform.py /path/to/pinned/klipper
```

A synthetic two-layer tool-change fixture matched the pinned Python
`GCodeMove.set_move_transform` / `reset_last_position` and coordinate
commands: 31 state snapshots and six resulting move profiles. The
fixture changes offsets while preserving toolhead position, restores
logical coordinates, and returns selected axes. It does not run the
Toolchanger plugin.

## Real Toolchanger stock baseline (no Rust host)

The Klippain smoke runner uses stock Klippy debug output, a matching Linux
MCU dictionary, and the real Toolchanger / LED Effect plugins. Tested
Klippain `77ea66b1`, Toolchanger `94756dfd`, and LED Effect `266f1049`
without changing their code or connecting a printer:

| Stock Klipper | StealthChanger | TapChanger | Shared LED chain |
| --- | --- | --- | --- |
| Earlier working pin `644cda5e` | Pass | Pass | Pass |
| Rust reference pin `c9559fa0` | Probe constructor incompatibility | Same | Same |

At the Rust reference pin, stock's `ProbeEndstopWrapper` requires
`probe_offsets` and `param_helper`; the pinned Toolchanger still calls the
one-argument constructor. The cases fail during initialization in Python,
before the smoke assertions. This is a baseline mismatch, not proof of a
Rust regression, and not a hardware or Toolchanger compatibility claim.

`python3 bench/toolchanger_baseline.py --help` lists the disposable stock and
plugin checkouts plus their matching dictionaries. Prepare each stock tree
with the four pinned Toolchanger modules and pinned LED Effect module in
`klippy/extras` (copied or symlinked), and supply the usual Klippy Python
requirements. The runner checks pins/module bytes and isolates debug
artifacts. Use `--results` with a new empty directory to retain stock logs.

## Offline thermal model

`thermal` ports stock PID and watermark controller mathematics and
temperature smoothing. It computes a simulated duty request only. A
disabled target or stale host signal produces zero modeled duty; invalid
or out-of-range sensor samples and backwards clocks latch an error.
This is not a heater driver and does not implement sensor conversion,
heater verification, PWM scheduling, MCU watchdogs, or emergency stop.

```sh
python3 bench/thermal.py /path/to/pinned/klipper
```

22,000 deterministic synthetic samples matched the stock PID and
watermark model with a dummy output sink, including target changes,
saturation, varying sample intervals and a stale-host gate. Rust unit
tests independently check the invalid input and latch behavior. No
heater or hardware was used.

## Offline motion wire encoding

`step_wire` uses the identified dictionary's actual `set_next_step_dir`
and `queue_step` message IDs and produces payload/release metadata for
already-validated per-stepper traces. It checks interval/count/add bounds,
direction ordering, and the cumulative clock. This does not expose a
motion transport or configure driver OIDs.

```sh
cargo build --locked --release --example step_wire_replay
python3 bench/step_wire.py /path/to/pinned/klipper /path/to/klipper.dict \
  target/release/examples/step_wire_replay
```

11,204 payloads and single-command test frames matched the pinned native
Linux MCU dictionary and the stock Python encoder byte-for-byte, including
sequence wraps, signed add values and multi-byte intervals. The independent
offline planner comparison covers batching; this test does not send any
byte to either a virtual or physical MCU.

## Compression CPU improvement

The compressor now avoids integer division when the current interval
bounds already satisfy the next point. The offline step encoder also
consumes blocks directly rather than allocating a temporary vector for
each direction run. All 31,482 blocks still match the C reference.

| Implementation | CPU ns/requested step, median |
| --- | ---: |
| Stock C with output validation | 11.845 |
| Prior Rust allocating intermediate blocks | 14.945 |
| Updated Rust emitting verified blocks | 13.758 |

Nine alternating single-core rounds of 70 passes over 278,249 requested
steps per pass on AMD EPYC 9V74. Timed work excludes parsing, file output,
I/O, motion generation and setup. This is ~7.9% faster than prior Rust
but still ~16% slower than the stock C helper on this corpus.

```sh
git show 3cd0582cebcebdf39f8d20031a18f6e1753a3dfd:rust-host/src/step_compress.rs > /tmp/prior-step-compress.rs
taskset -c <CPU> python3 bench/step_compress.py /path/to/pinned/klipper \
  --rounds 9 --repeat 70 --baseline-source /tmp/prior-step-compress.rs
```

## Offline pressure advance sampling

`pressure_advance::Advance` samples a signed nominal E trajectory with
stock's triangular smoothing, per-phase pressure eligibility, and an
ordered gain-change timeline. Retraction and E-only travel are not
pressure-advanced. It also supplies sample positions to the offline E step solver. The solver
includes lead/trailing smoothing windows around nominal E motion. It
does not change a printer's settings or output real steps.

```sh
python3 bench/pressure_advance.py /path/to/pinned/klipper
```

17,832 queries matched the real stock C extruder position callback,
including pressure gain changes, idle gaps, retracts, enabled and
disabled phases, and sampling around phase boundaries. The maximum
absolute difference was below 1e-12 mm. With a timed gain change,
the full `bench/machine_trace.py` comparison also matched 381 Cartesian
and 423 CoreXY direction/step records across four lanes, including the
E lane. No physical material flow was measured.

## Offline Classic CAN packetization

`can` splits a bounded Klipper write into up-to-eight-byte data fields
and reassembles response streams independently for assigned node IDs.
It rejects non-standard IDs, extended/RTR flags and invalid lengths.
No hardware discovery, admin commands, SocketCAN access or interface
changes occur. This does not simulate physical arbitration.

```sh
python3 bench/can.py /path/to/pinned/klipper
```

Against the pinned C writer using only an isolated local datagram
socketpair and C receive-frame validation, all 12,106 fragments and
1,440 extracted frames matched. The six synthetic node streams were
interleaved with corruption and 1,241 unrelated-ID frames; per-read
recovery/discard counts also matched. No physical devices are implied
by the six-node fixture.

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
