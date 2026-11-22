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
These rows are the recorded fixed-timer baseline. The same example now
uses the stock-matched adaptive timer with a separate one-second test
deadline. An adaptive rerun over the same wave counts measured 127.277
µs median / 241.925 µs p95 for Rust and 970.198 µs median / 1668.456 µs
p95 for stock. Clean waves had no retries; the dropped-frame and
dropped-reply tests still passed. Between-run differences are not a
speedup claim for the timer. No physical I/O was exercised.

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

## Offline tool selection lifecycle

`toolchange::Changer` models a bounded choose/park/pickup sequence.
It emits descriptive actions for coordinate save/restore, dock transform,
tool changes, pickup detection, and missing-tool watch activation. It
requires the configured axes to be marked homed and uses per-step tickets
to reject late confirmations. After a failed pickup, it does not expose an
active identity until explicit simulated detection succeeds. It does not
execute docking G-code, Jinja, fan/temperature commands or a real plugin.

On viesturz/klipper-toolchanger at
`94756dfde9b729fd69f9b8780067821c5c99a528`, with mocked callbacks and the
legacy compatible Klippy checkout, 64 selection/no-op cases and 505
ordered observations matched the stock plugin selection method.

```sh
cargo build --locked --release --example toolchange_replay
python3 bench/toolchange_order.py /path/to/legacy/klipper /path/to/toolchanger
```

## Offline heater fault gate

The stock heater progress/deficit check is modeled separately from PID and
watermark calculations. The comparator matched 68,906 synthetic readings
across 320 heater profiles, including all 119 stock fault decisions. Stale
or invalid samples are a distinct sticky virtual fault. In the integrated
demonstrator a virtual fault clears pending motion; no actual sensor,
heater, shutdown procedure or firmware watchdog is exercised.

```sh
cargo build --locked --release --example verify_heater_replay
python3 bench/verify_heater.py /path/to/pinned/klipper
```

## Integrated software-only run

`simulated_printer::Simulation` brings together virtual homing evidence,
stock-compatible SD line boundaries, optional limited macros, the offline
step pipeline and a virtual job. It presents read-only progress and print
statistics while moving through standby, printing, paused, complete or
faulted states. The reported input byte boundary is preflight metadata,
not a live virtual SD read position. The demo command below is XYZ-only;
the simulation library can also hold one explicitly inert extruder lane.
Neither accepts physical device or control-client paths.

```sh
cargo run --locked --release --example simulated_job -- /path/to/klipper.dict
```

The pinned 50 MHz Linux MCU dictionary produced 138 inert wire commands
across three lanes. Decoding and replaying those commands in the virtual
motors produced the expected CoreXY displacement of `[720, -120, 112]`
steps. A separate independent recurrence expands the full edge sequence:
43,344 timestamp and position observations across 2,721,525 edges match,
including timing boundaries and 32-bit clock wrap. The virtual stepper can
drop all pending edges on a latched stop. No electrical or motor timing is
measured by these tests.

The opt-in native motion fixture archives pinned stock Linux firmware into
a temporary tree, replaces GPIO with memory stubs, and uses an explicit
virtual board clock. Stock C scheduler and stepper logic are preserved.
The Rust test refuses the normal dictionary before sending configuration
or motion commands. With the marked fixture, it queues real firmware
commands on a fresh PTY and compares C and Rust step counters at 17 virtual
clock checkpoints. The XYZ macro path runs 138 commands with end counts
`[720, -120, 112]`. A second path with input shaping, pressure advance,
retraction and extrusion runs 277 commands with end counts
`[720, -120, 112, 100]`. Cold extrusion is rejected during virtual
preflight. No physical pins, pulses, motors or wall-clock timing are tested
in this deterministic fixture.

```sh
cargo build --locked --release --example native_mcu
python3 bench/native_gpio_motion.py /path/to/pinned/klipper
# Or add --native-gpio to bench/check_parity.py --native-pty.
```

Integration tests verify that a simulated fault drops
remaining work and that separate Unix stream pairs carry read-only JSON
subscriptions without accepting G-code or letting one slow client exhaust
the reply queue. The named Unix listener is optional and its bind/cleanup
test requires a host that allows local socket binding:

```sh
cargo test --locked --test simulated_printer listener_binds -- --ignored
```

## Offline print statistics

`print_stats::Stats` mirrors stock duration and filament accounting using
explicit virtual clocks and extrusion observations. It models pauses,
extruder changes, layer metadata and terminal status, rejecting
non-finite readings and backwards observations. The resulting status can
be fed into the read-only WebHooks model. It does not sense filament,
manage hardware, or treat simulation time as elapsed physical print time.

```sh
cargo build --locked --release --examples
python3 bench/print_stats.py /path/to/pinned/klipper
```

The comparator matched 12,480 snapshots from stock Klippy across 260
simulated jobs, including flow changes, no extrusion, tool changes,
layer updates, completions, cancellations and faults.

## Virtual SD checkpoints

The offline `virtual_sd::Job` issues immutable LF-delimited lines, tracks
UTF-8 byte checkpoints, supports controlled boundary rewind/pause, and
does not advance a failed or uncommitted line. An unterminated final
line is not dispatched, matching pinned stock virtual SD. Interior seeks,
invalid UTF-8 in a complete line, and overlong lines are rejected rather
than dispatched. This does not open a live SD service or execute an
on-error macro.

```sh
python3 bench/virtual_sd.py /path/to/pinned/klipper
cargo run --locked --release --bin klipper-preview -- \
  --virtual-sd --expand-macros examples/preview printer.cfg \
  examples/preview/path_macro_tail.gcode 20000000 1 1 .2 rails.cfg
```

The in-memory comparison against stock `VirtualSD.work_handler` matched
19,761 line/checkpoint, pause, and terminal observations across 64
fixtures, including UTF-8, CRLF, rewind, pause/resume and dispatch failure.
The CLI option only exposes an inert byte checkpoint and motion preview.

## Opt-in offline macro expansion

The preview CLI can expand bounded static `[gcode_macro NAME]` bodies,
nested calls, and finite numeric `{params.NAME}` substitutions with optional
numeric `|default(n)`. Unsupported definitions may be present but are
rejected if called. The expander does not execute Jinja, access printer
objects, register live commands, emulate template actions, rename commands,
or support macro variables and conditionals. Expansion is off by default.

```sh
cargo run --locked --release --bin klipper-preview -- --expand-macros \
  examples/preview printer.cfg examples/preview/path_macro.gcode \
  20000000 1 1 .2 rails.cfg
python3 bench/gcode_macros.py /path/to/pinned/klipper
```

The optional Python comparator requires stock Klippy's Jinja2 dependency.
Under the supported subset, all 2,700 dispatched commands from 600 seeded
calls matched stock Klippy/Jinja2, including nested and parameterized
calls. The CLI integration fixture produced byte-identical step traces
with and without macro expansion. The tests check rejection of cycles,
unsupported templates and missing arguments; this is not broad Toolchanger
macro compatibility.

## Offline homing evidence

`homing` projects simulated trigger and halt counters to Cartesian or
CoreXY positions, distinguishes overshoot from contact, estimates the
per-endstop sampling interval, and computes a second-approach position.
`Machine::trace_homed` optionally requires all three axes to home in the
fixture before a preview can enter the virtual job/transport pipeline.
Incomplete, late, or failed evidence cannot mark an axis homed. CoreXY XY
evidence requires both linked motors. There is no G28 command, GPIO,
firmware trigger scheduling, sensor read, or live interlock here.

```sh
python3 bench/homing.py /path/to/pinned/klipper
```

With mock positions and counters, stock Python matched 1,000 interval
calculations, 1,000 position/status cases and 922 retracts. The older
Toolchanger compatibility pin also matched interval and position results;
its retract math is inline, so that harness does not call it separately.
The virtual job integration test refuses a move before homing, then sends
the validated preview through local virtual CAN peers without hardware.

## Pin-free MCU startup fixture

The Rust startup gate encodes a bounded trigger-synchronizer-only profile,
checks the firmware's reported configuration/CRC, reserves requested move
slots, and refuses both shutdown states and CRC mismatches. It does not
reset or recover firmware automatically. No arbitrary config strings, pin
definitions, stepper activation, or heater commands are supported.

`bench/mcu_config.py` compares 256 CRCs and 33,152 payloads with pinned
Klippy. On fresh, disposable Linux MCU PTYs, four pin-free profiles matched
stock configuration and move capacity (1,021 usable after reserving three).
An identical profile reuses the configuration without writes; a differing
CRC is refused. Dropping the first config write ACK causes the fixture to
stop without retransmitting or finalizing the unknown state.

The pin-free trigger-sync fixture also matches 3,600 stock single- and
multi-MCU report/deadline vectors. On the software MCU, the Rust and stock
paths agree on host-request and watchdog-expiry reasons. A stop query
preserves the terminal expiry reason. These tests do not attach an endstop,
drive any pin, or validate physical multi-MCU synchronization.

The offline multi-peer dispatcher also matches pinned C across 14,400
reports with wrapped clocks and differing clock rates: 3,835 timeout
extensions and 1,188 propagated terminal sends. The first terminal result
stops extension and propagates once; unrelated OIDs cannot change state.
The comparator links real stock `trdispatch.c` against an in-memory
serial queue, not an actual bus.

```sh
cargo build --locked --release --examples
python3 bench/mcu_config.py /path/to/pinned/klipper
python3 bench/trigger_sync.py /path/to/pinned/klipper
python3 bench/trigger_dispatch.py /path/to/pinned/klipper
python3 bench/trigger_sync.py /path/to/pinned/klipper --native \
  --binary target/release/examples/native_mcu
```

## Fixed offline input shaping

The motion preview supports fixed X/Y/Z convolution from the stock
`[input_shaper]` settings (plain ZV, ZVD, default MZV, EI, 2-hump EI and
3-hump EI). The sampler uses both before/after phase windows, including
lead-in and tail steps. Parameterized MZV, runtime `SET_INPUT_SHAPER`,
accelerometer calibration, and hardware activation are not supported.

The config-to-step comparator checks custom pulses and all six named
shapers on both Cartesian and CoreXY. A separate 21-round run with
CoreXY X=MZV, Y=EI and extruder pressure advance matched 6,835 directions
and step commands including clocks. Simulated motion stayed at
11.342102983 seconds on both paths, while CPU time for the timed offline
section was 7.777 ms stock versus 5.481 ms Rust (30% lower). This uses
the same timing boundaries as the unshaped experiment below.

The convolution now evaluates only the requested axis and bypasses the
shaper for motor lanes whose shaping is disabled. In a 30-round
rotating-order comparison with the previous Rust source (commit
`c48674d70343d6ebf537652b572441d876c158d0`), the same 6,835 records and
simulated duration matched each round. Median process CPU time was
7.742 ms stock, 5.332 ms previous Rust, and 4.842 ms optimized Rust
(9% less than previous Rust, 37% less than stock).

```sh
taskset -c 2 python3 bench/machine_trace.py /path/to/pinned/klipper \
  --rounds 30 --shape-timing --baseline-rust /path/to/baseline/rust-host
```

## Config-to-step offline preview

`machine_preview::Machine` combines supported printer and rail settings with
G-code coordinates, travel checks, Z velocity/acceleration limits, XYZ
trajectory, and clocked step traces. It accepts one motor per Cartesian or
CoreXY rail and begins from a caller-supplied virtual position. An optional
offline extruder lane requires explicit geometry, step distance and
simulated temperature, with optional pressure advance. No physical heater
is used. It does not implement real temperature verification, powered homing,
bed transforms, tool changes or I/O; the result is not a live controller.

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

The regression suite also repeats the CoreXY fixture 12 times in one
timeline with pressure advance, extrusion/flow resets, and return moves.
All 5,796 direction and step commands still match, including cumulative
ticks; modeled motion time is unchanged at 11.342102983 seconds.
The repeated path exercises nonzero trapezoid phases shorter than one
representable tick of the host floating-point timeline.

```sh
taskset -c 2 python3 bench/machine_trace.py /path/to/pinned/klipper --rounds 21
```

On one AMD EPYC 9V74 core (Python 3.12.14, GCC 14.2, Rust 1.100.0),
21 interleaved measurements after one warmup took a median 4.712 ms of
combined stock Python and C process CPU time and 3.662 ms in Rust (22%
less). The timed sections cover config parsing through motion planning,
step generation and compression. They omit input/output, process startup,
and the fixture's cross-process text adapter. The C section also includes
internal message staging; Rust returns typed commands. This is one offline
host-side scenario, not full Klippy integration or elapsed printing speed.

The Rust step solver can now append into one buffer per motor instead of
allocating a temporary buffer for each motion phase. A separate 30-round
run, rotating the order with the previous Rust source (commit
`49da0dc898237bda872bdee9b54b601c7ba0204a`), measured 4.824 ms stock,
3.735 ms previous Rust, and 3.209 ms with reuse (14% less CPU than
previous Rust, 33% less than stock). All three produced the same 5,796
commands and modeled duration in every round. The two Rust libraries use
the same benchmark driver. To reproduce, add
`--rounds 30 --baseline-rust /path/to/baseline/rust-host` to
`bench/machine_trace.py`.

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

## Offline adaptive retry timing

`retry_timing` models the stock transport's RTT sample tracking,
variance smoothing, timeout floor/cap, timeout doubling, fast retry,
and Karn-style suppression after cached retransmits. `timed_flight`
composes that estimator with the flight ledger and lengths on the chosen
wire. The caller supplies the planned wire-start and receive timestamps.
This composition now drives the disposable read-only PTY pipeline.

```sh
python3 bench/retry_timing.py /path/to/pinned/klipper
```

18,798 ACK/timeout/fast-retry timer states matched the C reference exactly
across PTY, UART-style and Classic CAN wire-time formulas. The C harness
writes only to an isolated UNIX socketpair; no device timing was measured.

## Shared-bus CAN simulation

`virtual_can::Bus` composes Classic CAN packetization, per-node queues,
sequence ledgers, and adaptive retries over a deterministic shared host
timeline. Each node has independent release/ACK state. A silent peer is
isolated after bounded retries. The simulation only trades in-memory
datagrams. Host sends use a minimum-bit wire estimate; inbound datagrams
are timestamped by the fixture. This is not physical arbitration, bus
utilization, or measured device performance.

```sh
cargo test --locked --test virtual_can
```

One fixture interleaves six independent streams and drops a packet burst;
only its node is retried, with byte-identical data after the sync marker.
A second drives a healthy node through sequence wraps while a silent
node times out, then verifies that the healthy node can still send.

## Reproduce the correctness matrix

With a disposable stock checkout at the documented Klipper pin, Python
3.12 with the normal Klippy dependencies, Rust and a Linux C toolchain:

```sh
python3 bench/check_parity.py --upstream /path/to/pinned/klipper
# With a native Linux MCU build in that checkout (read-only PTY tests):
python3 bench/check_parity.py --upstream /path/to/pinned/klipper --native-pty
```

The runner checks the pin and reports each independent test matrix;
it does not install anything or start a real printer service. The full
local run passed 25/25 phases, including debug/release/no-default unit
tests and disposable native MCU PTY fault cases. The Toolchanger legacy
probe baseline remains a separate multi-checkout test.

## Virtual job lifecycle

`virtual_job::Job` preflights a simulated trace, releases inert command
records by virtual tick, and supports pause/resume, cancel, and a sticky
fault. It does not expose those operations over WebHooks. Its optional
read-only status object is explicitly named `virtual_job`, not a claim
of compatibility with physical Klippy printing.

```sh
cargo test --locked --test virtual_job
```

A config-to-motion fixture sends the prepared dictionary payloads through
three in-memory CAN nodes and verifies each node receives exactly its
original command bytes in order. Separate tests enforce pause/resume
without replay, fault isolation, and read-only status differences. No
device path, motor, external service or physical bus is opened.

## Opt-in Python plugin compatibility bridge

The optional `python-bridge` feature exposes the Rust junction solver to
stock Klippy's Python `LookAheadQueue.flush`. Python keeps the actual move,
kinematics and plugin objects and calls its original timing setters.
This is a debug-only experiment; the normal host is unchanged and it
must not be used on a printer.

```sh
cargo build --locked --release --features python-bridge
python3 bench/python_lookahead.py /path/to/pinned/klipper target/release/libklipper_host_protocol.so
python3 bench/hybrid_klippy.py /path/to/pinned/klipper /path/to/klipper.dict \
  target/release/libklipper_host_protocol.so
```

25,836 Python move profiles and flush decisions matched exactly with
and without the bridge. Running real stock Klippy in file debug mode
planned 1,200 moves in Rust and produced 4,299 identical decoded motor
commands by object ID; raw wire batching may differ. The three earlier
Toolchanger/LED Effect smoke cases passed with the bridge loaded, and
an extra virtual motion fixture after the actual legacy StealthChanger
plugin setup matched 416 decoded motor commands with Rust active. The
known probe incompatibility at the newer stock pin was unchanged.

The bridge itself is **not faster on the mixed workload**: seven
single-core rounds gave 4.063 µs/move stock Python and 4.234 µs with
the bridge; large queues were effectively tied (4.296 vs 4.284 µs).
Python attribute packing, the FFI boundary, and stock move object
updates remain in the timed path. This result differs from the pure
Rust look-ahead benchmark and does not justify running the bridge by
default.

## Offline arc interpolation

`gcode::Preview` can enable the optional arc extension; the machine preview
also reads an optional `[gcode_arcs]` / `resolution` section. It supports
G2/G3 with plane-specific I/J/K offsets, G17/G18/G19, helical motion,
full circles and proportional E. Relative XYZ and R-format arcs are
rejected as in stock. A request may generate at most 4,096 linear moves.

```sh
python3 bench/gcode_arcs.py /path/to/pinned/klipper
```

A synthetic fixture matched the stock `ArcSupport` G-code handler and
subsequent stock motion look-ahead: 864 state/flush snapshots and 6,761
move profiles agreed exactly. The fixture includes all three planes,
absolute and relative E, and short arcs. Nothing is sent to hardware.

## Offline bed mesh arithmetic

`bed_mesh` builds a bounded rectangular interpolated mesh using direct,
Lagrange, or bicubic sampling. It supports bilinear queries, XY
offsets, Z fade and its inverse, and the stock move splitter. It only
uses measurements supplied in memory; no probing, calibration state,
or printer configuration is changed.

```sh
python3 bench/bed_mesh.py /path/to/pinned/klipper
```

Against stock `ZMesh` and `MoveSplitter`, 3,528 point queries,
3,192 inverse positions, and 44,314 split positions across 3,192
moves matched exactly. The fixture varies grid density, all three
interpolation modes, mesh offsets and fade state.

The preview also composes the mesh as a lower transform beneath
tool offsets. A synthetic 9×9 bicubic grid with 25–325 mm bounds and
0.6–10 mm fade matched stock at 24 coordinate/flush snapshots and
57 physical move profiles, including a helical-height transition,
E interpolation, two nonzero tool translations and coordinate rebasing.
See `bench/tool_transform.py`. Heights are generated, not measured
printer data, and installing/clearing a preview mesh never moves hardware.

## Offline probe sampling

`probe` projects trigger coordinates into bed and tool positions and
applies stock mean/median, tolerance, and whole-set retries. The
simulated precondition rejects an unhomed session; invalid sensor
values and timeouts end it without a result. No pin, trigger
transport, deployment script or real homing is involved.

```sh
python3 bench/probe.py /path/to/pinned/klipper --legacy /path/to/legacy/klipper
```

700 synthetic outcomes matched both actual stock Python helpers at
the legacy and port pins: 258 success, 399 tolerance failures and
43 timeout paths. This isolates sampling parity from the separate
real Toolchanger constructor mismatch; it does not fix or load that
plugin in Rust.

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
