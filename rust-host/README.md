# Klipper Rust host port

This is an experimental host-side port, not a replacement for Klippy yet. Existing
Python host code, MCU firmware, and printer configuration are unchanged. Do not
use this crate to control a printer.

See [validation status and benchmark summary](PORT_STATUS.md) for
what works, what was measured, and what is still missing.


Phase 1 implements the Klipper transport envelope in Rust without third-party
dependencies: length, destination and sequence, payload, CRC, and sync marker.
It supports ACK frames, partial input, and decoding one frame from a concatenated
buffer. Tests use byte vectors from Klippy's Python implementation.

```sh
cargo test
```

The bootstrap stage now decodes the compressed MCU dictionary and uses its
message IDs for offline clock and uptime exchanges. Serial/CAN transport, scheduling,
G-code, and the Klippy socket API started as separate milestones. The
sections below document tested offline/PTY subsets; a live host remains
out of scope for the current preview.
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
boundaries. The planner also models `M204 S`, `M204 P… T…` (the
lower acceleration), and `SET_VELOCITY_LIMIT` for velocity, acceleration,
square-corner velocity and minimum cruise ratio. These commands change
only subsequent planned moves and never force a queue flush. A bare
`SET_VELOCITY_LIMIT` is a no-op in the preview rather than a status reply.
The preview also accepts coordinate-only `SET_GCODE_OFFSET`
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

With runtime limit changes included, the stock dispatcher comparison
now matches 7,083 line/flush and state records and 5,103 exact move
profiles. The separate Python+C step comparator matches 381 shaped
Cartesian and 441 shaped CoreXY direction/step commands including
clocks, with pressure advance enabled. A CLI test also verifies that
lower acceleration changes planned duration without changing the
three end-position counters. All of these checks are offline.

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

The offline motion preview now reuses a raw-event buffer for each solved
phase instead of retaining all crossings until a lane is finished. The
clock encoder carries the pending reversal, epoch, and same-direction
compression run across phase boundaries; handing it a batch does not
flush the queue. The preview still limits a lane to 65,535 raw crossing
and commit events, keeps its existing clock horizon, and does not gain
long-job or live transport support from this change. The stock C code
already accepts events incrementally and is unchanged. The position,
reversal and compression algorithms are unchanged.

In two 60-pair runs on separate pinned CPUs, compared with the prior Rust
preview, elapsed time including JSON output decreased 5.0% on the
2,500-move XYZ fixture, 5.1–6.5% on a 220-move layered fixture,
2.9–4.6% on a 420-move shaped extrusion fixture, and 1.6–2.4% on the
2,500-move sparse-tool fixture with one worker. Complete JSON (16,379,
14,935, 16,576 and 21,683 step records respectively) was byte-identical.
In nine independent process launches per arm from a minimal native
runner, median peak RSS decreased from 2,708 to 2,280 KiB on the layered
fixture and from 3,328 to 2,536 KiB on the shaped extrusion fixture.
These comparisons are against the prior Rust preview, not measurements
of printing speed or a Rust-versus-C runtime comparison.

To time the full offline preview before and after this change with
matching JSON output:

```sh
python3 bench/preview_cli_timing.py /path/to/prior /path/to/current --moves 2500
python3 bench/preview_cli_timing.py /path/to/prior /path/to/current --profile layered --moves 220
python3 bench/preview_cli_timing.py /path/to/prior /path/to/current --profile shaped-extrusion --moves 420
python3 bench/preview_cli_timing.py /path/to/prior /path/to/current --moves 2500 --tools \
  --baseline-workers 1 --candidate-workers 1
```

### Opt-in longer offline traces

Build with `cargo build --locked --release --features long-step-preview
--bin klipper-preview` to make `--long-step-preview` available. The normal
build does not include the extended queue. The option allows inert XYZ traces
or up to three modeled extruders with `--fixed-pressure-script` and one
temperature per tool (for example `--tool-temps 0:210,2:214,6:220`).
It follows the pinned C step queue's clock horizon, bounded queue extension
and prefix compression. A single step after a long idle can exceed the
normal compression horizon; if its interval does not fit the 32-bit
output field, the entire preview fails instead of wrapping it. This mode
does not run live queue deadlines or send anything to a device.

A lane has at most 1,000,000 crossing and commit events, 65,535 events per
solved phase, 131,072 queued clock slots, and 262,144 output records.
There are at most six sequentially solved lanes: three motors and three
extruders. The aggregate upper bounds are six million crossing/commit
events and 1,572,864 retained output records; only one lane's step queue
and phase scratch are active at once. The existing input/trajectory
limits are unchanged. This option rejects extra worker/initial-tool flags,
physical toolchanger operations, mixed heater plans, macro expansion, and
virtual SD. Saved mesh is supported in this mode only by explicit
`--mesh-script --mesh-profile NAME`. The script can load its selected profile,
clear it, or change validated X/Y/ZFADE offsets. Static profile-only mesh,
probing, homing, calibration and saving/removing profiles stay rejected.
The flag-free path keeps its earlier limits.

Pinned stock's planner, input shaper, pressure advance and C clock queue
matched all 257,111 timed records on a 3,000-move layered path and 115,855
on a 3,000-move shaped extrusion path. Smaller controls matched the default
Rust output byte-for-byte; an XYZ fixture without an extruder and a
43-second dwell also matched stock. A 1,056-move shaped virtual T0/T2/T6
script with named pressure changes matched 304,753 timed C records across
all six lanes. Re-selecting a lane preserves its dormant extruder position;
changing pressure on an inactive lane leaves XYZ, the other lanes and
duration unchanged. Scripted mesh fixtures that exceed the short event
limit matched 174,305 XYZ records, 176,641 single-extruder XYZ/E records,
and 309,466 shaped sparse-tool motor records against the pinned stock
handler and C solver. These are compatibility tests, not printing speed
measurements.

```sh
python3 bench/long_step_preview.py /path/to/pinned/klipper --target-dir target/long-preview
cargo build --locked --release --example tool_motion_replay
python3 bench/tool_motion.py /path/to/pinned/klipper --long-steps \
  --cli-binary target/long-preview/release/klipper-preview
```

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
The root tool catalog separately matches 836 numbered records across
220 configs against the pinned plugin's actual `Tool` initialization
reads, including global and per-tool XYZ offsets, restore axes and
detection presence. Unassigned slots are left out of this numbered
view. The catalog never resolves or opens the detection pins.
The separate dock-coordinate adapter saves the nozzle position, previews
in the dock frame, and restores only declared axes. It buffers inert motion
and preserves physical extrusion across the logical-state restore. Across
240 cases, including every restore-axis mask, 4,080 coordinate/feed
observations matched the plugin's actual dock and restore methods using
inert callbacks. The optional scripted composition accepts explicit
fixture lines for before/park/pickup/after, and releases the result only
after pickup identity and coordinate restore are confirmed. A failing
step discards the buffered profiles and leaves the selection faulted.
It does not read or execute arbitrary config hooks.

```sh
cargo build --locked --release --example toolchange_replay
python3 bench/toolchange_order.py /path/to/legacy/klipper /path/to/toolchanger
cargo build --locked --release --example tool_catalog_replay
python3 bench/tool_catalog.py /path/to/legacy/klipper /path/to/toolchanger
cargo build --locked --release --example tool_dock_replay
python3 bench/tool_dock.py /path/to/legacy/klipper /path/to/toolchanger
```

The modern Klippy debug-output test also loads physical T0/T2/T6 as
`extruder`/`extruder1`/`extruder2` with distinct MCU output files.
A T0 → T2 → T6 → T2 run recovers T2's parked E and rebases logical E
to zero. Hotend targets also route by physical tool number even while
T0 is selected, without waiting for a physical temperature. The
individual outputs contain 640, 160 net (320 forward,
160 reverse), and 960 steps; all 36 motor records match with stock
or Rust look-ahead. A missing `[extruder1]` is rejected at startup.
The test initializes selection directly, without triggering docking.

### Preview sparse tool motion from configuration

For a file using the supported motion-only G-code subset, the offline
CLI can now read independent extruder calibrations and an explicit
Klipper Toolchanger tool-to-extruder map. Supply a modeled temperature
for every lane; the command neither reads nor controls a heater. It
emits separate JSON step records with the physical tool ID, configured
extruder name and modeled step distance. The example has no real pins,
does not dock tools, and does not apply tool nozzle offsets or per-tool
shaper changes; configurations that require those features are rejected.

```sh
cargo run --locked --release --bin klipper-preview -- \
  --tool-temps 0:210,2:210,6:210 examples/preview sparse-tool.cfg \
  examples/preview/sparse-tool.gcode 20000000 1 1 .2 printer.cfg rails.cfg
```

The offline CLI writes the same JSON directly from the motion records,
through a buffered writer, without building a second tree of per-record
maps. For 2,500 synthetic CoreXY moves (16,379 records and 1,211,520
output bytes), all 40 alternating runs preserved the output SHA-256.
Median **whole CLI** CPU time was 34.864 → 6.905 ms and elapsed time
35.136 → 7.100 ms (about 80% lower) versus the prior Rust CLI. These
figures include launch, planning, serialization and writing the output
to a local file; they are not a Rust-versus-Klippy or printer benchmark.

```sh
taskset -c 3 python3 bench/preview_cli_timing.py \
  /path/to/baseline/klipper-preview target/release/klipper-preview \
  --rounds 40 --moves 2500
```

The integrated offline SD/job model can also run these named tool lanes
after verified virtual homing. Its read-only `virtual_tool_lanes` object
reports the *planned* active tool as the virtual cursor advances; pausing
freezes changes, and cancellation discards them. Per-tool E observations
rebase print statistics on switches, so a different parked E position
does not become apparent extrusion. Integration tests run
T0 → T2 → T6 → T2, preserve all six virtual motor lanes, and verify
the pause, resume, final filament total and read-only status. The status
does not attest to actual hardware state or physical docking.

For explicitly supplied synthetic dock programs, the separate lane-aware
exchange preview follows the stock action ordering: pre-change/dropoff
extrusion stays on the outgoing lane, and the parked E position of the
incoming lane is restored before pickup/after-change extrusion. Both
sets of staged profiles are returned only after detection verification
(if enabled) and coordinate restoration succeed. A failed pickup or
malformed synthetic script yields no partial profiles and faults the
inert changer. The actual stock coordinate/configure handlers and
GCodeMove activation matched 5,040 fields across 240 supplied cases.
This does not run actual docking macros or prove a safe physical path.
The machine-bound variant also checks virtual homing, XYZ travel, Z
velocity/acceleration and the correct lane's modeled extrusion limits
during every staged move, including automatic restore. Tests reject a
cold incoming lane, out-of-bounds dock coordinates and an offset that
would put restore travel outside the virtual envelope. The proof-gated
path can then encode the complete dock/restore and separate E lanes.
It commits the modeled tool only after step encoding succeeds.
The attested GPIO-free MCU fixture runs that trace in 29 frames,
matches 213 commands and checks five counters at 17 points. Wrong
pickup evidence yields no trace; a step encoding error keeps the
previous modeled tool. No real motor, probe, or dock is exercised.
A second fixture splits the tool job across two independently started
PTYs with different virtual clock origins and board-local OIDs (both
can use OID 0). It validates 140 carriage/outgoing commands and 28
incoming extruder commands with the same five final counters at 17
shared checkpoints. If either dictionary is unmarked, neither receives
configuration. A dropped secondary motion ACK stops all later messages
and prevents advancing either virtual clock; this is not a physical
cross-board emergency stop or real CAN timing validation.
Packing only consecutive commands for the same peer reduces the two
boards' frame counts from 140 + 28 to 19 + 5. Decoding the actual host
traffic verifies identical cross-board command order, and both
directions of the enqueue benchmark check all 17 counters. Thirty
alternating pairs took a median 3.464 → 0.533 ms until the last
software ACK (85% lower versus per-command Rust framing). Fault
injection still halts both peers before either virtual clock advances.

```sh
python3 bench/native_gpio_motion.py /path/to/pinned/klipper --dual-rounds 30
```



Long multi-tool previews can opt in to 1–8 lane workers with
`--lane-workers`. Shared G-code geometry is planned once; step solving
runs independently, and motion is written in the original lane order.
Short paths and the default remain single-threaded. In two 40-pair
runs on a four-logical-CPU affinity, the same 2,500-move sparse fixture
took 29.213 → 16.201 ms and 32.366 → 20.667 ms median elapsed
(36–45% less). With two workers it took 29.293 → 18.595 ms.
All 21,683 records and complete JSON were byte-identical; the parity
suite also compares all 1–8 requested workers. The numbers are for
the offline CLI, not the real printer or Klippy daemon.

```sh
taskset -c 2,3,4,5 python3 bench/preview_cli_timing.py \
  target/release/klipper-preview target/release/klipper-preview \
  --tools --candidate-workers 4 --rounds 40 --moves 2500
```

When 3 or more workers share more lanes than workers, each worker now
claims its next lane after finishing rather than following a fixed
stripe. This spreads input-shaper and pressure-advance workloads while
preserving the original output order. One and two workers, or one
worker per lane, retain the previous scheduling. With 4 requested
workers, two 40-pair runs of the same 2,500-move three-tool file gave
17.490 → 14.111 ms and 16.154 → 13.045 ms median elapsed time. The
median within-pair reduction was 19.1% in both runs (39/40 and 38/40
wins); 3-worker reductions were 13.0% and 9.5%. Each pair used the same
CPU affinity, worker count, and output destination; all 21,683 records
and complete JSON matched. This includes parsing, planning, process
startup and writing the JSON; it excludes printer I/O.

```sh
taskset -c 2-5 python3 bench/preview_cli_timing.py \
  /path/to/pre-scheduler/klipper-preview target/release/klipper-preview \
  --tools --baseline-workers 4 --candidate-workers 4 --rounds 40 --moves 2500
```

The CLI can explicitly load one fixed version-1 bed mesh profile from
a regular section or the entry file's Klipper `SAVE_CONFIG` block.
It reads only the requested saved mesh, never writes configuration,
and emits a preview constrained by the simulated physical travel after
mesh compensation. In an interpolation/fade fixture, both config forms
produced the same 8,797 motor records and 5.781542142 modeled seconds as
stock ProfileManager + GCodeMove and the C step solver. This does not
probe a bed or compose a mesh with scripted dock motion.

```sh
cargo run --locked --release --bin klipper-preview -- --mesh-profile test_surface \
  examples/preview mesh-saved.cfg examples/preview/path_mesh.gcode \
  20000000 1 1 .2 printer.cfg rails.cfg
```

In a motion-only preview, `--mesh-script --mesh-profile test_surface`
instead starts unmeshed and lets the script load its one allowed saved
profile with `BED_MESH_PROFILE LOAD=test_surface` or disable compensation
with `BED_MESH_CLEAR`. It reinterprets logical coordinates at the current
physical position; it neither moves the toolhead nor flushes look-ahead.
The output's `mesh_script_profile` records the allowed name, not the
profile active at the end. A pinned stock handler comparison across
load, clear and reload matched all 8,804 timed motor records. This mode
rejects unknown profiles, saving/removing profiles, calibration, mixed
heater stages, and physical docking. An explicitly modeled single
extruder can combine this with `--fixed-pressure-script --tool-temps 0:210`.
The stock mesh and pressure callbacks together matched all 4,640 timed
XYZ/E records across loaded, cleared and reloaded mesh states. Removing
gain changes leaves the XYZ trace and duration unchanged.
The same opt-in flags can also bind every configured virtual lane
(for example `--tool-temps 0:210,2:214,6:220`). Sparse tool changes
preserve the mesh state and parked lane positions, and a named inactive
pressure update still affects only its own lane. Combining stock shaped
XYZ, load/clear, named pressure compensation and virtual T0/T2/T6 routing
matched all 9,934 timed records. One and four workers gave identical
output. The preview does not model physical pickup or probing.

Within the same opt-in mode, `BED_MESH_OFFSET X=<mm> Y=<mm>` updates the
mesh lookup coordinates without changing the commanded physical point
or draining look-ahead. Values replace the supplied axes, omitted axes
keep their setting, and an update without an active mesh has no effect.
Loading a profile again resets offsets. The pinned stock handlers and C
step solver matched 7,614 timed XYZ records and 9,920 timed XYZ/E
records with shaped virtual T0/T2/T6 and named pressure updates.

`BED_MESH_OFFSET ZFADE=<mm>` changes only the Z coordinate used to
calculate the fade factor; it does not move the current physical point.
It accepts finite values between -1000 and 1000 mm in this offline mode.
X/Y and ZFADE may be set together or independently; omitted fields are
retained, a reload resets all three, and ZFADE has no motion effect when
fade is disabled. The stock handler and C solver matched 8,638 timed XYZ
records and 9,866 timed XYZ/E records across shaped virtual T0/T2/T6,
including transitions in and out of the fade range. This
extends offline compatibility, not the interpolation algorithm or the
default stock runtime.

## Inert multiple-extruder coordinates

Switching a virtual extruder now returns the pending profiles tagged for
the outgoing lane, rebases the new tool to its parked physical E
coordinate, and resets logical E and flow exactly as stock GCodeMove
does on activation. The switch itself emits no extrusion. Comparing with
the actual stock activation handler on an inert toolhead matched 21,120
coordinate snapshots with flow changes, state restore and retractions.
Explicit tool-labeled sections can also share one XYZ timeline while
feeding independent pressure-advance E0/E1 queues. On a shaped A-B-A
path, 234 decoded step commands and their clocks match pinned stock
Python+C (1.270130318 simulated seconds). The GPIO-free C firmware mode
checks all five motors at 17 checkpoints, reaching
`[3920, -80, 40, 150, 175]` on the fixed path. This validates moving
with explicitly selected lanes, not automatic T-command docking, a full
tool-change plugin runtime, or any physical setup. The preview accepts
bare `T0`–`T255` selection lines as an alternative to explicit sections.
They select only a declared simulation lane; they do not dock tools or
run T-command macros. The independent Rust trace also ran T0 → T2 → T6
→ T2 with shaping, pressure advance, and a T2 retraction. All 342 step
commands across six motor lanes matched stock Python+C, including clocks. A missing lane is rejected even on a trailing T
line with no motion; `M104 T1` is not a selection line.

```sh
cargo build --locked --release --example multi_extruder_replay
python3 bench/multi_extruder.py /path/to/pinned/klipper
cargo build --locked --release --example tool_motion_replay
python3 bench/tool_motion.py /path/to/pinned/klipper
```

## Inert fan and tool fan handoff

The fan queue coalesces requests, applies minimum scheduling intervals and
kick-start hysteresis, and emits normalized PWM/digital intents without
accessing a fan or any pin. Real stock `Fan` and `GCodeRequestQueue`
callbacks matched 22,800 snapshots and 11,307 inert output events.

The tool fan handoff carries the observed effective fan speed across
unmount/mount and never guesses if the outgoing measurement is missing.
Against the optional legacy plugin reference it matched 11,700 snapshots
and 8,664 intents. The legacy `multi_fan` command adapter is not invoked.

```sh
cargo build --locked --release --examples
python3 bench/fan.py /path/to/pinned/klipper
python3 bench/fan_transfer.py /path/to/legacy/klipper /path/to/toolchanger
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
preflight. No physical pins, pulses, motors or physical timing are tested
in this deterministic fixture. The sparse three-extruder mode sends 372
commands in 50 frames and matches the six firmware position counters
(`[4080, 400, 40, 100, 155, 50]`) at 17 checkpoints. It refuses a normal
Linux MCU dictionary before any configuration or motion command.

The same four-lane motion payloads can be packed into 38 firmware frames
instead of 277 single-command frames without reordering them. In 30
rotating-order runs against the same fixture, the measured host enqueue
stage (collecting/encoding payloads and waiting for software PTY ACKs)
went from 1.771 ms to 0.282 ms median (84% less wall time). This compares
two Rust transfer strategies, not Rust vs stock Klippy's serial batching
or actual print time. Both variants check each of the 17 motion
checkpoints against the C firmware counters. A separate relay test drops
the first grouped motion write ACK and checks that the host exits without
retransmitting, sending any later frame, or advancing the inert clock.

```sh
cargo build --locked --release --example native_mcu
python3 bench/native_gpio_motion.py /path/to/pinned/klipper --rounds 30
# Or add --native-gpio to bench/check_parity.py --native-pty.
```

Integration tests verify that a simulated fault drops
remaining work and that separate Unix stream pairs carry read-only JSON
subscriptions without accepting G-code or letting one slow client exhaust
the reply queue. Two-client checks cover disjoint fields, absent-field nulls,
coalesced requests, replacing subscriptions, unchanged updates and peer
disconnect. These are direct read-only Klippy protocol checks, not Moonraker
startup or write-command compatibility. The named Unix listener is optional and its bind/cleanup
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

The pulse sampler now retains a reference to the current trajectory
phase and only looks it up again when crossing a boundary. Pulse order
and floating-point operations are unchanged. Two independent single-CPU,
35-round runs with prior Rust at `ab3f1ac7a366171cf9f681e04d54faf601c9ee58`
gave 5.123 → 4.906 ms and 4.691 → 4.557 ms median CPU (4.2% and 2.9%
lower). Stock was 8.450 and 7.595 ms respectively. All 6,835 records and
11.342102983 simulated seconds matched in all rounds. This is not a
benchmark of the full Klippy host or of physical print time.
The complete offline preview CLI, including parsing and JSON output, saw
paired elapsed changes of +0.1% and −2.3% across two 40-round runs of
2,500 three-tool moves. All 21,683 output records were byte-identical.
That test did not establish a repeatable CLI-wide gain from this change.

```sh
taskset -c 2 python3 bench/machine_trace.py /path/to/pinned/klipper \
  --rounds 35 --shape-timing --baseline-rust /path/to/baseline/rust-host
```

The optional Python/Rust look-ahead adapter reuses aligned per-thread
buffers and packs/unpacks rows directly. A separate benchmark replays
6,366 queues captured from actual stock Klippy Move/LookAheadQueue
objects (25,836 planned moves) and times the full flush, including
Python/Rust conversions and applying the profiles. Across two
45-round, rotating-order runs, median CPU time changed from
47.906 to 44.712 ms and from 45.990 to 42.003 ms versus the old adapter
(7–9% less). Stock medians were 48.309 and 46.044 ms. The actual Klippy
debug-output fixture independently matched all 4,299 decoded motor
commands with zero bridge fallbacks. This times the look-ahead seam,
not the complete host process.

```sh
taskset -c 2 python3 bench/hybrid_lookahead_timing.py /path/to/pinned/klipper \
  target/release/libklipper_host_protocol.so --rounds 45 \
  --baseline-adapter /path/to/baseline/klipper_rust_lookahead.py
```

The whole-process A/B runs the same program in two fresh instances of
pinned Klippy. It times launch, parsing, scheduling and writing simulated
MCU debug files, not printer I/O. It checks every timed motor record
and requires complete bridge planning with zero fallbacks. The prior
12,000-move runs could not reproduce the isolated look-ahead speedup;
the expanded 1,200–40,000 move matrix in [PORT_STATUS.md](PORT_STATUS.md)
also does not establish a reliable whole-host speedup.

The default `xyz` workload has three moving axes. The
`shaped-extrusion` profile additionally exercises MZV/EI shaping,
pressure advance, and relative extrusion on a fourth motor with
proportional extrusion per move. `--wait-interval 0` removes
intermediate waits for that comparison. Each test still performs a
final `M400`. The generated configuration is only valid for the
disposable debug-output fixture.

```sh
taskset -c 2 python3 bench/hybrid_process_timing.py /path/to/pinned/klipper \
  /path/to/pinned/klipper/out/klipper.dict \
  target/release/libklipper_host_protocol.so --rounds 15 --moves 40000 \
  --profile shaped-extrusion --wait-interval 80 --json timing.json
```

Use `--baseline-library /path/to/old/libklipper_host_protocol.so` to
test two Rust library builds with the same Python adapter. Both sides
must then report zero look-ahead fallbacks. The JSON report retains each pair,
bridge coverage and library SHA-256. Hashing, verification and report
writing are all outside the measured child-process interval.

### Opt-in Rust G0/G1 dispatch experiment

`--rust-dispatch` on the pinned debug-output benchmark recognizes
bounded batches of canonical ASCII, space-delimited G0/G1 with unique
X/Y/Z/E/F arguments. Bounded inline comments can follow the ASCII
arguments, including UTF-8 text. The Rust scanner returns only offsets; the
original Klippy handlers still validate values, plan motion, and
produce ACKs and error responses. Standalone comments, unsupported spellings,
short batches, and uncertain syntax use the stock dispatcher. It is
disabled by default and refuses an unrecognized dispatcher. It is not
a printer installation option or proof of full plugin compatibility.

The 40,000-move shaped-extrusion workload without intermediate waits
matched all 186,972 timed motor records, with every movement covered
by the scanner and no look-ahead fallback. In two 15-pair, single-core
runs, the candidate plus look-ahead bridge reduced paired median
whole-process elapsed by 5.44% and 4.84% versus stock. The timed scope
includes startup, parsing, scheduling, and debug-file output; physical
printer I/O and printing time are excluded. The separate dispatcher
fixture submits 72,000 lines across 2,400 batches and compares ordered
parameters and fallback, ACK, and error behavior to stock.

The benchmark now explicitly drains debug output on both sides when
using `--rust-dispatch`, so a partial file cannot count as a speedup.
Two drained 40,000-move repeats measured 5.89% and 10.31% lower paired
elapsed, with 13/15 candidate wins in each. With draining, 12,000-move
comparisons retained 43,033 exact XYZ and 57,245 shaped-extrusion
motor records with 12,000 moves and a wait every 80 moves. The 12 paired
elapsed medians improved 6.98% and 5.57%, respectively, versus stock.

The synthetic `layered` fixture uses perimeters, infill, variable
extrusion, retract/unretract, non-extruding travel, layer changes,
comments and periodic waits. With 12,000 moves, support for inline
comments and UTF-8 batches increased native dispatch coverage from
2,992/12,000 to 11,727/12,000; all 60,602 timed motor records matched.
Two 20-pair runs against stock changed elapsed by −4.29% and +4.81%;
the same comparison against the prior optional scanner was −2.94% and
−0.26%. This fixture demonstrates broader coverage, not a dependable
whole-host speed gain for slicer-style files.

```sh
cargo build --locked --release --features python-bridge --lib
python3 bench/hybrid_dispatch_parity.py /path/to/pinned/klipper \
  target/release/libklipper_host_protocol.so
taskset -c 2 python3 bench/hybrid_process_timing.py /path/to/pinned/klipper \
  /path/to/pinned/klipper/out/klipper.dict target/release/libklipper_host_protocol.so \
  --rust-dispatch --moves 40000 --rounds 15 --profile shaped-extrusion --wait-interval 0
taskset -c 2 python3 bench/hybrid_process_timing.py /path/to/pinned/klipper \
  /path/to/pinned/klipper/out/klipper.dict target/release/libklipper_host_protocol.so \
  --rust-dispatch --moves 12000 --rounds 20 --profile layered --wait-interval 1024 --json layered.json
```

Use `--factorial` to measure the scanner separately from the look-ahead
bridge. It runs stock, look-ahead only, scanner only and both against
one pinned fixture. All four modes drain debug output, check every
timed motor record and rotate through a balanced run order. Use a
multiple of four timed blocks. The JSON preserves the per-block
process CPU time, elapsed time, mode switches and actual coverage.
This option does not enable either path outside the offline harness.

```sh
taskset -c 2 python3 bench/hybrid_process_timing.py /path/to/pinned/klipper \
  /path/to/pinned/klipper/out/klipper.dict target/release/libklipper_host_protocol.so \
  --factorial --moves 40000 --rounds 12 --profile shaped-extrusion --wait-interval 0 \
  --json factorial.json
```

For an isolated language comparison, the [C scanner](bench/dispatch_scanner.c)
uses the same ABI, Python adapter, stock handlers, and byte-for-byte
offset validation as the Rust scanner. The benchmark runs stock and
each scanner independently without look-ahead replacement, with
complete debug draining, exact timed motor comparison, and separate
scanner-only CPU samples. See [the results](PORT_STATUS.md#c-and-rust-versions-of-the-same-g-code-scanner).

```sh
taskset -c 5 python3 bench/dispatch_language.py /path/to/pinned/klipper \
  /path/to/pinned/klipper/out/klipper.dict target/release/libklipper_host_protocol.so \
  --profile layered --moves 12000 --rounds 12 --wait-interval 1024 --json dispatch-language.json
```

Step timing now retains the clock epoch until compression advances it,
rather than revalidating and recomputing it on every crossing. Two
60-round rotating-order runs matched the same 6,835 records and modeled
time. On one pinned core, median CPU time was 7.789 ms stock, 4.955 ms
before the cache, and 4.750 ms after; on a second core the medians were
7.699, 4.910, and 4.772 ms. The cache reduced median CPU time by about
3–4% over the immediately preceding Rust kernel, not by 3–4% of actual
print time.

The reversal filter rolls back a same-tick, uncommitted opposite pair
as stock C does. A third raw crossing may reuse an empty tick, but the
model still rejects backwards clocks and committed or same-direction
duplicates. In a 2,500-move sparse shaped/pressure-advance fixture,
the CLI matches all 21,683 stock Python+C motor records across six
lanes, including clocks. In a separately matched serializer A/B, both
binaries used this same motion kernel, output 1,611,322 identical bytes,
and elapsed time was 67.097 → 29.173 ms over 40 rotating-order rounds
(56% lower versus the prior Rust JSON implementation).

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

## Modern Toolchanger probe candidate (pinned debug tests)

The compatibility failure below is reproducible with the old plugin pin.
The reviewed, unmerged
[viesturz/klipper-toolchanger PR #174](https://github.com/viesturz/klipper-toolchanger/pull/174)
candidate `0bded502b27dae5b9261789e03b0c8a0b84f48fb` also has an explicit
reproducer. In a disposable checkout of the same stock Klippy pin,
all three Klippain 300 mm, two-tool debug cases passed (StealthChanger,
TapChanger, and StealthChanger with a shared LED chain). Inert probe
sessions verified T0/T1 routing, distinct offsets, and refusal with no
active probe. With the optional Rust look-ahead bridge and plugins loaded,
stock and bridged debug motion produced 416 equal motor commands.

```sh
python3 bench/toolchanger_candidate.py --port /path/to/pinned/klipper \
  --dictionary /path/to/matched/klipper.dict --klippain /path/to/klippain \
  --candidate /path/to/candidate/toolchanger --led-effect /path/to/led-effect
```

The comparator expects the candidate and its base commit in the Git
checkout. It uses no live printer, and does not install, endorse or merge
the candidate. Add `--bridge /path/to/libklipper_host_protocol.so` to check
debug motion after building with `--features python-bridge`.

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

The separate `thermal::commands` model routes stock M104/M109,
M140/M190, named heater targets, all-off, and sensor-range waits
against declared offline heaters. Explicit T selects a contiguous
stock heater index, not a sparse physical Toolchanger number. The
reference calls the actual Klippy command handlers using inert
observations: 1,848 target snapshots, outcomes and poll counts matched
in normal and debug modes. The Rust model rejects non-finite values;
missing or stale samples never complete a wait, and a sample from the
wrong source is rejected. No sensor or heater I/O is available.

```sh
cargo build --locked --release --example thermal_command_replay
python3 bench/thermal_commands.py /path/to/pinned/klipper
```

Wide inert job schedules now index one pending head per lane. For
fewer than nine lanes they retain the small scan. The heap preserves
required-tick ordering, same-tick lane ties, and each lane's order.
Across 25 paired runs per workload with 16 lanes and 73,156 records,
median poll time dropped 29–40% versus the original Rust scan, with
identical ordered payload digests. The 11-lane tests improved 12–35%;
the 3/6-lane paths retained the small scan and varied within 6%.
Fault and cancellation still clear all pending work, without I/O.

```sh
cargo build --locked --release --example virtual_job_timing
taskset -c 2 python3 bench/virtual_job_timing.py /path/to/baseline/virtual_job_timing \
  target/release/examples/virtual_job_timing --rounds 25
```

Step payloads now keep their bytes in an inline 18-byte buffer, sized
for the worst-case Klipper VLQ fields. Output bytes, required ticks
and emission ordering are unchanged; command preparation no longer
allocates a separate byte vector per record. With 16 lanes and 73,156
commands, construction (validation, encoding and queue creation) was
23–38% faster in the three 25-round paired workloads. This measures
inert Rust job preparation, not an MCU transfer.

```sh
taskset -c 3 python3 bench/virtual_job_timing.py /path/to/baseline/virtual_job_timing \
  target/release/examples/virtual_job_timing --prepare --rounds 25
```

A simulated job can pause its release cursor at a temperature wait
created by the offline heater command model. Generic resume does not
bypass this pause. No queued work is released on missing, stale, or
wrong-source readings; a sufficiently fresh matching observation can
resume without replaying the elapsed wall clock as motion. A simulated
shutdown faults instead. The read-only `virtual_heat_wait` status
reports whether this gate is active. It does not issue heater commands
or assure that any physical reading is valid.
The explicit simulation boundary can now also consume stock heater
G-code text. It stages the target update and accepts it only if the wait
can be installed. It does not interpret Klippain's T-number remapping
or automatically schedule arbitrary job files; bare T selections stay
distinct from heater indices. Parsed text and typed commands both
match the 1,848 stock heater handler outcomes in the reference fixture.
A bounded borrowed-text splitter can now identify such heater
boundaries in a mixed file while leaving all nonheater G-code and its
line numbers intact. It does not time, validate, or release motion
chunks. A future integrated executor must enforce each barrier before
submitting work from the next motion chunk.
A separate preflight now carries one coordinate/extrusion state across
those chunks, with an explicit full look-ahead flush at each boundary.
It returns no partial plan if any later phase is invalid. The reference
adds the same conservative barriers to stock Klippy: 180 mixed files,
2,160 motion profiles and 720 boundary positions match. This compares
the explicit-barrier schedule, not stock's overlap at heater commands.
The machine-bound compiler can also encode those stages on separate
inert clocks without resetting the underlying fractional step grid.
A sub-half-step move before a wait is not lost or counted twice when
the next move crosses the threshold. Signed final positions match
stock C for 73 synthetic phase sequences each with and without static
input shaping. This does not reproduce absolute stock clocks across
the virtual heater wait, or allow a real MCU to be reset mid-print.
The inert mixed-job runner then holds each compiled motion phase in a
distinct queue and generation. A later phase cannot emit even a
zero-release-tick command until its heater wait receives acceptable
simulated evidence. Tests cover stale and wrong sources, multiple
waits, large host-clock jumps, cancellation, and simulated shutdown.
In the GPIO-free Linux firmware fixture, a two-phase mixed job sends
13 commands in each accepted phase and reaches motor counters
`[560, -80, 0, 75]`. With simulated heater shutdown at the second wait,
the second phase sends zero commands and all four counters remain at
the first-phase values. This fixture has no physical heat or sensors
and refuses an ordinary Linux MCU dictionary.
The fixture also opts into an explicit motion completion gate: software
ACK count, generation, and an observed virtual tick covering phase end
must match after the firmware counters agree. Draining the host queue
or waiting on its wall clock is not sufficient. The read-only
`virtual_mixed_job` snapshot distinguishes waiting for motion proof from
waiting for temperature. Wrong proof leaves the next stage closed.


The CLI can display this staged plan without running it. This opt-in
variant uses stock heater indices and a fixed initial extruder, not
Toolchanger macro remapping. A modeled T0 temperature is mandatory
for extrusion, and declared heater waits remain unresolved in JSON.

```sh
cargo run --locked --release --bin klipper-preview -- --mixed-plan --tool-temps 0:210 \
  examples/preview mixed.cfg examples/preview/path_mixed.gcode \
  20000000 1 1 .2 printer.cfg rails.cfg
```







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

The opt-in `--fixed-pressure-script` offline mode recognizes
`SET_PRESSURE_ADVANCE ADVANCE=<positive gain>` with an already-enabled
fixed smoothing window. Like the pinned stock look-ahead callback, a
gain starts at the end of the last queued move, or immediately if the
queue is empty. It does not flush the queue. The reference invokes
stock's Python handler and timing callback, then sends the resulting
schedule through the real stock C step solver. Short boundary cases
matched 342 Cartesian and 382 CoreXY records; rolling 320-move cases
matched 1,334 and 1,734. Removing the commands left XYZ timing unchanged
and changed the extruder trace. Changing smoothing, zero gain, and
externally scheduled gain changes mixed with these commands are outside
this mode. A standalone single-extruder preview accepts only implicit
routing.

The same flag can be used with one modeled temperature per virtual tool,
for example `--fixed-pressure-script --tool-temps 0:210,2:214,6:220`.
A bare `T` selector changes the implicit target; the previous lane keeps
its independent gain history. Every lane receiving a gain change must
start with positive PA and smoothing. Tests invoking the pinned stock
activation and implicit pressure-advance handlers matched 1,153 and 9,159
timed records across XYZ and T0/T2/T6 on shaped paths. Removing the gain
commands left XYZ and duration unchanged and changed each E trace.
In virtual multi-tool mode, `EXTRUDER=extruder1` (or another exact
configured backing name) explicitly selects an independently stepped
extruder, even while a different tool is active. The command binds to
the same shared look-ahead boundary without selecting or moving that
tool. Logical tool aliases such as `T2` are not extruder names. The
named stock mux fixture matched 9,889 timed motor records; removing one
inactive T6 update changed only the T6 E lane, not XYZ, duration, or
the other E lanes.
This does not execute docking, toolchanger initialization, or the
mixed heater/motion plan.

```sh
cargo run --locked --release --bin klipper-preview -- \
  --fixed-pressure-script --tool-temps 0:210 examples/preview single-pressure.cfg \
  examples/preview/single-pressure.gcode 20000000 1 1 .2 printer.cfg rails.cfg
```

The temperature is a modeled input. This is an offline trace, not a
printer installation or a live calibration command.

The crossing solver now resolves the pressure gain once per ordered
trajectory phase rather than looking it up inside each integration.
Eligibility, gain-change boundaries, and floating-point calculation
order are unchanged. An explicit boundary test also compares
prepared gains directly with the individual-query path.

Two 50-pair, single-worker runs of the offline 2,500-move three-tool
preview against `c4c58ab41a74ec1505aaa217d42176b8189c10d1` measured
34.461 → 29.679 ms and 30.985 → 27.041 ms median elapsed. The median
within-pair reductions were 11.5% and 10.3%. With four workers on
both sides, the within-pair reductions were 7.5% and 5.9% in two
more 50-pair runs. All 21,683 motion records and the full JSON
file matched byte for byte. These are measurements of the entire
offline preview process, not live Klippy or physical printing.
A separate direct comparison to the pre-scheduling Rust build at
`ab3f1ac7a366171cf9f681e04d54faf601c9ee58`, with four workers
on both sides, measured 21.8% and 23.2% paired reductions. Those
results were measured together; they are not sums of the individual
optimizations.

```sh
taskset -c 2 python3 bench/preview_cli_timing.py \
  /path/to/c4c58ab4/klipper-preview target/release/klipper-preview \
  --tools --rounds 50 --moves 2500
```

The iterative solver also prepares pressure-integration terms once per
phase, without changing the order of floating-point operations. On two
45-pair reruns of the same 2,500-move CLI job against
`3af52d0d33ea2c45859f2f53863f957b4b3a6b7c`, the paired median elapsed
reduction with one worker was 2.9% and 3.9% (33/45 and 36/45 wins).
With four workers on four CPUs, it was 2.7% and 0.9% (28/45 and 23/45
wins), a smaller and less stable effect. All 21,683 timed records and
the complete JSON file remained identical. This is additional offline
preview processing improvement, not a real-print speed measurement.

The prepared-phase algorithm also has a benchmark-only C counterpart.
The script applies [the C patch](bench/pa_prepared_c.patch) to a
temporary copy of the pinned helpers, never to the checkout. It tests
positive runtime gain changes and benchmarks a shaped repeated path
with a balanced three-way order. See [results](PORT_STATUS.md#same-pressure-algorithm-in-c-and-rust)
for the native-C and full offline measurements. The stock C route is
unchanged; the C patch is not a streaming or printer installation.

```sh
taskset -c 5 python3 bench/pa_three_way.py /path/to/pinned/klipper \
  --rounds 48 --json pa-comparison.json
```

## Bounded slicer display status

The offline preview accepts unnumbered `M73 P<percent>` (optional
numeric `R` and `C` do not change displayed progress) and `M117 <text>`
as non-motion commands. Display commands neither flush look-ahead nor
alter the timed output. `M117` keeps its UTF-8 text and embedded
semicolons; an empty M117 clears the message. `SET_DISPLAY_TEXT MSG=...`
also supports quoted and escaped text; `;` and `#` start comments only
outside the message's quotes. Missing `MSG` clears the message, while
`MSG=""` explicitly sets an empty string. Numbered/checksummed lines and
malformed fields are not accepted by this subset.

The read-only `display_status_replay` fixture supplies explicit virtual
dispatch time, Printing/Idle state and optional virtual-SD progress.
It clamps M73 progress and uses the stock five-second expiration
outside Printing, then falls back to the supplied SD progress (or zero).
It publishes only a modeled `display_status` object; the preview does
not schedule display updates or contact a frontend or screen. The
pinned stock display handler and dispatcher match across 303 sampled
states; adding display commands left all 984 timed motor records
and the complete preview JSON unchanged.

```sh
cargo build --locked --release --example display_status_replay --bin klipper-preview
python3 bench/display_status.py /path/to/pinned/klipper \
  target/release/examples/display_status_replay target/release/klipper-preview
```

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

The runner checks the pin and reports each named phase; it does not
install anything or start a real printer service. The short commands
above intentionally omit optional phases. To reproduce the complete
stock and Toolchanger matrix, pass its pinned checkouts and the
matching Linux firmware dictionary explicitly:

```sh
cargo build --locked --release --features python-bridge
python3 bench/check_parity.py --upstream "$stock_klipper" \
  --dictionary "$stock_klipper/out/klipper.dict" \
  --macro-reference --native-pty --native-gpio \
  --legacy-klippy "$legacy_klippy" --toolchanger-reference "$legacy_toolchanger" \
  --modern-toolchanger "$modern_candidate" --klippain "$klippain" \
  --led-effect "$led_effect" --modern-bridge target/release/libklipper_host_protocol.so
```

The stock pin is `c9559fa04ec6b11fb1a9a1610903546319fcf310`; legacy
Klippy is `644cda5ecaa39d0dcf797624c19d5425cb8121ec`; legacy Toolchanger
is `94756dfde9b729fd69f9b8780067821c5c99a528`. The modern candidate,
Klippain and LED Effect commits are listed under the PR #174 debug
tests below. Check for all 56 named phases and do not infer coverage
from a short invocation. The optional `--moonraker "$moonraker"`
adds the client fixture below (57 total).

### Mixed sparse tools

The offline tool/thermal preflight requires an explicit mapping from
physical tool IDs (e.g. T0/T2/T6) to configured stock heater indices.
A bare selector changes the modeled active tool and preserves each
parked E coordinate. Explicit heater parameters such as `M104 T1`
still use the stock heater role, without changing the active tool.
Every motion or heater boundary is a conservative stop in this
preview; targets do not stand in for observations.

To see the same typed stages in the preview JSON, specify the starting
tool and one modeled temperature for every configured tool. The output
identifies which extruder lane supplies each motion and which heater
table entry a wait refers to. It does not clear any wait or run a tool
macro. An explicitly present stock `[fan]` also enables bounded M106
and M107 demand stages in this multi-tool preview. Fan demand advances
only after the preceding phase is committed; it is not scheduled PWM
or a physical fan speed. Named or indexed fan macros are unsupported.
The stock parser/handler differential covers 253 bounded fan requests.

```sh
cargo build --locked --release
target/release/klipper-preview --mixed-plan --initial-tool 0 \
  --tool-temps 0:210,2:215,6:220 examples/preview sparse-tool.cfg \
  examples/preview/path_sparse_mixed.gcode 20000000 1 1 .2 printer.cfg rails.cfg
```

The phase-local step compiler preserves cumulative signed steps at
each boundary, with separate extruder grids. In 41 generated
CoreXY jobs with input shaping, pressure advance, and returning
sparse tools, all six final signed motor positions matched pinned
stock Python+C. The comparator explicitly stops and allows filter
history to settle at the same boundaries. It does not assert equal
global firmware clocks or real docking motion.

The software-only program also requires motion completion evidence
before arming a new tool or heater phase. The GPIO-free firmware test
sends 13+13+13 records in a T2/T5/T2 path, checks all five cumulative
motor counters, and verifies that an injected shutdown during the
second wait leaves only the first 13 records issued. Unsupported
non-fixture firmware must reject both modes before configuring any
simulated motion. The runtime uses no physical dock or heater.

A second fixture routes the same path to two independently started
software MCU peers, using separate OID namespaces, phase-local clocks
and one common wire dictionary. It sends 39 commands, verifies all
five final counters, and withholds one peer receipt after the second
phase to prove that the final 13 commands stay unissued. An aggregate
ACK count is not accepted for routed stages. Batching only contiguous
commands to the same peer preserves all 39 commands and reduces wire
frames from 39 to 9. Thirty alternating fresh pairs measured summed
phase enqueue through ACK at 0.967 → 0.278 ms; a separate single-CPU
repeat measured 0.232 → 0.062 ms. Temperature waits, motion execution
and clock/position checks are not timed. A missing secondary receipt
blocks all final commands; deliberately dropping one secondary ACK
also prevents a second-stage virtual clock advance. An unconfirmed
fixture write now permanently closes that fixture session to further
writes and queries. An anonymous-socket regression verifies only the
first frame reaches the peer, even if a caller explicitly tries to
continue. This is not a SocketCAN or hardware driver.

```sh
python3 bench/native_gpio_motion.py "$stock_klipper" --mixed-rounds 30
```

```sh
cargo build --locked --release --example mixed_tool_steps
python3 bench/mixed_tool_steps.py "$stock_klipper" \
  --binary target/release/examples/mixed_tool_steps
```

### Read-only Moonraker client fixture

Use an unmodified Moonraker checkout at
`b559b9a81161f958a43c2f527b6ad96f73310d5d`. The fixture uses the
pinned `KlippyConnection` and `KlippyAPI` classes and injects only the
dial with a connected Unix socket pair. It verifies unavailable,
startup, ready, simulated shutdown and reconnection; a status query
and subscription diff; old subscriptions cleared and a newly
installed subscription receiving updates. The rest of Moonraker is
not started; no named socket or HTTP/WebSocket frontend is tested.

```sh
cargo build --locked --release --example moonraker_status_fixture
python3 bench/moonraker_connection.py "$moonraker" \
  target/release/examples/moonraker_status_fixture
```

Use the pinned Moonraker Python dependencies (fixture tested with
`importlib-metadata 8.7.0` and `tornado 6.5.8`). Unsupported endpoints
such as G-code execution and emergency stop are deliberately absent.
Moonraker's initial G-code output subscription receives the expected
unsupported-endpoint error; the test does not fake support for it.

The separate `--moonraker-http` option (58 phases with all other
flags) starts the pinned real Moonraker application on an ephemeral
`127.0.0.1` port with temporary data and an ephemeral test key. It
uses the same explicitly injected Klippy dial. The fixture verifies
unauthenticated rejection, authenticated HTTP readiness and query,
WebSocket status differences, shutdown/disconnect notifications, and
a fresh subscription after reconnection. It requires Moonraker's
stock Python dependencies. It does not create a real Klippy socket,
test native named-socket dialing, expose a server on the network, or
execute printer commands.

The modern Toolchanger debug-motion comparison explicitly waits for
all three stock C file-output queues before reading the recorded
commands. It checks the main and both tool files independently.
That wait affects only this fixture, not the normal benchmark or
a running printer.

Add `--moonraker-job` to include a 59th phase. The fixture privately
drives an actual virtual Rust Program and checks HTTP and WebSocket
views of its temperature gate, routed motion proof and fan demand.
A partial synthetic peer receipt leaves the state unchanged. Complete
synthetic evidence updates the visible tool and fan and opens the next
temperature gate. The same exercise checks the config and filtered
`fan.speed` subscription used by KlipperScreen's fan panel, with 0%
at startup, a 40% update and a fresh 0% after reconnect. RPM is null
because this is modeled demand, not fan feedback. The fixture does not
launch KlipperScreen or implement fan control. Its controls never
become Moonraker endpoints and cannot operate a printer.

```sh
python3 bench/moonraker_frontend.py "$moonraker" \
  target/release/examples/moonraker_status_fixture --program
```

### Status polling A/B

The read-only subscription update compares the borrowed current values
with the previous sent fields. It clones only changed fields and
advances the cache after a frame successfully serializes, not when an
oversize frame is rejected. Initially unresolved objects still bind
their field list on the first nonempty observation.

The fixture runs 12,000 polls for eight mixed full/field-filtered
subscriptions with mostly unchanged data. Across two 30-pair CPU
runs, previous Rust changed from 394.226 to 84.094 ms and from
392.586 to 81.125 ms (about 79% lower). The 3,432 ETX-delimited result
frames were byte-identical (399,216 bytes). The change is limited to
this sparse status workload, not Moonraker or whole-host latency.

Build the same `examples/status_update_timing.rs` in the old and new
crate checkouts, then compare both binaries:

```sh
cargo build --locked --release --example status_update_timing
taskset -c 2 python3 bench/status_update_timing.py \
  "$baseline/target/release/examples/status_update_timing" \
  target/release/examples/status_update_timing --rounds 30 --json status-timing.json
```

The checker verifies byte-for-byte output separately. Timed fresh
processes construct the response frames but do not write them onto
a physical socket or console.

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

### Virtual SD and print accounting status

The optional WebHooks projection maps existing in-memory state to
stock-shaped `virtual_sdcard` and `print_stats` objects. It uses committed
byte offsets, keeps an in-flight pause active until its issued line commits,
and publishes the empty SD slot on cancellation or reset. It consumes
already sampled print accounting and treats paths as metadata; it never
opens those paths or accepts commands.

```sh
cargo build --locked --release --example virtual_sd_replay --example print_stats_replay
python3 bench/virtual_sd.py /path/to/pinned/klipper --status
python3 bench/print_stats.py /path/to/pinned/klipper
```

The pinned handlers matched 17,784 SD and 12,480 print accounting status
snapshots through the Rust query handler. The pinned Moonraker loopback
fixture with `--program --delay-ready` additionally verifies HTTP,
projected WebSocket diffs, an unavailable G-code endpoint and fresh
reconnect. Adding the `--stock-klippy /path/to/pinned/klipper` option
compares the combined display/SD timeline with ten actual stock observations:
an expired M73 remains visible through the in-flight pause, falls back to
committed SD bytes when idle, and does not conflate empty with null text.
These are injected socket-pair software fixtures, not live
printing or named Klippy socket compatibility.

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

### Standalone shared-extrusion preview

`klipper-shared-preview` is a separate offline command that does not change
the existing `klipper-preview` executable. It reads an explicit model JSON
and G-code from ordinary files. It never reads a printer configuration,
serial endpoint, or hardware state. The XYZ start and both extrusion
temperatures in the JSON are simulated, not measured.

From this directory, to preview the absolute-extrusion test body:

```sh
sed '1d' bench/shared_extruder_fixture/absolute_flow.gcode > /tmp/shared-body.gcode
cargo run --locked --release --bin klipper-shared-preview -- \
  bench/shared_extruder_fixture/offline_model.json /tmp/shared-body.gcode
```

The sample's first line is fixture-only virtual initialization. For your
input, set the virtual start in the JSON instead of adding a homing or
position-forcing command. Finish with M400. Intermediate M400 commands
flush look-ahead but do not introduce wall-clock waits in this offline mode.
`G4 P<number>` adds a nonnegative decimal dwell in milliseconds to the
simulated timeline (plain `G4` is zero); neither command pauses the preview.
The total simulated motion timeline remains under 120 seconds.
`M204 S…` (or `M204 P… T…`) and `SET_VELOCITY_LIMIT` update limits for
subsequent moves without flushing queued moves. These affect only the
virtual machine; they do not change the modeled Z limit or printer settings.
Bounded slicer display metadata (`M73`, `M117`, and `SET_DISPLAY_TEXT`)
is accepted without changing the trace, including after the final `M400`.
The preview does not display progress or track text state.
`SAVE_GCODE_STATE` and `RESTORE_GCODE_STATE` use the bounded offline
coordinate snapshot store. An explicit `MOVE=1` emits a simulated XYZ
return move subject to the same virtual limits; it keeps the current
physical E position rather than undoing extrusion.
`SET_GCODE_OFFSET` uses the same virtual coordinate rules, including
incremental adjustments. If `MOVE=1` is requested, the resulting move
must stay within the configured virtual bounds.
The model covers CoreXY with two
logical extrusion queues and one shared E lane, fixed positive pressure
smoothing, and motion and queue switches supported by the offline planner.
It does not run macros, model a physical changer, or perform live pacing.
Unsupported commands and unknown model fields fail without a trace.
