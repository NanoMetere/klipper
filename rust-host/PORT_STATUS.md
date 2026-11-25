# Rust host port: validation status

This branch adds a host-side Rust preview, protocol models and a limited
Python/Rust bridge. It is **not** a replacement for Klippy, a firmware
update, or a printer installation. The existing Python host and MCU
firmware are unchanged. All results below are either offline reference
checks or disposable software MCU checks.

## Measured performance

These comparisons measure different boundaries. Do not add them or treat
them as changes in the elapsed time of a real print.

| Workload | Baseline and result | Meaning |
| --- | --- | --- |
| Shaped CoreXY + pressure advance, 6,835 exact records | Stock Python+C 8.450/7.595 ms, prior Rust 5.123/4.691 ms, current Rust 4.906/4.557 ms median CPU | 3–4% less CPU vs prior Rust and 40–42% vs stock for this offline config-to-step workload; not a whole-host benchmark |
| Klippy move look-ahead, 25,836 moves including Python/Rust conversions | Stock 48.309 → bridge 44.712 ms and stock 46.044 → bridge 42.003 ms median CPU | 7–9% less CPU in the captured queue flush seam |
| Whole pinned Klippy process, XYZ or shaped extrusion | 1,200 to 40,000 moves, every timed motor record equal; run-level paired elapsed changes vary in direction | No repeatable end-to-end speedup demonstrated from the optional bridge |
| Whole pinned Klippy debug process, 40,000 shaped extrusion moves, no middle waits | Opt-in Rust G0/G1 scanner plus bridge vs stock: 5.89% and 10.31% lower paired median elapsed in two 15-pair single-core runs with debug output drained | 186,972 exact motor records; actual Klippy with simulated file output, not a printer runtime |
| Whole pinned Klippy debug process, synthetic 12,000-move layered path | Scanner coverage rose from 2,992 to 11,727 moves; elapsed changed by −4.29% and +4.81% vs stock | 60,602 exact motor records; improved coverage, no dependable whole-process gain on this fixture |
| Offline Rust CLI, 2,500 CoreXY moves | Prior Rust CLI 35.136 → 7.100 ms median elapsed | About 80% less elapsed time after streaming identical JSON to a file |
| Offline Rust CLI, three tools, 2,500 moves, 21,683 exact records | Same motion kernel, prior JSON 67.097 → 29.173 ms; separately, one lane worker 29.213 → four workers 16.201 ms (repeat 32.366 → 20.667 ms) | Serialization and concurrency were measured in independent A/B tests |
| Offline Rust CLI, same three-tool workload and four workers on both sides | Fixed lane stripes 17.490 → dynamic scheduling 14.111 ms median elapsed; repeat 16.154 → 13.045 ms | 19.1% paired median reduction on both CPU-affinity sets, byte-identical JSON; not a Klippy or hardware result |
| Offline Rust CLI, same workload after pressure-advance gain preparation | One worker: 34.461 → 29.679 ms and 30.985 → 27.041 ms median elapsed | 10.3–11.5% paired reduction with one worker; 5.9–7.5% with four workers; byte-identical JSON, not a live host result |
| Offline Rust CLI, same three-tool path with prepared pressure integration terms | One worker: 2.9% and 3.9% lower paired elapsed; four workers: 2.7% and 0.9% lower | Two 45-pair runs per worker count against B118, 21,683 exact records and byte-identical JSON; four-worker benefit is smaller and less stable |
| Offline Rust CLI, reusing per-phase raw-event buffers | XYZ 5.0% lower in both runs; layered 5.1–6.5% lower; shaped extrusion 2.9–4.6% lower; three virtual tools 1.6–2.4% lower | Against the earlier Rust CLI, two 60-pair runs for each fixture, including JSON output. Complete output matched for 16,379 / 14,935 / 16,576 / 21,683 records, respectively. No change to stock C or print speed evidence. |
| Two GPIO-free software MCUs | 168 → 24 frames, 3.464 → 0.533 ms median enqueue through last software ACK | Same order, same five motor counters; this is Rust before/after framing, not physical transport or stock Klippy |
| Mixed virtual job across two software MCUs, 39 commands | 39 → 9 frames; host enqueue through ACK 0.967 → 0.278 ms; single-CPU repeat 0.232 → 0.062 ms | 71–73% less enqueue time in this fixture, identical command order and counters, not real print duration |
| Offline status polling, 8 subscriptions × 12,000 polls | Prior Rust 394.226 → 84.094 ms and 392.586 → 81.125 ms median CPU; 3,432 byte-identical frames | About 79% less CPU by avoiding copies of unchanged fields; synthetic sparse updates, not whole-host, Moonraker frontend or print speed |

All figures use the checked-in test fixtures, the same host for both
sides within a run, and no printing hardware. The platform is Linux
x86-64 (AMD EPYC 9V74), not a Raspberry Pi. Use the runnable commands in
[README](README.md) to reproduce the individual measurements.

### Offline worker scheduling experiment

The Rust CLI can already choose one or four workers for the same
three-tool preview. With the process restricted to one CPU, switching
from four workers to one lowered paired elapsed time by 2.6–2.8%, but
raised CPU time by 16.0–16.9%. On two distinct four-CPU affinity sets,
using four workers instead of one lowered paired elapsed time by
39–49%. Each affinity used 100 alternating pairs of 2,500 moves,
with exactly the same 21,683 step records and identical full JSON.
This compares worker settings, not languages or live printer speed.

A shadow C patch runs the pinned Klippy step-generation jobs inline,
keeping the same per-emitter work and output ordering. Twenty-four
alternating pairs per affinity tested whole processes with drained
debug files and stock 25 μs tolerance (60,340 exact layered records,
74,811 exact shaped-extrusion records):

| CPU affinity | Layered elapsed change | Shaped extrusion elapsed change |
| --- | ---: | ---: |
| One CPU, two runs | −2.34% / −6.18% | −2.18% / +2.52% |
| Two CPUs, two runs | −11.33% / −6.36% | −5.49% / −15.26% |
| Four CPUs, one run | −1.72% | +1.50% |

Negative values favor inline C. Output equality is limited to the
offline jobs; default-tolerance timing did not consistently improve
across workload and affinity, and no realtime behavior was validated.
Production C and the default Rust worker count remain unchanged.
For this offline workload, the Rust `--lane-workers 4` flag is useful
only when multiple CPUs are actually available and elapsed time matters.
The C fixture can be reproduced with
`python3 bench/step_compress_exact_zero.py /path/to/pinned-klipper
--shadow inline-steppersync --mode default --rounds 24`.

### Zero-error step compression

At zero timing tolerance, two increasing step timestamps uniquely
determine the interval and signed interval delta. The offline Rust
compressor now walks that progression instead of searching for an error
range. Nonzero tolerance still uses the general compressor. The pinned
stock and shadow C versions, and both Rust versions, matched 32,153
encoded blocks over 432,488 input clocks (2,222 runs), including clock
wrap, the signed delta limits, and a 65,535-step run.

The following are paired whole-process elapsed changes against each
language's own baseline. Both Rust executables include complete,
byte-identical JSON; the shadow C runs include pinned Klippy and drained
debug-file output. Negative numbers indicate a shorter elapsed time.

| Workload and timing tolerance | Paired elapsed change |
| --- | ---: |
| Rust CoreXY, 2,500 moves, zero | −4.8% / −3.9% |
| Rust layered, 220 moves, zero | −5.9% / −6.9% |
| Rust shaped extrusion, 420 moves, zero | −4.8% / −6.1% |
| Rust shared-extruder CLI, three fixtures, zero | −4.76% to −3.93% |
| Rust shared-extruder CLI, the same fixtures, 1,250 ticks | −1.38% to +0.09% |
| Shadow C layered, 12,000 moves, zero | −1.12% / −6.59% |
| Shadow C shaped extrusion, 16,000 moves, zero | −3.07% / −4.50% |
| Shadow C layered / shaped, stock tolerance | −5.75% / −0.34% layered; +1.06% / +0.69% shaped |

Rust used 70–100 pairs per fixture/core, and shadow C used 20 pairs per
fixture/core. The two zero-error C fixtures matched 623,651 and 563,044
motor records. The shadow's zero-error results varied more, and its
default-tolerance shaped runs did not improve, so the stock C source
remains unchanged. The Rust change is not a claim about live Klippy,
nonzero-error speed, or print duration. Reproduce the shadow comparison
with `python3 bench/step_compress_exact_zero.py /path/to/pinned-klipper
--mode exact --rounds 20` (or `--mode default`).

### Shared traversal for matching X/Y input shapers

The Rust offline CoreXY preview shares the phase walk and distance
evaluation only when the configured X and Y pulse times match bit for
bit. Each axis still accumulates with its own amplitudes in the
original order. The old path is selected once per lane for all other
configurations; no shaper policy is changed.

Paired median elapsed changes compare candidates with their own
original implementations. Rust measurements ran the full release CLI
including byte-identical JSON (two 80-pair runs). The C counterpart
ran actual drained stock Klippy debug jobs with a temporary helper
(two 24-pair runs); it is not a timing comparison between languages.

| Candidate and configured pulse times | Layered elapsed | Shaped extrusion elapsed |
| --- | ---: | ---: |
| Rust, matching | −1.71% / −1.29% | −5.15% / −4.27% |
| Rust, different | −1.20% / −1.11% | +0.01% / −1.83% |
| Shadow C, matching | −2.11% / −5.68% | −3.87% / −3.55% |
| Shadow C, different | −1.51% / −3.24% | +1.74% / +2.51% |

The matching Rust workloads verified 14,937 and 16,774 timed records.
The matching C workloads verified 54,083 and 70,156 timed records;
C runtime transitions matching → different → matching → disabled X
→ matching also reproduced stock. Two quiet 88-pair Rust repeats on the
unshaped 2,500-move control were unchanged or 0.5% faster in paired
elapsed (16,379 exact records). Same-time MZV and 3HUMP_EI fixtures
also match pinned Cartesian/CoreXY reference output with shaped Z and
pressure advance. The opt-in long preview matched 658,497 stock C timed
records with matching pulse schedules across eight workloads. Only Rust's
bounded offline path is changed; the
C implementation stays a benchmark-only patch because of the regression
when X/Y pulse schedules differ. These are not real-print timings.

Reproduce the C comparison from `rust-host` using the pinned Linux
software-MCU dictionary and the same Python environment as the parity
suite; `different` and `changing` are additional accepted modes:

```sh
python3 bench/shaper_shared_xy.py /path/to/pinned-klipper \
  --pulse-times matching --rounds 24
```

### Same pressure algorithm in C and Rust

Two 48-block comparisons of the B119 coefficient-preparation change
used the same pinned shaped CoreXY config and input. All results matched
6,835 timed motor records and the modeled elapsed time. The two C modes
use the same stock Python planner and C generation setup. The prototype
C change is confined to a disposable copy of Klippy helpers.

| Comparison against stock | CPU paired change, core 5 / core 7 |
| --- | ---: |
| Prepared C, native stage (trapq and step generation) | −2.12% / −1.86% |
| Prepared C with stock Python, offline config-to-steps | −1.54% / −0.89% |
| Prepared Rust, offline config-to-steps | −43.12% / −42.23% |

The C result is modest, so stock C is unchanged. The Rust numbers
include different planning and record-generation implementations; they
are not a language-only comparison or measured live print speed.

### C and Rust versions of the same G-code scanner

The C counterpart implements the same bounded grammar and offset ABI.
Both candidates use the identical Python adapter and original stock
handlers; the Rust look-ahead bridge is disabled. Two 12-block runs
per profile used drained stock Klippy output, and two 80-pair runs per
profile also timed the scanner calls alone. Negative means faster.

| Profile | Exact records | Rust vs C, scanner CPU | C vs stock, whole elapsed | Rust vs stock, whole elapsed | Rust vs C, whole elapsed |
| --- | ---: | ---: | ---: | ---: | ---: |
| Layered, 12,000 moves | 60,602 | +20.54% / +24.60% | −4.77% / −5.30% | −3.19% / −4.56% | −0.35% / +1.62% |
| Shaped extrusion, 40,000 moves | 186,972 | +17.24% / +22.22% | −6.93% / −3.25% | −6.86% / −3.20% | +1.75% / +3.03% |

The C implementation used less CPU in this scanner microbenchmark
(including the same FFI overhead on both sides). It did not show a
similarly large advantage in whole-process elapsed time. These are
fixed debug workloads, not evidence of real printer performance. The
default stock dispatcher and C helpers are unchanged; neither scanner
is selected by default.

### Separating scanner and look-ahead costs

Each table cell is the paired median elapsed change relative to stock
Klippy, across two 12-block runs pinned separately to CPU 2 and CPU 8.
Each block includes all four modes in balanced order, after warmup.
Every mode drained the same debug writer and matched all timed motor
records; neither mode is installed on a printer.

| Profile | Exact records | Look-ahead only | Scanner only | Both |
| --- | ---: | ---: | ---: | ---: |
| Shaped extrusion, 40,000 moves, final wait only | 186,972 | +4.21% / +2.53% | −8.16% / −8.06% | −6.05% / −2.96% |
| Layered, 12,000 moves, wait every 1,024 | 60,602 | −0.47% / −8.44% | −3.38% / −1.05% | +0.30% / −0.23% |

With the scanner alone, the shaped fixture was faster in 12/12 and
9/12 blocks; both scanner arms recognized all 40,000 moves. Layered
coverage was 11,727/12,000 in both scanner arms, but scanner-only
elapsed wins fell from 11/12 to 6/12 across repeats. No dependable
layered acceleration or look-ahead-only acceleration is established.
A separate prototype that unrolled the Python parameter loop was not
adopted: whole-process elapsed changed by −0.69% layered and +1.55%
shaped relative to the existing scanner, despite a dispatch-only gain.

### Expanded whole-process comparison

Each cell is the median elapsed change in 15 alternating stock/bridge
pairs, excluding one initial warmup pair. Negative means faster.
All moves were planned by the bridge with zero fallbacks and all
three or four motors were verified. The 40,000-move XYZ repeat
changed sign relative to the first set (+1.51% vs −2.80%).
Do not infer a stable gain or a fixed startup cost from this variation.

| Profile | Moves | Middle waits | Exact timed records | Median elapsed (bridge minus stock) |
| --- | ---: | ---: | ---: | ---: |
| XYZ | 1,200 | every 80 | 4,299 | +3.57% (+6.142 ms) |
| XYZ | 12,000 | every 80 | 43,033 | +6.47% (+21.265 ms) |
| XYZ | 40,000 | every 80 | 143,538 | −2.80% (−21.198 ms) |
| Shaped extrusion | 1,200 | every 80 | 5,908 | +1.95% (+3.714 ms) |
| Shaped extrusion | 12,000 | every 80 | 57,245 | +2.16% (+9.960 ms) |
| Shaped extrusion | 40,000 | every 80 | 190,390 | −2.20% (−26.997 ms) |
| Shaped extrusion | 40,000 | none | 186,972 | −3.42% (−46.607 ms) |

These runs compare stock against the published B94 bridge build,
not a new optimized build. A reusable junction-scratch experiment
reduced the isolated flush median by less than 1% on two cores,
but the process results varied in direction. It was not carried
forward. Python profiling is diagnostic only: in the long XYZ fixture the step generation
flush and G-code dispatch both consume more time than the Rust
replacement seam covers. The benchmark writes a real debug file,
but cannot measure printer I/O or physical print duration.

A fresh whole-process comparison with the unchanged bridge on 40,000
shaped extrusion moves and no intermediate waits verified all 186,972
timed motor records in two 15-pair runs. Paired median elapsed changed
by −0.13% and +1.57% (bridge versus stock). An instrumented diagnostic
showed motion step generation/flush and Python G-code dispatch both
cost more than look-ahead flush alone; instrumented timings are not
used as benchmark results. No total-host speedup is claimed for the
look-ahead-only bridge. The separate opt-in dispatch experiment above
is limited to the tested dense debug-file workload.

### Experiments kept out of the default path

The exact uniform-stride compression experiment was implemented in C
and Rust against their respective earlier versions. The isolated kernel
improved by 4.10% in C and 7.32% in Rust on the corpus containing a
65,535-step uniform run; excluding that run left changes of only 1.49%
and 1.39%. In full drained stock debug jobs the C variant improved
elapsed time by 1.45–2.06%; the full Rust CLI ranged from 0.7% faster
to 2.2% slower. Output matched throughout, but the gains were too small
or inconsistent to adopt.

A second experiment skipped input-shaper neighbor traversal when all
pulses were in one move. C and Rust reproduced their original outputs,
but whole-process elapsed increased by 0.90–2.65% in C and 0.25–1.89%
in Rust. Neither implementation was adopted. These are measured
software workloads, not printer benchmarks.

The C and Rust iterative step solvers were also prototyped with the same
residual-cache transformation: reuse both secant distances while the
target is unchanged and recompute from the original positions immediately
on a direction change or confirmed crossing. Original and modified C,
and modified Rust, matched 34,805 crossings, 64,048 commit markers, and
5,000 reported positions. The stock C comparison used only a shadow
helper loaded into pinned Klippy; both versions drained actual debug
file output. The Rust comparison was between full release CLIs and
included byte-identical JSON output.

| Candidate vs its original | Layered elapsed | Shaped extrusion elapsed |
| --- | ---: | ---: |
| Shadow C, 24 pairs per repeat | −0.42% / −1.23% | −3.01% / −4.38% |
| Rust CLI, 80 pairs per repeat | −0.17% / +0.66% | −0.02% / −0.58% |

Each repeat kept both arms on one CPU and alternated order. The C
workloads matched 60,340 and 74,811 timed records; the Rust workloads
matched 14,935 and 16,576. The modest, fixture-dependent changes did not
justify changing either solver. No live performance claim follows.

## Verified scope

- Configuration and motion: bounded Cartesians/CoreXY, stock move
  look-ahead, a bounded G-code subset, input shaping, pressure advance,
  runtime motion limits (`M204` and `SET_VELOCITY_LIMIT`), and fixed
  saved bed meshes in the offline CLI. The mesh-script opt-in loads or
  clears one allowed profile without flushing: 8,804 XYZ records
  matched stock. Combined with a single modeled positive fixed-window
  extruder, 4,640 XYZ/E records matched stock across reloads and gain
  changes. With three virtual shaped lanes and named pressure updates,
  the same feature matched 9,934 timed XYZ/E records. Scripted X/Y lookup
  offsets matched 7,614 XYZ and 9,920 shaped sparse XYZ/E records;
  following ZFADE comparisons matched 8,638 XYZ and 9,866 shaped sparse
  XYZ/E records. A motion-only
  job can use distinct extruder lanes for sparse physical tool IDs.
  With explicit opt-in, positive pressure-advance gain updates preserve
  the stock queue boundary and configured smoothing independently for
  each selected lane. The single-extruder case matched 3,792 timed
  records; the shaped sparse T0/T2/T6 case matched 10,312 XYZ/E records
  across two jobs. Explicit backing extruder names can also route
  updates to inactive virtual lanes at the global queue boundary;
  the stock mux comparator matched 9,889 records. No live calibration,
  dynamic smoothing or docking is supported by this option.
- Long offline motion: the separate Cargo feature and explicit
  `--long-step-preview` flag use the pinned C queue horizon and bounded
  extension/trim policy. The ordinary build retains the short event cap.
  Stock matched 257,111 layered and 115,855 shaped extrusion records at
  3,000 moves each, and 304,753 records in a three-tool trace with named
  pressure updates. A 43-second dwell matched; intervals not representable
  in the offline 32-bit format are rejected instead of wrapping. In the
  long mode, scripted saved-mesh load/clear and X/Y/ZFADE offsets matched
  174,305 XYZ, 176,641 single-extruder XYZ/E, and 309,466 three-tool
  records. The build supports no more than six sequential lanes, six
  million total raw events, and 1,572,864 retained output records. It
  excludes static profile-only mesh, live probing, homing/calibration,
  saving a profile, physical tooling, parallel workers, mixed heater
  plans, macro expansion, and virtual SD. See [README](README.md#opt-in-longer-offline-traces).
- Tool exchange: the inert program separates outbound and inbound
  extrusion, verifies supplied pickup evidence and validates the
  staged coordinates before exposing a trace. The modern stock Klippy
  compatibility fixture loads the reviewed Toolchanger PR #174
  candidate, not a merged upstream release.
- Shared extrusion binding: a bounded offline state model for two
  logical queues and one physical-stepper identity matches pinned
  Klippy's independent activation, positive pressure gain status,
  detach and re-anchor behavior. Two fixtures matched 14 Rust/stock
  snapshots, including the signed integer step position at both
  positive and negative rebinds and unchanged physical position across
  a detached move. Each model is fed committed planner positions and
  a caller-supplied flushed commanded position. The 429 timed XYZ/E
  records are a repeatable stock-only reference for future timing
  work, not yet Rust-generated parity. No preview CLI option, queue
  scheduling, shaper dynamics, hardware, or third-party compatibility is
  implied by this state model. Separately, a flushed-window E timing
  kernel with caller-supplied prepared phases, effective gain times,
  and queue switches matches unchanged stock C for 71 synthetic
  scenarios: 133,188 raw crossings, 269,845 commit markers, and 67,653
  direction/compressed-step records at 20 MHz. Flushes preserve one
  stepper's direction and clock across rebind/detach and roll back on
  an invalid or over-cap window. This is not full configured
  Klippy-to-Rust XYZ/E time parity; no CLI path calls this kernel.
  Actual pinned stock debug-file tests cover two relative-E jobs and an
  absolute-E job with G92, flow overrides (120% and 90%), a relative/absolute
  coordinate transition, queue switches, and detach/rejoin. From the same
  explicit virtual start and geometry, the independent Rust planner matches
  38 E and 24 XYZ scheduling events per job (excluding duplicate/no-time
  flushes). The four OIDs match at 50 MHz: 65,699 individual step events per
  compression mode, 23,168 exact compressed/direction records with zero
  error and 641 with stock's default 25 μs tolerance. The shared E
  stream also matches unchanged C replay. The typed offline
  `shared_debug_plan::trace` call directly renders the four complete
  lanes from G-code without a phase-capture or text adapter; the same
  reference test verifies that path record-for-record. Its bounded
  no-pacing debug planner requires a final M400 and fixed positive
  smoothing; it is not a live scheduler or physical nozzle-changer
  support. The separate optional `klipper-shared-preview` executable
  exposes this typed path as four JSON streams from a strict model file
  and a bounded G-code file; it has no printer or hardware path. A fourth
  stock-backed job exercises waits before motion, consecutive waits and
  waits after XY/E and Z moves. All four lanes match at both tolerances
  (21,738 physical steps per mode). The intermediate waits drain look-ahead
  without pacing or advancing the step-generation boundary in file mode.
  A fifth job checks leading, fractional mid-job, zero and trailing `G4`
  dwells with all 21,738 steps matching on all four lanes at both tolerances.
  Dwell changes simulated timestamps only and does not block the preview.
  A sixth job interleaves both M204 forms and SET_VELOCITY_LIMIT with
  queued XYZ/E and pressure changes. All 21,730 steps match on the
  four lanes at both tolerances without an extra generation boundary.
  A seventh job includes slicer display metadata before, during and after
  motion. It leaves all 21,738 stock steps unchanged on all four lanes
  at both tolerances. The preview validates those lines but does not
  maintain a display state.
  The eighth and ninth jobs verify saved coordinate state, including
  relative mode and feed restoration and an explicit XYZ return move.
  Stock, Rust and C E replay match at both tolerances: 21,738 and
  21,874 steps over all four lanes, respectively.
  A tenth job verifies absolute XYZ offsets, a Z adjustment with a
  simulated move, and clearing the offsets without a move. Across the
  four lanes, 21,798 steps match stock at both tolerances.
  A separate 216-additional-move fixture uses timestamps recorded from
  stock generation and transmit cutoffs. The opt-in `capture-replay`
  encoder matches stock's complete records on four lanes at both
  tolerances (29,992 steps per mode); pinned C independently matches the
  captured E lane. This is capture-driven replay, not support for
  independently scheduling longer scripts. The public preview's
  128-line limit is unchanged.
- Transport: frame/identify/clock code and bounded stock Linux PTY
  experiments; software-only motion with deterministic clocks and
  fault injection, including two independent peers. A fragmented
  outbound identify frame with a flipped CRC bit triggered replay in
  both the pinned stock C-backed host and Rust on separate disposable
  Linux MCU PTYs. Each host then matched all 4,096 and 1,024 read-only
  replies at two wave widths, including sequence wraps. A dropped
  application reply still fails closed even after every frame was
  acknowledged. No SocketCAN or real printer sessions are used.
- Slicer display metadata: bounded `M73`, `M117` and `SET_DISPLAY_TEXT`
  are inert in motion previews. The read-only display fixture matches the pinned
  stock progress and message behavior without a real display or UI.
- State: virtual job/SD/control boundaries, read-only status snapshots
  and connection-pair tests. Pinned Moonraker client components
  additionally verify the read-only status lifecycle, shutdown and
  reconnection through a socket pair. Pinned Moonraker also serves
  authenticated read-only HTTP/JSON-RPC and WebSocket status over
  an ephemeral loopback port with the Klippy dial injected. Named
  Klippy socket dialing, command execution and a real frontend
  installation are not verified. The same loopback fixture can stream
  a real simulated Rust program's read-only proof, tool, fan demand and
  temperature gate; the driver supplies only synthetic observations
  and exposes no control endpoint. The loopback lifecycle also follows
  the second phase through completion: a receipt carrying an old phase
  generation cannot change read-only HTTP or WebSocket job status;
  the right generation completes it, and reconnection starts fresh.
  The optional status projection also matches 17,784 pinned
  `virtual_sdcard` snapshots (including byte offsets for UTF-8 input)
  and 12,480 pinned `print_stats` snapshots through Rust WebHooks queries.
  The existing socket-pair/loopback fixture confirmed projected HTTP
  snapshots and WebSocket diffs for pending pause, completion,
  cancel, reset and reconnect, while G-code remains unavailable. The
  file path is caller-supplied metadata and is never opened. A composed
  read-only display fixture matched ten pinned DisplayStatus and VirtualSD
  observations: an M73 override survives an in-flight pause, then falls
  back to committed UTF-8 byte progress once idle and expired.
  Empty and null display messages remain distinguishable.
  The fixture provides read-only objects for the fan configuration and speed used
  by frontend subscribers, with 0% on startup, 40% when virtual
  demand changes, and 0% after a fresh peer reconnect. Tachometer RPM
  is unavailable; no frontend UI or fan hardware is exercised. The
  mixed preview provides separate
  motion queues around typed heater boundaries and never infers that
  a real heater is ready. The GPIO-free firmware exercise additionally
  checks that both temperature evidence and prior motion completion
  are required before the next phase. A separate static multi-tool
  preflight maps sparse virtual selectors to explicit heater roles;
  the offline CLI exposes this with explicit startup tool and temperatures.
  The signed final positions of all six lanes in 41 shaped and
  pressure-advanced jobs matched stock at deliberate stops. The
  GPIO-free T2/T5/T2 path then verified 13+13+13 commands on the
  intended lanes, while the synthetic shutdown released 13+0+0.
  Two software peers also preserve cumulative positions and block
  the final 13 commands if the second peer's receipt is absent.
  Thermal waits consume only explicit simulation observations.
  Stock print-fan M106/M107 can produce read-only mixed-tool demand
  stages with an explicit fan configuration; this does not schedule
  PWM or imply measured RPM. No endpoints operate a real heater, probe,
  motor or tool dock.

## Remaining integration work

A deployable host still needs a long-running runtime and independently
verified real transport scheduling, pin/peripheral integration, clock
fault and emergency-stop behavior, broader macro/plugin compatibility
and end-to-end Moonraker interoperability. Named Unix socket dialing
has not been verified; the managed test runner rejected opening that
socket, and all positive Moonraker results still use the explicitly
injected local socket pair. Actual printer timing,
power/heat shutdown and mechanical docking need separate hardware
validation. Do not use the offline fixtures as installation or
calibration instructions.
