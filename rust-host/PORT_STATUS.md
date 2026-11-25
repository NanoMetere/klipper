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
| Two GPIO-free software MCUs | 168 → 24 frames, 3.464 → 0.533 ms median enqueue through last software ACK | Same order, same five motor counters; this is Rust before/after framing, not physical transport or stock Klippy |
| Mixed virtual job across two software MCUs, 39 commands | 39 → 9 frames; host enqueue through ACK 0.967 → 0.278 ms; single-CPU repeat 0.232 → 0.062 ms | 71–73% less enqueue time in this fixture, identical command order and counters, not real print duration |
| Offline status polling, 8 subscriptions × 12,000 polls | Prior Rust 394.226 → 84.094 ms and 392.586 → 81.125 ms median CPU; 3,432 byte-identical frames | About 79% less CPU by avoiding copies of unchanged fields; synthetic sparse updates, not whole-host, Moonraker frontend or print speed |

All figures use the checked-in test fixtures, the same host for both
sides within a run, and no printing hardware. The platform is Linux
x86-64 (AMD EPYC 9V74), not a Raspberry Pi. Use the runnable commands in
[README](README.md) to reproduce the individual measurements.

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

## Verified scope

- Configuration and motion: bounded Cartesians/CoreXY, stock move
  look-ahead, a bounded G-code subset, input shaping, pressure advance,
  runtime motion limits (`M204` and `SET_VELOCITY_LIMIT`), and fixed
  saved bed meshes in the offline CLI. A motion-only
  job can use distinct extruder lanes for sparse physical tool IDs.
  With explicit opt-in, positive pressure-advance gain updates preserve
  the stock queue boundary and configured smoothing independently for
  each selected lane. The single-extruder case matched 3,792 timed
  records; the shaped sparse T0/T2/T6 case matched 10,312 XYZ/E records
  across two jobs. No live calibration, dynamic smoothing or docking
  is supported by this option.
- Tool exchange: the inert program separates outbound and inbound
  extrusion, verifies supplied pickup evidence and validates the
  staged coordinates before exposing a trace. The modern stock Klippy
  compatibility fixture loads the reviewed Toolchanger PR #174
  candidate, not a merged upstream release.
- Transport: frame/identify/clock code and bounded stock Linux PTY
  experiments; software-only motion with deterministic clocks and
  fault injection, including two independent peers. No SocketCAN or
  real printer sessions are used.
- Slicer display metadata: bounded `M73` and `M117` are inert in
  motion previews. The read-only display fixture matches the pinned
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
  and exposes no control endpoint. The fixture provides read-only
  objects for the fan configuration and speed used
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
and end-to-end Moonraker interoperability. Actual printer timing,
power/heat shutdown and mechanical docking need separate hardware
validation. Do not use the offline fixtures as installation or
calibration instructions.
