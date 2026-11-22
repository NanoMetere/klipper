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
| Shaped CoreXY + pressure advance, 6,835 exact records | Stock Python+C 7.789 ms → Rust 4.750 ms median CPU; repeat 7.699 → 4.772 ms | 38–39% less CPU for the offline config-to-step kernel |
| Klippy move look-ahead, 25,836 moves including Python/Rust conversions | Stock 48.309 → bridge 44.712 ms and stock 46.044 → bridge 42.003 ms median CPU | 7–9% less CPU in the captured queue flush seam |
| Whole pinned Klippy process, 12,000 moves, 43,033 identical motor records | Paired elapsed changes +1.48% and +1.28% in two runs | No end-to-end speedup demonstrated from the optional bridge |
| Offline Rust CLI, 2,500 CoreXY moves | Prior Rust CLI 35.136 → 7.100 ms median elapsed | About 80% less elapsed time after streaming identical JSON to a file |
| Offline Rust CLI, three tools, 2,500 moves, 21,683 exact records | Same motion kernel, prior JSON 67.097 → 29.173 ms; separately, one lane worker 29.213 → four workers 16.201 ms (repeat 32.366 → 20.667 ms) | Serialization and concurrency were measured in independent A/B tests |
| Two GPIO-free software MCUs | 168 → 24 frames, 3.464 → 0.533 ms median enqueue through last software ACK | Same order, same five motor counters; this is Rust before/after framing, not physical transport or stock Klippy |

All figures use the checked-in test fixtures, the same host for both
sides within a run, and no printing hardware. The platform is Linux
x86-64 (AMD EPYC 9V74), not a Raspberry Pi. Use the runnable commands in
[README](README.md) to reproduce the individual measurements.

## Verified scope

- Configuration and motion: bounded Cartesians/CoreXY, stock move
  look-ahead, a limited static G-code subset, input shaping, pressure
  advance, and fixed saved bed meshes in the offline CLI. A motion-only
  job can use distinct extruder lanes for sparse physical tool IDs.
- Tool exchange: the inert program separates outbound and inbound
  extrusion, verifies supplied pickup evidence and validates the
  staged coordinates before exposing a trace. The modern stock Klippy
  compatibility fixture loads the reviewed Toolchanger PR #174
  candidate, not a merged upstream release.
- Transport: frame/identify/clock code and bounded stock Linux PTY
  experiments; software-only motion with deterministic clocks and
  fault injection, including two independent peers. No SocketCAN or
  real printer sessions are used.
- State: virtual job/SD/control boundaries, read-only status snapshots
  and connection-pair tests. The mixed preview provides separate
  motion queues around typed heater boundaries and never infers that
  a real heater is ready. Thermal waits consume only explicit
  simulation observations. No endpoints operate a real heater, probe,
  motor or tool dock.

## Remaining integration work

A deployable host still needs a long-running runtime and independently
verified real transport scheduling, pin/peripheral integration, clock
fault and emergency-stop behavior, broader macro/plugin compatibility
and end-to-end Moonraker interoperability. Actual printer timing,
power/heat shutdown and mechanical docking need separate hardware
validation. Do not use the offline fixtures as installation or
calibration instructions.
