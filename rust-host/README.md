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
misaligned chunks do not advance the exchange. Timeouts, retries, sequence
tracking, receive-stream recovery, and clock synchronization are not part of
this offline API yet. It does not interpret movement or heater commands.

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
