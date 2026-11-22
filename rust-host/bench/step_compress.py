#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare Rust step clock blocks with pinned C compression over exact fixtures."""
import argparse
import os
from pathlib import Path
import random
import statistics
import subprocess
import tempfile

from planner import PIN, ROOT


def fixture():
    rng = random.Random(76521)
    for i in range(1800):
        base = rng.choice([0, 2**32 - 30, 2**32 + 130, 2**42 + 1])
        error = rng.choice([0, 0, 1, 6, 28, 60, 120, 225])
        length = rng.randrange(1, 240)
        interval = rng.randrange(10, 4000)
        slope = rng.choice([-12, -4, 0, 0, 0, 1, 13, 29])
        jitter = rng.choice([0, 0, 0, 1, 15, 30, 100])
        check = base
        clocks = []
        for j in range(length):
            check += max(1, interval + slope*j + rng.randint(-jitter, jitter))
            clocks.append(check)
        yield (base, error, length, *clocks)
    # Long, exact stationary-rate sequence crossing the 32-bit clock boundary.
    base = 2**32 - 100
    yield (base, 5, 65535, *(base + (i+1)*200 for i in range(65535)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument('--rounds', type=int, default=0)
    p.add_argument('--repeat', type=int, default=25)
    p.add_argument('--baseline-source', type=Path)
    args = p.parse_args()
    rows = list(fixture())
    data = "\n".join(" ".join(map(str, row)) for row in rows) + "\n"
    with tempfile.TemporaryDirectory() as d:
        t = Path(d)
        c, rs, lib = t / "stock", t / "rust", t / "libklipper_host_protocol.rlib"
        h = args.upstream.resolve() / "klippy/chelper"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-ffunction-sections", "-fdata-sections",
                        "-Wl,--gc-sections", "-I", h, ROOT / "bench/step_compress.c", h / "pyhelper.c",
                        "-o", c], check=True)
        def build_rust(source, binary, prior=False):
            subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                            "--crate-name=klipper_host_protocol", source, "-o", lib], check=True)
            subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/step_compress.rs",
                            "--extern", f"klipper_host_protocol={lib}", "-o", binary, *(["--cfg", "previous_allocator"] if prior else [])], check=True)

        build_rust(ROOT / "src/step_compress.rs", rs)
        programs = {"stock": c, "rust": rs}
        if args.baseline_source:
            prior = t / "previous"
            build_rust(args.baseline_source, prior, prior=True)
            programs["before"] = prior
        a, b = [subprocess.check_output([exe], input=data, text=True).splitlines() for exe in (c, rs)]
        if args.baseline_source:
            assert subprocess.check_output([prior], input=data, text=True).splitlines() == a
        for i, (x, y) in enumerate(zip(a, b, strict=True)):
            assert x == y, (i, x, y)
        steps = sum(row[2] for row in rows)
        print(f"{len(rows)} runs; {steps} clocks; {len(a)-len(rows)} exact blocks")
        counts = {}
        checksums = set()
        for i in range(args.rounds):
            order = list(programs.items())
            order = order[i % len(order):] + order[:i % len(order)]
            for label, cmd in order:
                timing = subprocess.check_output([cmd, str(args.repeat)], input=data, text=True)
                elapsed, checksum = map(int, timing.split())
                counts.setdefault(label, []).append(elapsed / (steps * args.repeat))
                checksums.add(checksum)
        assert len(checksums) <= 1
        if args.rounds:
            for label, samples in counts.items():
                print(f"{label}: {statistics.median(samples):.3f} ns/step median, "
                      f"{min(samples):.3f}–{max(samples):.3f} range; {samples}")


if __name__ == "__main__":
    main()
