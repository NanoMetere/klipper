#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare raw crossings and commit markers with pinned stock C itersolve."""
import argparse
import os
from pathlib import Path
import random
import subprocess
import tempfile

from planner import PIN, ROOT


def corpus():
    r = random.Random(60007)
    for axis in ("x", "y", "z", "+", "-"):
        for _ in range(20):
            pos = [r.uniform(-10., 10.) for _ in range(3)]
            yield ("r", axis, r.choice([.00625, .01, .1]), *pos)
            at = 2.0
            for _ in range(25):
                duration = r.uniform(.001, .045)
                direction = r.choice([(1, 0, 0), (-1, 0, 0), (0, 1, 0),
                                      (0, 0, 1), (0, 0, -1), (.5, -.5, 0),
                                      (-.5, -.5, 0), (.5, 0, -.5)])
                half_a = r.uniform(-35., 75.)
                start_v = max(0., -2 * half_a * duration) + r.uniform(0., 30.)
                # Both full and separately flushed portions.
                mid = at + .4 * duration
                end = at + duration
                yield ("m", at, duration, *pos, *direction, start_v, half_a, at, mid)
                yield ("m", at, duration, *pos, *direction, start_v, half_a, mid, end)
                distance = (start_v + half_a * duration) * duration
                pos = [x + y * distance for x, y in zip(pos, direction)]
                at = end + r.choice([0., .0002, .02])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    args = parser.parse_args()
    data = "\n".join(" ".join(map(str, row)) for row in corpus()) + "\n"
    with tempfile.TemporaryDirectory() as d:
        t = Path(d)
        c, rs, lib = t / "stock", t / "rust", t / "libklipper_host_protocol.rlib"
        h = args.upstream.resolve() / "klippy/chelper"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-ffunction-sections", "-fdata-sections",
                        "-Wl,--gc-sections", "-I", h, ROOT / "bench/step_times.c", h / "trapq.c",
                        h / "kin_cartesian.c", h / "kin_corexy.c", "-o", c], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/step_times.rs", "--extern",
                        f"klipper_host_protocol={lib}", "-o", rs], check=True)
        a, b = [subprocess.check_output([p], input=data, text=True).splitlines() for p in (c, rs)]
        stats = {}
        for i, (x, y) in enumerate(zip(a, b, strict=True)):
            p, q = x.split(), y.split()
            assert p[0] == q[0] and list(map(float,p[1:])) == list(map(float,q[1:])), (i, x, y)
            stats[p[0]] = stats.get(p[0], 0) + 1
        print(f"Exact stock C/Rust outcomes: {stats}")


if __name__ == "__main__":
    main()
