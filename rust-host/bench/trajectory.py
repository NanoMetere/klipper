#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare Rust XYZ trapezoids and sample coordinates with pinned C trapq."""
import argparse
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile

from planner import PIN, ROOT


def corpus():
    rng = random.Random(20261124)
    times = [-2., 0.]
    yield ("p", 0.)
    end = 0.0
    pos = [0.] * 3
    for i in range(1700):
        gap = rng.choice([0., 0., .001, .34, 1.2, 3.]) if i else 10.0
        at = end + gap
        if gap:
            times.extend([end + .33 * gap, at - .1 * gap])
        phases = [rng.choice([0., 1e-7, .2, .1, .70, .95]) for _ in range(3)]
        raw = rng.choice([(1, 0, 0), (0, 0, -1), (1, 1, 0), (-1, 2, 4)])
        norm = math.sqrt(sum(v*v for v in raw))
        direction = [x/norm for x in raw]
        accel = rng.uniform(2., 130.)
        cruise = max(phases[0], phases[2]) * accel + rng.uniform(0.5, 70.0)
        start = cruise - phases[0] * accel
        yield ("a", at, *phases, *pos, *direction, start, cruise, accel)
        end = at
        for duration, speed, half_accel in zip(phases, (start, cruise, cruise), (.5*accel, 0., -.5*accel)):
            if duration:
                times.extend([end, end + duration / 2, end + duration])
                end += duration
                distance = (speed + half_accel * duration) * duration
                pos = [x + d * distance for x, d in zip(pos, direction)]
    yield ("d",)
    for t in times + [end + 1.]:
        yield ("p", t)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    args = p.parse_args()
    data = "\n".join(" ".join(map(str, row)) for row in corpus()) + "\n"
    with tempfile.TemporaryDirectory() as dir_:
        tmp = Path(dir_)
        c, rust, lib = tmp / "stock", tmp / "traj", tmp / "libklipper_host_protocol.rlib"
        helper = args.upstream.resolve() / "klippy/chelper"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", helper, ROOT / "bench/trajectory.c",
                        helper / "trapq.c", "-o", c], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/trajectory.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        want, got = [subprocess.check_output([cmd], input=data, text=True).splitlines() for cmd in [c, rust]]
        counts = {}
        max_abs = 0.
        for i, (a, b) in enumerate(zip(want, got, strict=True)):
            x, y = a.split(), b.split()
            assert x[0] == y[0], (i, a, b)
            counts[x[0]] = counts.get(x[0], 0) + 1
            for v, w in zip(map(float, x[1:]), map(float, y[1:]), strict=True):
                assert math.isclose(v, w, rel_tol=2e-12, abs_tol=2e-12), (i, a, b)
                max_abs = max(max_abs, abs(v-w))
        print(f"{counts}; maximum segment/coordinate absolute difference {max_abs:g}")


if __name__ == "__main__":
    main()
