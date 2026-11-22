#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare nominal and pressure-advance positions with pinned Klippy C."""
import argparse
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile

from planner import PIN, ROOT


def corpus():
    rng = random.Random(1101)
    for index in range(24):
        yield ("r",)
        smooth = (.0, .0125, .05, .2)[index % 4]
        for t in (0., 4.09, 4.2, 4.63, 5.2, 8.2):
            yield ("g", t, rng.choice([0., .027, .045, .087, .132]), smooth)
        time = 4.
        bounds = [3.95]
        position = rng.uniform(-3., 3.)
        for _ in range(35):
            a,c,d = [rng.choice([.004, .018, .031, .2]) for _ in range(3)]
            base = rng.uniform(.5, 17.)
            accel = rng.uniform(.0, 120.)
            cruise = base + a*accel
            if d*accel >= cruise:
                d = 0.5*cruise/accel
            direction = rng.choice([-1, 1, 1, 1])
            enabled = int(direction == 1 and rng.randrange(4) != 0)
            yield ("m", time, a, c, d, position, direction*base, direction*cruise, direction*accel, enabled)
            bounds += [time, time + a, time + a + c, time + a + c + d]
            position += direction*((base+.5*a*accel)*a + cruise*c + (cruise-.5*d*accel)*d)
            time = time + a + c + d + rng.choice([1e-7, 0.01, .06])
        for t in bounds:
            for delta in [-.00001, 0, .00001]:
                yield ("p", t+delta)
        for _ in range(320):
            yield ("p", rng.uniform(3.95, time+.1))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    a = p.parse_args()
    h = a.upstream.resolve() / "klippy/chelper"
    data = "\n".join(" ".join(map(str, r)) for r in corpus())+"\n"
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        c, rust, lib = tmp / "stock", tmp / "rust", tmp / "libklipper_host_protocol.rlib"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", h, ROOT / "bench/pressure_advance.c",
                        h / "trapq.c", h / "kin_extruder.c", "-o", c], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/pressure_advance.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        want, got = [subprocess.check_output([program], text=True, input=data).splitlines() for program in (c, rust)]
        error = 0.
        for i, (x, y) in enumerate(zip(map(float,want), map(float,got), strict=True)):
            error = max(error, abs(x-y))
            assert math.isclose(x, y, abs_tol=4e-10, rel_tol=2e-12), (i, x, y)
        print(f"{len(want)} samples matched; max absolute difference {error:g} mm")


if __name__ == "__main__":
    main()
