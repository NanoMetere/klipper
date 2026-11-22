#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline step commands with stock C clock/filter/compress output."""
import argparse
import os
from pathlib import Path
import random
import subprocess
import tempfile

from planner import PIN, ROOT


def corpus():
    yield ("r", 0, 0, 1_000_000, 0, 0)
    for tick, direction in [(1000, 1), (1749, 0), (2000, 1), (2001, 0), (3000, 0), (3750, 1)]:
        if tick == 2001:
            yield ("c",)
        yield ("a", direction, 0, tick/1e6)
    yield ("f",)
    r = random.Random(72514)
    for i in range(640):
        base = r.choice([0, 2**31 - 5, 2**32 - 70, 2**36])
        hz = r.choice([1e6, 12e6, 48e6, 100e6])
        origin = r.choice([-9.4, 0, 471.625])
        yield ("r", base, origin, hz, r.choice([0, 1, 11, 30, 200]), r.randrange(2))
        at = origin + base/hz
        delay = .005
        direction = r.randrange(2)
        for k in range(r.randrange(5, 300)):
            if r.randrange(14) == 0:
                direction = 1 - direction
            if r.randrange(10) == 0:
                yield ("c",)
            delay += r.choice([.00007, .0001, .0003, .003, .000750, .000751])
            if k % 18 == 0:
                at += delay
                delay = 0.0
            yield ("a", direction, at, delay)
        yield ("f",)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    args = p.parse_args()
    data = "\n".join(" ".join(map(str, row)) for row in corpus()) + "\n"
    with tempfile.TemporaryDirectory() as dir_:
        d = Path(dir_)
        c, rust, lib = d / "stock", d / "rust", d / "libklipper_host_protocol.rlib"
        h = args.upstream.resolve() / "klippy/chelper"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", h, ROOT / "bench/step_clock.c",
                        h / "stepcompress.c", h / "msgblock.c", h / "pyhelper.c", "-lm", "-o", c], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/step_clock.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        want, got = [subprocess.check_output([exe], input=data, text=True).splitlines() for exe in (c, rust)]
        stats = {}
        for i, (a, b) in enumerate(zip(want, got, strict=True)):
            assert a == b, (i, a, b)
            key = a.split()[0]
            stats[key] = stats.get(key, 0) + 1
        print(f"Exact clock/filter/command output: {stats}")


if __name__ == "__main__":
    main()
