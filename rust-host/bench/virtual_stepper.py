#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Independent per-step recurrence vs bounded block-clock replay."""
import argparse
import bisect
from pathlib import Path
import random
import subprocess

from planner import ROOT


def cases():
    rng = random.Random(67956)
    for i in range(48):
        base = start = 2**32 + rng.randrange(-150_000, 150_000)
        pos = rng.randrange(-100000, 100000)
        lines, clocks, positions, ends = [f"start {start} {pos}"], [], [pos], []
        for block in range(104):
            direction = rng.choice([1, -1])
            n = rng.randint(1, 65535 if block == 103 else 500)
            add = 0 if block == 103 else rng.choice([-3, -1, 0, 0, 1, 7])
            if block == 103 and i == 0:
                n = 65535
            delay = rng.randint(3, 700)
            if add < 0:
                delay += -add * (n - 1)
            lines += [f"dir {int(direction > 0)}", f"block {delay} {n} {add}"]
            for _ in range(n):
                base += delay
                pos += direction
                clocks.append(base)
                positions.append(pos)
                delay += add
            ends.append(base)
        ticks = [start, base, base+1]
        ticks += rng.sample(clocks, 300)
        ticks += [x-1 for x in rng.sample(clocks, 300)]
        ticks += rng.sample(range(start, base), 300)
        want = []
        for tick in sorted(ticks):
            lines.append(f"at {tick}")
            count = bisect.bisect_right(clocks, tick)
            pending = len(ends) - bisect.bisect_right(ends, tick)
            want.append(f"{tick} {positions[count]} {pending}")
        yield lines, want, len(clocks)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/virtual_stepper_replay")
    a = p.parse_args()
    lines, desired, edges = [], [], 0
    for case, want, count in cases():
        lines.extend(case)
        desired.extend(want)
        edges += count
    result = subprocess.check_output([a.binary], text=True, input="\n".join(lines)+"\n").splitlines()
    for i, (a, b) in enumerate(zip(desired, result, strict=True)):
        assert a == b, (i, a, b)
    print(f"{len(result)} exact edge-clock/position checks over {edges} independently expanded steps")


if __name__ == "__main__":
    main()
