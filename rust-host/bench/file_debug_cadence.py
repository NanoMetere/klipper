#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare independent bounded file-debug cadence implementations, not live motion."""
import argparse
import random
from pathlib import Path
import statistics
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rounds", type=int, default=0)
    args = parser.parse_args()
    if not 0 <= args.rounds <= 100:
        parser.error("--rounds must be 0–100")
    bench = Path(__file__).resolve().parent
    # Include the fast-forward path, equal-boundary, existing wire lead and
    # exhausted-activity cases as well as seeded bounded timing states.
    states = [(0, 0, 0), (2.5, 1, 1), (110, 2, 1.96), (10, 15, 14.95)]
    rng = random.Random(20261125)
    for _ in range(3000):
        gen = rng.randrange(0, 118000)
        states.append((rng.randrange(0, 119000) / 1000, gen / 1000,
                       rng.randrange(gen + 1) / 1000))
    raw = "".join(" ".join(map(str, vals)) + "\n" for vals in states)
    with tempfile.TemporaryDirectory(prefix="klipper-cadence-shadow-") as directory:
        cpp, rust = [Path(directory) / s for s in ("shadow-c", "rust")]
        subprocess.run(["cc", "-O3", "-ffp-contract=off", bench / "file_debug_cadence.c",
                        "-lm", "-o", cpp], check=True)
        subprocess.run(["rustc", "--edition", "2021", "-O", bench / "file_debug_cadence.rs",
                        "-o", rust], check=True)
        values = []
        for exe in (cpp, rust):
            reference = subprocess.check_output([exe], input=raw, text=True, timeout=20)
            intervals = [tuple(line.split()) for line in reference.splitlines()]
            assert sum(a == "end" for a, _ in intervals) == len(states)
            values.append([(a if a == "end" else float(a), float(b)) for a, b in intervals])
            for invalid in ("nan 0 0\n", "0 -1 0\n", "0 1 2\n", "121 0 0\n"):
                p = subprocess.run([exe], input=invalid, text=True, capture_output=True, timeout=5)
                assert p.returncode == 2 and not p.stdout
        assert values[0] == values[1]
        print(f"{len(states)} states; {len(values[0]) - len(states)} exact transmit/generation boundaries")
        times = [[], []]
        for i in range(args.rounds):
            for j in ((0, 1) if i % 2 == 0 else (1, 0)):
                start = time.perf_counter_ns()
                subprocess.run([(cpp, rust)[j]], input=raw, text=True,
                               stdout=subprocess.DEVNULL, check=True, timeout=20)
                times[j].append(time.perf_counter_ns() - start)
        if args.rounds:
            print("Cadence CLI only; median elapsed ms C/Rust:",
                  *(round(statistics.median(x) / 1e6, 3) for x in times))


if __name__ == "__main__":
    main()
