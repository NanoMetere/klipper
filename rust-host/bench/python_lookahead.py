#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify an opt-in Rust look-ahead kernel through actual Klippy Move objects."""
import argparse
from pathlib import Path
import statistics
import subprocess
import sys
import time

from lookahead import fixture, stock
from planner import PIN, ROOT


def worker(upstream, lib, mode):
    sys.path.insert(0, str(upstream / "klippy"))
    if lib != "-":
        sys.path.insert(0, str(ROOT / "python"))
        import toolhead
        from klipper_rust_lookahead import install
        stats = install(toolhead, lib)
    else:
        stats = None
    case = [(words[0], list(map(float, words[1:]))) for words in map(str.split, sys.stdin)]
    if mode == "verify":
        stock(case, True)
    else:
        stock(case, False)
        start = time.perf_counter_ns()
        count, total = stock(case, False)
        print(time.perf_counter_ns()-start, count, total)
    if stats is not None:
        assert stats["planned"] > 0 and stats["fallbacks"] == 0, stats


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=lambda x: Path(x).resolve(), help=f"checkout at {PIN}")
    p.add_argument("library", type=lambda x: Path(x).resolve())
    p.add_argument("--rounds", type=int, default=7)
    p.add_argument("--large", action="store_true", help="omit lazy flush requests to form larger batches")
    a = p.parse_args()
    cases = (r for r in fixture() if not a.large or r != ("f", 1))
    data = "\n".join(" ".join(map(str,row)) for row in cases)+"\n"

    def run(lib, mode):
        return subprocess.check_output([sys.executable, __file__, "--worker", a.upstream, lib, mode],
                                       input=data, text=True, timeout=30)

    want = run("-", "verify")
    actual = run(a.library, "verify")
    assert want == actual
    locations = [("-", "Python"), (a.library,"Python+Rust")]
    samples = {k: [] for _,k in locations}
    target = None
    for i in range(a.rounds):
        for lib, label in locations[::(-1 if i%2 else 1)]:
            elapsed, count, total = run(lib, "timed").split()
            if target is None:
                target = count, total
            assert (count,total) == target
            samples[label].append(int(elapsed)/int(count))
    print(f"{sum(l.startswith('move ') for l in want.splitlines())} exact profiles and flush decisions")
    for label, array in samples.items():
        if array:
            print(f"{label}: {statistics.median(array):.1f} ns/move median; {array}")


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "--worker":
        worker(Path(sys.argv[2]), sys.argv[3], sys.argv[4])
    else:
        main()
