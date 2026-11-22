#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare two inert job scheduler implementations across exact wire records."""
import argparse
from pathlib import Path
import statistics
import subprocess


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("baseline", type=Path)
    p.add_argument("candidate", type=Path)
    p.add_argument("--rounds", type=int, default=15)
    p.add_argument("--prepare", action="store_true", help="time job construction instead of polling")
    a = p.parse_args()
    if not 1 <= a.rounds <= 30:
        p.error("--rounds must be 1–30")
    programs = {"baseline": a.baseline.resolve(), "candidate": a.candidate.resolve()}
    for n in (3, 6, 11, 16):
        for variant in range(3):
            times = {label: [] for label in programs}
            expected = None
            for i in range(a.rounds + 1):
                for label in tuple(programs)[::(1 if i % 2 == 0 else -1)]:
                    raw = subprocess.check_output([programs[label], str(n), "1", str(variant),
                                                   *(["--prepare"] if a.prepare else [])], text=True)
                    ns, count, digest = map(int, raw.splitlines()[-1].split())
                    expected = expected or (count, digest)
                    assert (count, digest) == expected, (i, n, variant, label, expected, count, digest)
                    if i:
                        times[label].append(ns * 1e-6)
            before, after = (statistics.median(rows) for rows in times.values())
            print(f"{n} lanes, workload {variant}: {expected[0]} records "
                  f"baseline {before:.3f} ms, candidate {after:.3f} ms "
                  f"({(after/before-1)*100:+.1f}%)", flush=True)


if __name__ == "__main__":
    main()
