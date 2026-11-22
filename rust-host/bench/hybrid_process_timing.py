#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Benchmark stock vs Rust look-ahead inside the whole Klippy debug process."""
import argparse
import json
from pathlib import Path
import resource
import statistics
import subprocess
import sys
import tempfile
import time

from hybrid_klippy import CFG, motor_trace
from planner import PIN, ROOT


def gcode(count):
    yield "SET_KINEMATIC_POSITION X=10 Y=10 Z=1"
    yield "G90"
    for i in range(count):
        x = 10 + (i % 100) * .12
        y = 15 + (i % 46) * .2
        z = 1 + (i % 3000 // 50) * .05
        yield f"G1 X{x:.5f} Y{y:.5f} Z{z:.5f} F{1800 + (i % 6) * 700}"
        if i % 80 == 79:
            yield "M400"
    yield "M400"


def child_cpu():
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    return usage.ru_utime + usage.ru_stime


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"stock checkout at {PIN}")
    p.add_argument("dictionary", type=Path)
    p.add_argument("library", type=Path)
    p.add_argument("--rounds", type=int, default=9)
    p.add_argument("--moves", type=int, default=12000)
    a = p.parse_args()
    if not 1 <= a.rounds <= 40 or not 1200 <= a.moves <= 40000:
        p.error("--rounds must be 1–40 and --moves must be 1,200–40,000")
    upstream, dictionary, library = [p.resolve() for p in (a.upstream, a.dictionary, a.library)]
    pin = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    if pin != PIN:
        p.error(f"stock checkout must be {PIN}")
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    klippy = upstream / "klippy/klippy.py"
    entry = ROOT / "bench/hybrid_klippy_entry.py"
    times = {"stock": [], "adapter": []}
    with tempfile.TemporaryDirectory(prefix="klippy-process-time-") as d:
        root = Path(d)
        cfg, script = root / "printer.cfg", root / "input.gcode"
        cfg.write_text(CFG)
        script.write_text("\n".join(gcode(a.moves)) + "\n")
        reference = None
        for r in range(a.rounds + 1):
            for label in tuple(times)[::(1 if r % 2 == 0 else -1)]:
                output, log = root / f"{label}.out", root / f"{label}.log"
                cpu, wall = child_cpu(), time.perf_counter()
                proc = subprocess.run(
                    [sys.executable, entry, klippy, library if label == "adapter" else "--stock-only",
                     cfg, "-i", script, "-o", output, "-d", dictionary, "-l", log],
                    capture_output=True, text=True, timeout=120,
                )
                elapsed = (child_cpu() - cpu, time.perf_counter() - wall)
                if proc.returncode:
                    raise RuntimeError(proc.stdout + proc.stderr + log.read_text()[-5000:])
                got = motor_trace(upstream, dictionary, output.read_bytes())
                if reference is None:
                    reference = got
                assert reference.keys() == got.keys(), (label, r, got.keys())
                for key, commands in got.items():
                    assert commands == reference[key], (label, r, key, len(commands), len(reference[key]))
                if label == "adapter":
                    row = next(line for line in proc.stdout.splitlines() if line.startswith("RUST_LOOKAHEAD_STATS "))
                    stats = json.loads(row.split(" ", 1)[1])
                    assert stats["planned"] == a.moves and stats["fallbacks"] == 0, stats
                if r:
                    times[label].append(elapsed)
            print(f"round {r}: all {sum(map(len, reference.values()))} motor records match", flush=True)
        for label, values in times.items():
            cpu = [round(1000 * value[0], 3) for value in values]
            wall = [round(1000 * value[1], 3) for value in values]
            print(f"{label}: median CPU {statistics.median(cpu):.3f} ms {cpu}; "
                  f"median elapsed {statistics.median(wall):.3f} ms {wall}")
        for column, kind in [(0, "CPU"), (1, "elapsed")]:
            ratios = [new[column] / old[column]
                      for old, new in zip(times["stock"], times["adapter"], strict=True)]
            print(f"{kind} paired median change {(statistics.median(ratios)-1)*100:+.2f}% "
                  f"({sum(r < 1 for r in ratios)}/{len(ratios)} adapter wins)")


if __name__ == "__main__":
    main()
