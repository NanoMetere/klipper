#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Benchmark stock vs Rust look-ahead inside the whole Klippy debug process."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import resource
import statistics
import subprocess
import sys
import tempfile
import time

from hybrid_klippy import CFG, motor_trace
from planner import PIN, ROOT


SHAPED_EXTRUSION_CFG = """
[extruder]
step_pin: gpio10
dir_pin: gpio11
heater_pin: gpio12
sensor_pin: analog0
sensor_type: EPCOS 100K B57560G104F
control: watermark
min_temp: 0
max_temp: 250
microsteps: 16
rotation_distance: 22.17
nozzle_diameter: 0.4
filament_diameter: 1.75
pressure_advance: 0.04

[input_shaper]
shaper_type_x: mzv
shaper_freq_x: 45
shaper_type_y: ei
shaper_freq_y: 39
"""


def gcode(count, profile="xyz", wait_interval=80):
    yield "SET_KINEMATIC_POSITION X=10 Y=10 Z=1"
    yield "G90"
    if profile == "shaped-extrusion":
        yield "M83"
    last = (10.0, 10.0, 1.0)
    for i in range(count):
        x = 10 + (i % 100) * .12
        y = 15 + (i % 46) * .2
        z = 1 + (i % 3000 // 50) * .05
        words = f"G1 X{x:.5f} Y{y:.5f} Z{z:.5f} F{1800 + (i % 6) * 700}"
        if profile == "shaped-extrusion":
            # Follow each XY move with proportional nominal extrusion.
            distance = math.dist(last, (x, y, z))
            last = (x, y, z)
            words += f" E{.035 * distance:.7f}"
        yield words
        if wait_interval and i % wait_interval == wait_interval - 1:
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
    p.add_argument("--profile", choices=("xyz", "shaped-extrusion"), default="xyz")
    p.add_argument("--wait-interval", type=int, default=80,
                   help="moves between explicit M400 waits (0 disables intermediate waits)")
    p.add_argument("--baseline-library", type=Path,
                   help="compare against a prior Rust bridge using the same Python adapter")
    p.add_argument("--json", type=Path, help="write raw sample pairs and verified coverage")
    a = p.parse_args()
    if not 1 <= a.rounds <= 40 or not 1200 <= a.moves <= 40000:
        p.error("--rounds must be 1–40 and --moves must be 1,200–40,000")
    if a.wait_interval < 0:
        p.error("--wait-interval must be non-negative")
    upstream, dictionary, library = [p.resolve() for p in (a.upstream, a.dictionary, a.library)]
    pin = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    if pin != PIN:
        p.error(f"stock checkout must be {PIN}")
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    klippy = upstream / "klippy/klippy.py"
    entry = ROOT / "bench/hybrid_klippy_entry.py"
    baseline = a.baseline_library.resolve() if a.baseline_library else None
    entries = {"prior bridge" if baseline else "stock": baseline, "adapter": library}
    hashes = {label: hashlib.sha256(path.read_bytes()).hexdigest()
              for label, path in entries.items() if path}
    times = {label: [] for label in entries}
    coverage = {label: [] for label in hashes}
    print(f"{a.profile}: {a.moves} moves; intermediate wait interval {a.wait_interval}")
    baseline_label = next(iter(entries))
    with tempfile.TemporaryDirectory(prefix="klippy-process-time-") as d:
        root = Path(d)
        cfg, script = root / "printer.cfg", root / "input.gcode"
        cfg.write_text(CFG + (SHAPED_EXTRUSION_CFG if a.profile == "shaped-extrusion" else ""))
        script.write_text("\n".join(gcode(a.moves, a.profile, a.wait_interval)) + "\n")
        reference = None
        for r in range(a.rounds + 1):
            for label in tuple(times)[::(1 if r % 2 == 0 else -1)]:
                output, log = root / f"{label}.out", root / f"{label}.log"
                cpu, wall = child_cpu(), time.perf_counter()
                proc = subprocess.run(
                    [sys.executable, entry, klippy, entries[label] or "--stock-only",
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
                if entries[label]:
                    row = next(line for line in proc.stdout.splitlines() if line.startswith("RUST_LOOKAHEAD_STATS "))
                    stats = json.loads(row.split(" ", 1)[1])
                    assert stats["planned"] == a.moves and stats["fallbacks"] == 0, stats
                    if r:
                        coverage[label].append(stats)
                if r:
                    times[label].append(elapsed)
            print(f"round {r}: all {sum(map(len, reference.values()))} motor records match", flush=True)
        # One OID per configured motor; require all of them to contribute.
        lanes = 4 if a.profile == "shaped-extrusion" else 3
        assert len(reference) == lanes and all(len(rows) > 1 for rows in reference.values())
        motor_records = sum(map(len, reference.values()))
        for label, values in times.items():
            cpu = [round(1000 * value[0], 3) for value in values]
            wall = [round(1000 * value[1], 3) for value in values]
            print(f"{label}: median CPU {statistics.median(cpu):.3f} ms {cpu}; "
                  f"median elapsed {statistics.median(wall):.3f} ms {wall}")
        for column, kind in [(0, "CPU"), (1, "elapsed")]:
            ratios = [new[column] / old[column]
                      for old, new in zip(times[baseline_label], times["adapter"], strict=True)]
            deltas = [new[column] - old[column]
                      for old, new in zip(times[baseline_label], times["adapter"], strict=True)]
            print(f"{kind} paired median change {(statistics.median(ratios)-1)*100:+.2f}% "
                  f"({statistics.median(deltas)*1000:+.3f} ms; "
                  f"{sum(r < 1 for r in ratios)}/{len(ratios)} adapter wins)")
    for label, path in entries.items():
        if path and hashlib.sha256(path.read_bytes()).hexdigest() != hashes[label]:
            raise RuntimeError(f"{label} library changed during the benchmark")
    if a.json:
        report = {"stock_pin": pin, "profile": a.profile, "moves": a.moves,
                  "wait_interval": a.wait_interval, "motor_count": lanes,
                  "exact_motor_records": motor_records, "bridge_coverage": coverage,
                  "library_sha256": hashes,
                  "sample_fields": ["process_cpu_seconds", "elapsed_seconds"], "samples": times}
        a.json.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
