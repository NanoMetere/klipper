#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Benchmark optional Rust paths inside the whole pinned Klippy debug process."""
import argparse
import hashlib
import json
import math
import os
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


def layered():
    """Synthetic slicer-style walls, infill, retractions and travels."""
    for layer in range(240):  # More than the harness needs at its 40k-move cap.
        z = 1.2 + layer * .2
        yield f"; LAYER {layer} · perimeter and infill", False
        yield f"G1 Z{z:.3f} F900 ; layer change", True
        yield "G1 E-.65 F1500 ; retract", True
        yield "G0 X12 Y12 F6000 ; travel to wall", True
        yield "G1 E.65 F1500", True
        now = (12., 12.)
        for n, end in enumerate(((28., 12.), (28., 28.), (12., 28.), (12., 12.))):
            start = now
            for j in range(1, 33):
                point = tuple(a + (b - a) * j / 32 for a, b in zip(start, end, strict=True))
                e = math.dist(now, point) * (.033 + n * .001)
                comment = " ; perimeter" if j <= 20 else ""
                yield f"G1 X{point[0]:.3f} Y{point[1]:.3f} E{e:.6f} F2100{comment}", True
                now = point
        yield "G1 E-.65 F1500", True
        yield "G0 X13 Y13 F6000", True
        yield "G1 E.65 F1500 ; unretract", True
        now = (13., 13.)
        for j in range(19):
            y = 13. + j * .75
            x = now[0]
            if j:
                yield f"G0 X{x:.3f} Y{y:.3f} F6000", True
                now = x, y
            end = 27. if j % 2 == 0 else 13.
            for k in range(1, 9):
                target = x + (end - x) * k / 8
                e = abs(target - now[0]) * (.026 + .001 * (k % 3))
                # Compact lowercase lines exercise the stock fallback.
                if k == 8 and j % 3 == 0:
                    yield f"g1x{target:.3f}y{y:.3f}e{e:.6f}f2700", True
                else:
                    comment = " ; infill" if k % 3 else ""
                    yield f"G1 X{target:.3f} Y{y:.3f} E{e:.6f} F2700{comment}", True
                now = target, y


def gcode(count, profile="xyz", wait_interval=80):
    yield "SET_KINEMATIC_POSITION X=10 Y=10 Z=1"
    yield "G90"
    if profile != "xyz":
        yield "M83"
    if profile == "layered":
        n = 0
        for line, moves in layered():
            yield line
            n += moves
            if moves and wait_interval and n % wait_interval == 0:
                yield "M400"
            if n == count:
                break
        assert n == count
        yield "M400"
        return
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
    p.add_argument("--rounds", type=int,
                   help="timed blocks (defaults to 9, or 12 in factorial mode)")
    p.add_argument("--moves", type=int, default=12000)
    p.add_argument("--profile", choices=("xyz", "shaped-extrusion", "layered"), default="xyz")
    p.add_argument("--wait-interval", type=int, default=80,
                   help="moves between explicit M400 waits (0 disables intermediate waits)")
    p.add_argument("--baseline-library", type=Path,
                   help="compare against a prior Rust bridge using the same Python adapter")
    p.add_argument("--rust-dispatch", action="store_true",
                   help="enable the optional dispatch scanner for the candidate only")
    p.add_argument("--factorial", action="store_true",
                   help="compare stock, look-ahead only, scanner only, and both with identical draining")
    p.add_argument("--json", type=Path, help="write raw sample pairs and verified coverage")
    a = p.parse_args()
    if a.rounds is None:
        a.rounds = 12 if a.factorial else 9
    if a.factorial and a.rounds % 4:
        p.error("--factorial needs a multiple of four rounds for balanced order")
    if not 1 <= a.rounds <= 40 or not 1200 <= a.moves <= 40000:
        p.error("--rounds must be 1–40 and --moves must be 1,200–40,000")
    if a.factorial and (a.baseline_library or a.rust_dispatch):
        p.error("--factorial sets the four arms; do not combine with --baseline-library or --rust-dispatch")
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
    # A tuple binds the path and both independent code-path switches.
    arms = ({"stock": (None, False, False),
             "look-ahead only": (library, True, False),
             "scanner only": (library, False, True),
             "both": (library, True, True)}
            if a.factorial else
            {"prior bridge" if baseline else "stock": (baseline, bool(baseline), False),
             "adapter": (library, True, a.rust_dispatch)})
    hashes = {label: hashlib.sha256(path.read_bytes()).hexdigest()
              for label, (path, _, _) in arms.items() if path}
    times = {label: [] for label in arms}
    coverage = {label: [] for label, (_, planner_on, _) in arms.items() if planner_on}
    scanner_coverage = {label: [] for label, (_, _, scanner_on) in arms.items() if scanner_on}
    drain_output = a.rust_dispatch or a.factorial
    print(f"{a.profile}: {a.moves} moves; intermediate wait interval {a.wait_interval}")
    baseline_label = next(iter(arms))
    labels = tuple(arms)
    with tempfile.TemporaryDirectory(prefix="klippy-process-time-") as d:
        root = Path(d)
        cfg, script = root / "printer.cfg", root / "input.gcode"
        cfg.write_text(CFG + (SHAPED_EXTRUSION_CFG if a.profile != "xyz" else ""))
        script.write_text("\n".join(gcode(a.moves, a.profile, a.wait_interval)) + "\n")
        script_digest = hashlib.sha256(script.read_bytes()).hexdigest()
        reference = None
        for r in range(a.rounds + 1):
            if a.factorial:
                # Each group of four blocks balances position and immediate
                # predecessor; warmup is not counted in timed blocks.
                order = [labels[(idx + (r - 1)) % 4] for idx in (0, 1, 3, 2)]
            else:
                order = labels[::(1 if r % 2 == 0 else -1)]
            for label in order:
                lib, planner_on, scanner_on = arms[label]
                output, log = root / f"{label}.out", root / f"{label}.log"
                cpu, wall = child_cpu(), time.perf_counter()
                flags = ["--drain-debug-output"] if drain_output else []
                if scanner_on:
                    flags.append("--rust-dispatch")
                    if not planner_on:
                        flags.append("--scanner-only")
                proc = subprocess.run(
                    [sys.executable, entry, *flags, klippy, lib or "--stock-only",
                     cfg, "-i", script, "-o", output, "-d", dictionary, "-l", log],
                    capture_output=True, text=True, timeout=120,
                )
                elapsed = (child_cpu() - cpu, time.perf_counter() - wall)
                if proc.returncode:
                    raise RuntimeError(proc.stdout + proc.stderr + log.read_text()[-5000:])
                if drain_output:
                    drain = next(line for line in proc.stdout.splitlines() if line.startswith("FILE_OUTPUT_DRAIN "))
                    assert json.loads(drain.split(" ", 1)[1]) == [str(output)], drain
                got = motor_trace(upstream, dictionary, output.read_bytes())
                if reference is None:
                    reference = got
                assert reference.keys() == got.keys(), (label, r, got.keys())
                for key, commands in got.items():
                    assert commands == reference[key], (label, r, key, len(commands), len(reference[key]))
                assert ("RUST_LOOKAHEAD_STATS " in proc.stdout) == planner_on, (label, proc.stdout)
                assert ("RUST_DISPATCH_STATS " in proc.stdout) == scanner_on, (label, proc.stdout)
                if planner_on:
                    row = next(line for line in proc.stdout.splitlines() if line.startswith("RUST_LOOKAHEAD_STATS "))
                    stats = json.loads(row.split(" ", 1)[1])
                    assert stats["planned"] == a.moves and stats["fallbacks"] == 0, stats
                    if r:
                        coverage[label].append(stats)
                if scanner_on:
                    row = next(line for line in proc.stdout.splitlines() if line.startswith("RUST_DISPATCH_STATS "))
                    stats = json.loads(row.split(" ", 1)[1])
                    if a.profile == "layered":
                        assert 0 < stats["canonical"] <= a.moves and stats["stock"] > 0, stats
                    else:
                        assert stats["canonical"] == a.moves and stats["stock"] > 0, stats
                    if r:
                        scanner_coverage[label].append(stats)
                if r:
                    times[label].append(elapsed)
            print(f"round {r}: all {sum(map(len, reference.values()))} motor records match", flush=True)
        # One OID per configured motor; require all of them to contribute.
        lanes = 4 if a.profile != "xyz" else 3
        assert len(reference) == lanes and all(len(rows) > 1 for rows in reference.values())
        motor_records = sum(map(len, reference.values()))
        for label, values in times.items():
            cpu = [round(1000 * value[0], 3) for value in values]
            wall = [round(1000 * value[1], 3) for value in values]
            print(f"{label}: median CPU {statistics.median(cpu):.3f} ms {cpu}; "
                  f"median elapsed {statistics.median(wall):.3f} ms {wall}")
        contrasts = ([(baseline_label, arm) for arm in labels[1:]]
                     + [("look-ahead only", "both"), ("scanner only", "both")]
                     if a.factorial else [(baseline_label, "adapter")])
        for old_label, new_label in contrasts:
            for column, kind in [(0, "CPU"), (1, "elapsed")]:
                ratios = [new[column] / old[column]
                          for old, new in zip(times[old_label], times[new_label], strict=True)]
                deltas = [new[column] - old[column]
                          for old, new in zip(times[old_label], times[new_label], strict=True)]
                prefix = f"{new_label} vs {old_label}: " if a.factorial else ""
                print(f"{prefix}{kind} paired median change {(statistics.median(ratios)-1)*100:+.2f}% "
                      f"({statistics.median(deltas)*1000:+.3f} ms; "
                      f"{sum(r < 1 for r in ratios)}/{len(ratios)} {new_label} wins)")
    for label, (path, _, _) in arms.items():
        if path and hashlib.sha256(path.read_bytes()).hexdigest() != hashes[label]:
            raise RuntimeError(f"{label} library changed during the benchmark")
    for label, scanned in scanner_coverage.items():
        prefix = f"{label}: " if a.factorial else ""
        print(f"{prefix}Rust scanner canonical moves: {min(s['canonical'] for s in scanned)}"
              f"–{max(s['canonical'] for s in scanned)} of {a.moves}")
    if a.json:
        report = {"stock_pin": pin, "profile": a.profile, "moves": a.moves,
                  "gcode_sha256": script_digest,
                  "wait_interval": a.wait_interval, "motor_count": lanes,
                  "exact_motor_records": motor_records, "bridge_coverage": coverage,
                  "dispatch_coverage": scanner_coverage if a.factorial else scanner_coverage.get("adapter", []),
                  "debug_output_drained": drain_output,
                  "library_sha256": hashes,
                  "sample_fields": ["process_cpu_seconds", "elapsed_seconds"], "samples": times}
        if a.factorial:
            if hasattr(os, "sched_getaffinity"):
                report["cpu_affinity"] = sorted(os.sched_getaffinity(0))
            report["rounds_per_arm"] = a.rounds
            report["arms"] = {label: {"rust_lookahead": planner_on, "rust_scanner": scanner_on}
                              for label, (_, planner_on, scanner_on) in arms.items()}
        a.json.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
