#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare canonical scanner code in the same offline stock Klippy adapter."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

from hybrid_process_timing import CFG, SHAPED_EXTRUSION_CFG, child_cpu, gcode, motor_trace
from planner import PIN, ROOT

ORDER = ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0))


def scanner(path):
    scan = ctypes.CDLL(str(path)).klipper_rs_move_scan_v1
    scan.argtypes = (ctypes.c_char_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t)
    scan.restype = ctypes.c_int
    return scan


def report(samples, metrics, contrasts):
    for base, candidate in contrasts:
        for i, metric in enumerate(metrics):
            ratios = [v[i] / b[i] for b, v in zip(samples[base], samples[candidate], strict=True)]
            delta = (statistics.median(ratios) - 1) * 100
            wins = sum(x < 1 for x in ratios)
            print(f"{candidate} vs {base} {metric}: {delta:+.2f}% paired, {wins}/{len(ratios)} wins")


def check_abi(funcs):
    # Empty rows, final separators, malformed data, and short/misaligned
    # output must behave identically. Sentinels catch out-of-range writes.
    for data, size, expected in (
        (b"", 16, 0),
        (b"G1 X1\n", 32, 0),
        (b"\nG1 X1 X2\nG1 E.4;\xc3\xa9", 48, 0),
        (b"G1 X1\n", 16, -1),
        (b"G1 X1", 32, -1),
        (b"G1 X1", 15, -1),
        (b"G1 X1", 0, -1),
    ):
        found = []
        for scan in funcs.values():
            raw = (ctypes.c_ubyte * (size + 2))(*([0xa5] * (size + 2)))
            out = ctypes.cast(ctypes.byref(raw, 1), ctypes.c_void_p)
            assert scan(data, len(data), out, size) == expected
            assert raw[0] == raw[-1] == 0xa5
            found.append(bytes(raw))
        assert found[0] == found[1]


def native(c, rust, lines, rounds):
    funcs = {"C": scanner(c), "Rust": scanner(rust)}
    check_abi(funcs)
    inputs = []
    for i in range(0, len(lines), 1536):
        batch = lines[i:i+1536]
        data = "\n".join(batch).encode()
        size = len(batch) * 16
        left, right = (ctypes.c_ubyte * size)(), (ctypes.c_ubyte * size)()
        assert funcs["C"](data, len(data), left, size) == 0
        assert funcs["Rust"](data, len(data), right, size) == 0
        assert bytes(left) == bytes(right)
        inputs.append((data, len(data), left, size))
    results = {label: [] for label in funcs}
    for i in range(-1, rounds):
        for label in tuple(funcs)[::(1 if i % 2 else -1)]:
            scan = funcs[label]
            start = time.process_time_ns()
            for _ in range(7):
                for args in inputs:
                    if scan(*args):
                        raise ValueError("validated corpus failed")
            if i >= 0:
                results[label].append([time.process_time_ns() - start])
    report(results, ["scanner CPU"], [("C", "Rust")])
    return results


def whole(root, upstream, dictionary, libs, lines, moves, rounds):
    klippy = upstream / "klippy/klippy.py"
    entry = ROOT / "bench/hybrid_klippy_entry.py"
    cfg, source = root / "printer.cfg", root / "input.gcode"
    cfg.write_text(CFG + SHAPED_EXTRUSION_CFG)
    source.write_text("\n".join(lines) + "\n")
    labels = tuple(libs)
    results = {label: [] for label in labels}
    reference, expected_coverage = None, None
    for round_ in range(-1, rounds):
        for i in ORDER[round_ % len(ORDER)]:
            label = labels[i]
            lib = libs[label]
            output, log = root / f"{label}.out", root / f"{label}.log"
            flags = ["--drain-debug-output"]
            if lib is not None:
                flags += ["--rust-dispatch", "--scanner-only"]
            cpu, wall = child_cpu(), time.perf_counter()
            proc = subprocess.run([sys.executable, entry, *flags, klippy, lib or "--stock-only",
                                  cfg, "-i", source, "-o", output, "-d", dictionary, "-l", log],
                                  text=True, capture_output=True, timeout=120)
            elapsed = [child_cpu() - cpu, time.perf_counter() - wall]
            if proc.returncode:
                raise RuntimeError(proc.stdout + proc.stderr + log.read_text()[-4000:])
            prefix = ("FILE_OUTPUT_DRAIN ", "RUST_DISPATCH_STATS ")
            telemetry = {}
            for line in proc.stdout.splitlines():
                if line.startswith(prefix):
                    key, value = line.split(" ", 1)
                    telemetry[key] = json.loads(value)
            assert telemetry["FILE_OUTPUT_DRAIN"] == [str(output)]
            assert "RUST_LOOKAHEAD_STATS" not in proc.stdout
            assert ("RUST_DISPATCH_STATS" in telemetry) == (lib is not None)
            if lib is not None:
                coverage = telemetry["RUST_DISPATCH_STATS"]
                assert 0 < coverage["canonical"] <= moves and coverage["stock"] > 0
                if expected_coverage is None:
                    expected_coverage = coverage
                assert coverage == expected_coverage
            motors = motor_trace(upstream, dictionary, output.read_bytes())
            if reference is None:
                reference = motors
            assert motors == reference, (label, round_)
            if round_ >= 0:
                results[label].append(elapsed)
    assert len(reference) == 4 and all(len(v) > 1 for v in reference.values())
    records = sum(map(len, reference.values()))
    print(f"{records} exact motor records; scanner coverage {expected_coverage}")
    report(results, ["whole CPU", "whole elapsed"], [("stock", "C"), ("stock", "Rust"), ("C", "Rust")])
    return records, expected_coverage, results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path)
    p.add_argument("dictionary", type=Path)
    p.add_argument("rust_library", type=Path)
    p.add_argument("--moves", type=int, default=12000)
    p.add_argument("--profile", choices=("layered", "shaped-extrusion"), default="layered")
    p.add_argument("--wait-interval", type=int, default=1024)
    p.add_argument("--rounds", type=int, default=12)
    p.add_argument("--native-rounds", type=int, default=80)
    p.add_argument("--json", type=Path)
    a = p.parse_args()
    if not 1200 <= a.moves <= 40000 or a.wait_interval < 0:
        p.error("invalid input size or wait interval")
    if not 6 <= a.rounds <= 60 or a.rounds % 6 or not 2 <= a.native_rounds <= 200:
        p.error("whole rounds must be a multiple of 6 in 6–60; native rounds must be 2–200")
    upstream, dictionary, rust = (v.resolve() for v in (a.upstream, a.dictionary, a.rust_library))
    assert subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip() == PIN
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    lines = list(gcode(a.moves, a.profile, a.wait_interval))
    with tempfile.TemporaryDirectory(prefix="dispatch-language-") as folder:
        root = Path(folder)
        c = root / "scanner.so"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-Wall", "-Wextra", "-Werror",
                        "-fPIC", "-shared", ROOT/"bench/dispatch_scanner.c", "-o", c], check=True)
        subprocess.run([sys.executable, ROOT/"bench/hybrid_dispatch_parity.py", upstream, c], check=True)
        kernel = native(c, rust, lines, a.native_rounds)
        records, coverage, elapsed = whole(root, upstream, dictionary, {"stock":None,"C":c,"Rust":rust},
                                           lines, a.moves, a.rounds)
        if a.json:
            data = {"pin":PIN, "profile":a.profile, "moves":a.moves, "wait_interval":a.wait_interval,
                    "records":records, "coverage":coverage,
                    "isolated_scanner_cpu_ns":kernel, "whole_cpu_elapsed_seconds":elapsed}
            if hasattr(os, "sched_getaffinity"):
                data["affinity"] = sorted(os.sched_getaffinity(0))
            a.json.write_text(json.dumps(data, indent=2)+"\n")


if __name__ == "__main__":
    main()
