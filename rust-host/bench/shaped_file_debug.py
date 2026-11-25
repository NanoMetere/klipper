#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Pinned stock, C replay, and independent Rust fixed XY shaped file-debug."""
import argparse
import json
import os
from pathlib import Path
import shutil
from statistics import median
import subprocess
import sys
import tempfile
from time import perf_counter_ns

from file_debug_lane_timing import cases, rows_with_prefix, timeline
from file_debug_process_timing import stock_arm
from shared_window_capture import (ATTRS, XYZ_ATTRS, PIN, ROOT, flatten_file,
                                   independent_cases, run_stock)

SCHEMA = "klipper.shared-corexy.shaped-file-debug.v1"
SHAPERS = (("mzv", 45, "ei", 39), ("mzv", 45, "mzv", 45),
           ("zv", 60, "zvd", 45), ("2hump_ei", 35, "3hump_ei", 42),
           ("mzv", 45, "mzv", 0), ("mzv", 0, "ei", 39))


def build_c(upstream, exe):
    helper = upstream / "klippy/chelper"
    sources = ("itersolve", "trapq", "kin_corexy", "kin_cartesian",
               "kin_extruder", "kin_shaper", "stepcompress", "msgblock", "pyhelper")
    subprocess.run(["cc", "-O3", "-DWIRE", "-DCAPTURE_LANES", "-DCAPTURE_SHAPE",
                    "-pthread", "-I", helper, ROOT / "bench/file_debug_capture_lanes.c",
                    ROOT / "bench/shared_windows.c",
                    *(helper / f"{src}.c" for src in sources), "-lm", "-o", exe],
                   check=True)


def check_c(exe, rows, stock, xyz, e, hz, shapers, tolerance):
    from extras import shaper_defs
    meta, = rows_with_prefix(rows, "XYZ_BIND")
    assert meta["max_error"] == tolerance
    eid, = {row["oid"] for row in rows_with_prefix(rows, "SHARED_OUTCOME")}
    e_rows = rows_with_prefix(rows, "WINDOW")
    start = next(i for i, row in enumerate(e_rows) if row["op"] == "p")
    e.write_text(timeline(e_rows[start:], ATTRS))
    pulses = []
    for axis, (kind, frequency) in zip("xy", shapers, strict=True):
        if frequency:
            a, t = shaper_defs.init_shaper(kind, frequency, shaper_defs.DEFAULT_DAMPING_RATIO)
            fields = (v for pair in zip(a, t, strict=True) for v in pair)
            pulses.append(" ".join(map(str, ("h", axis, len(a), *fields))) + "\n")
    xyz.write_text("".join(pulses) + timeline(rows_with_prefix(rows, "XYZ"), XYZ_ATTRS))
    expected = [flatten_file(stock[f"oid={oid}"]) for oid in [*meta["oids"], eid]]
    text = "".join("\n".join([*packets, "end", ""]) for packets, _ in expected).encode()
    args = [xyz, e, str(hz), str(tolerance),
            *map(str, meta["d"]), *(str(int(v)) for v in meta["inverted"])]
    for workers in ("1", "4"):
        assert subprocess.check_output([exe, workers, *args], timeout=10) == text
    return expected, args, text


def normalize(raw, hz):
    trace = json.loads(raw)
    assert trace["schema"] == SCHEMA and trace["clock_hz"] == hz
    assert trace["lane_order"] == ["corexy_a", "corexy_b", "z", "shared_e"]
    return [[f"d {int(row['positive'])}" if row["kind"] == "direction" else
             "s {interval} {count} {add} {last_clock}".format(**row)
             for row in lane] for lane in trace["lanes"]]


def paired(arms, out, expected, rounds):
    times = []
    for i in range(-2, rounds):
        pair = [0, 0]
        for j in ((0, 1) if i % 2 == 0 else (1, 0)):
            start = perf_counter_ns()
            with out.open("wb") as f:
                subprocess.run(arms[j], stdout=f, check=True)
            pair[j] = perf_counter_ns() - start
            assert out.read_bytes() == expected
        if i >= 0:
            times.append(pair)
    stock, rust = (median(samples) / 1e6 for samples in zip(*times))
    print(f"  Whole normalized output: stock {stock:.2f} ms, Rust {rust:.2f} ms, "
          f"{rounds} alternating pairs", flush=True)


def paired_workers(label, arms, output, expected, rounds):
    samples = []
    for i in range(-6, rounds):
        times = [0] * len(arms)
        order = [(i+j) % len(arms) for j in range(len(arms))]
        for j in (order if i % 2 == 0 else reversed(order)):
            start = perf_counter_ns()
            with output.open("wb") as f:
                subprocess.run(arms[j], stdout=f, check=True)
            times[j] = perf_counter_ns() - start
            assert output.read_bytes() == expected, (label, i, j)
        if i >= 0:
            samples.append(times)
    ms = [round(median(values)/1e6, 3) for values in zip(*samples)]
    delta = [round(100*median(row[j]/row[0]-1 for row in samples), 2)
             for j in range(1, len(arms))]
    print(f"  {label}: ms {ms}, paired wall change {delta}%", flush=True)


def invalid_inputs(cli, model, script, settings, root):
    good = settings.read_text()
    invalid = [
        "",
        "[input_shaper]\nshaper_freq_x: 0\n",
        "[input_shaper]\nshaper_freq_x: NaN\n",
        "[input_shaper]\nshaper_freq_x: 0.1\n",
        "[input_shaper]\nshaper_freq_x: 45\nshaper_freq_z: 40\n",
        "[input_shaper]\nshaper_freq_x: 45\nshaper_type_x: something\n",
        "[input_shaper]\nshaper_freq_x: 45\n[printer]\nkinematics: corexy\n",
        "[include other.cfg]\n" + good,
        "[DEFAULT]\nshaper_freq_x: 45\n[input_shaper]\n",
        " " * 16_385,
    ]
    for case in invalid:
        settings.write_text(case)
        attempt = subprocess.run([cli, model, script, settings], capture_output=True, timeout=5)
        assert attempt.returncode == 2 and not attempt.stdout, (case[:60], attempt.stderr)
    settings.write_text(good)
    alias = root / "alias.cfg"
    alias.symlink_to(settings)
    attempt = subprocess.run([cli, model, script, alias], capture_output=True, timeout=5)
    assert attempt.returncode == 2 and not attempt.stdout and b"non-symlink" in attempt.stderr
    attempt = subprocess.run([cli, model, script], capture_output=True, timeout=5)
    assert attempt.returncode == 2 and not attempt.stdout and b"Usage:" in attempt.stderr
    for workers in ("0", "1", "8", "04", "auto"):
        attempt = subprocess.run([cli, "--lane-workers", workers, model, script, settings],
                                 capture_output=True, timeout=5)
        assert attempt.returncode == 2 and not attempt.stdout and b"Usage:" in attempt.stderr
    script.write_text("M400\n")
    attempt = subprocess.run([cli, model, script, settings], capture_output=True, timeout=5)
    assert attempt.returncode == 2 and not attempt.stdout


def main(argv):
    if len(argv) == 6 and argv[0] == "--stock-arm":
        stock_arm(*(Path(w).resolve() for w in argv[1:]), schema=SCHEMA)
        return
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path)
    p.add_argument("dictionary", type=Path)
    p.add_argument("cli", type=Path)
    p.add_argument("--rounds", type=int, default=0, help="timing pairs for the first shaper setup")
    p.add_argument("--worker-rounds", type=int, default=0, help="paired serial/worker timings on the first two presets")
    p.add_argument("--baseline-cli", type=Path, help="prior shaped CLI to time as the first Rust arm")
    p.add_argument("--cores", help="comma-separated Linux CPU affinity")
    a = p.parse_args(argv)
    if not 0 <= a.rounds <= 80:
        p.error("--rounds must be 0–80")
    if not 0 <= a.worker_rounds <= 200:
        p.error("--worker-rounds must be 0–200")
    if a.baseline_cli and not a.worker_rounds:
        p.error("--baseline-cli requires --worker-rounds")
    baseline = a.baseline_cli.resolve() if a.baseline_cli else None
    if a.cores:
        os.sched_setaffinity(0, {int(c) for c in a.cores.split(",")})
    upstream, dictionary, cli = (v.resolve() for v in (a.upstream, a.dictionary, a.cli))
    assert subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip() == PIN
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(upstream / "klippy"))
    from extras import shaper_defs
    assert shaper_defs.DEFAULT_DAMPING_RATIO == .1
    base_fixture = ROOT / "bench/shared_extruder_fixture"
    hz = int(json.loads(dictionary.read_text())["config"]["CLOCK_FREQ"])
    model_cfg = json.loads((base_fixture / "offline_model.json").read_text())
    assert hz == model_cfg["clock_hz"]
    checked = 0
    with tempfile.TemporaryDirectory(prefix="klipper-shaped-file-debug-") as d:
        tmp = Path(d)
        fixture = tmp / "fixture"
        shutil.copytree(base_fixture, fixture)
        cfg = fixture / "printer.cfg"
        base = cfg.read_text()
        exe, model, script, settings, xyz, e, output = (tmp / f for f in
            ("captured-c", "model.json", "moves.gcode", "shape.cfg", "xyz.txt", "e.txt", "out.json"))
        build_c(upstream, exe)
        if a.rounds or a.worker_rounds:
            print("Effective Linux affinity:", sorted(os.sched_getaffinity(0)), flush=True)
        examples = list(cases(base_fixture))
        plain = (base_fixture / "path.gcode").read_text()
        boundaries = [case for case in independent_cases(plain, base_fixture, examples[0][1])
                      if case[0] in {"byte-4095", "byte-4096", "byte-4097"}]
        for spec in SHAPERS:
            x, fx, y, fy = spec
            shape_cfg = (f"[input_shaper]\nshaper_type_x: {x}\nshaper_freq_x: {fx}\n"
                         f"shaper_type_y: {y}\nshaper_freq_y: {fy}\n")
            cfg.write_text(base + "\n" + shape_cfg)
            settings.write_text(shape_cfg)
            jobs = examples + boundaries if spec == SHAPERS[0] else examples
            for name, gcode, _ in jobs:
                script.write_text(gcode)
                for tol in (0, int(hz * .000025)):
                    rows, stock = run_stock(upstream, dictionary, fixture, tmp, script, tol == 0)
                    _, repeat = run_stock(upstream, dictionary, fixture, tmp, script, tol == 0)
                    assert stock == repeat
                    expected, c_args, c_raw = check_c(
                        exe, rows, stock, xyz, e, hz, ((x, fx), (y, fy)), tol)
                    model_cfg["max_error_ticks"] = tol
                    model.write_text(json.dumps(model_cfg))
                    rust = [cli, model, script, settings]
                    raw = subprocess.check_output(rust, timeout=10)
                    workers = [cli, "--lane-workers", "4", model, script, settings]
                    assert subprocess.check_output(workers, timeout=10) == raw, (spec, name, tol)
                    assert normalize(raw, hz) == [lane for lane, _ in expected], (spec, name, tol)
                    count = sum(n for _, n in expected)
                    print(f"{x}{fx}/{y}{fy} {name}, {tol} ticks: {count} exact steps", flush=True)
                    checked += 1
                    if a.rounds and spec == SHAPERS[0] and not name.startswith("byte-"):
                        native = [sys.executable, Path(__file__).resolve(), "--stock-arm",
                                  upstream, dictionary, fixture, model, script]
                        paired([native, rust], output, raw, a.rounds)
                    if a.worker_rounds and spec in SHAPERS[:2] and not name.startswith("byte-"):
                        arms = [rust, workers]
                        if baseline:
                            arms.insert(0, [baseline, model, script, settings])
                        paired_workers("Rust whole CLI", arms, output, raw, a.worker_rounds)
                        paired_workers("C captured replay", [[exe, n, *c_args] for n in ("1", "4")],
                                       output, c_raw, a.worker_rounds)
        invalid_inputs(cli, model, script, settings, tmp)
    print(f"{checked} shaped stock cases, exact C replay and both Rust worker modes; invalid inputs rejected")


if __name__ == "__main__":
    main(sys.argv[1:])
