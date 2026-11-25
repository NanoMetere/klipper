#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Time one or four lanes within each workload, not C versus Rust throughput."""
import argparse
import json
import os
from pathlib import Path
from statistics import median
import subprocess
import tempfile
from time import perf_counter_ns

from planner import PIN, ROOT
from shared_window_capture import ATTRS, XYZ_ATTRS, flatten_file, independent_cases, run_stock


def build_c(upstream, exe):
    helper = upstream / "klippy/chelper"
    sources = ("itersolve", "trapq", "kin_corexy", "kin_cartesian",
               "kin_extruder", "stepcompress", "msgblock", "pyhelper")
    subprocess.run(
        ["cc", "-O3", "-DWIRE", "-DCAPTURE_LANES", "-pthread", "-I", helper,
         ROOT / "bench/file_debug_capture_lanes.c", ROOT / "bench/shared_windows.c",
         *(helper / f"{src}.c" for src in sources), "-lm", "-o", exe], check=True)


def cases(fixture):
    source = (fixture / "path.gcode").read_text()
    long = source.replace("M400\n", "".join(
        f"G1 X{18+(i%2)*.24:.3f} Y{39+(1-i%2)*.14:.3f} E.002 F600\n"
        for i in range(216)) + "M400\n", 1)
    yield "alternating", long, 29992
    yield from (case for case in independent_cases(source, fixture, long)
                if case[0] in {"wide", "xyz-with-retract", "waits-and-dwell"})


def rows_with_prefix(rows, prefix):
    return [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith(prefix + " ")]


def timeline(rows, attrs):
    lines = []
    for row in rows:
        op = row["op"]
        lines.append(" ".join(map(str, (op, *(row[key] for key in attrs[op].split())))))
        if op == "f":
            lines.append(f"w {row['cutoff']}")
    return "\n".join(lines) + "\n"


def rust_records(raw, hz):
    cfg = json.loads(raw)
    assert cfg["schema"] == "klipper.shared-corexy.file-debug.v1"
    assert cfg["clock_hz"] == hz and cfg["lane_order"] == ["corexy_a", "corexy_b", "z", "shared_e"]
    return [[f"d {int(row['positive'])}" if row["kind"] == "direction" else
             "s {interval} {count} {add} {last_clock}".format(**row)
             for row in lane] for lane in cfg["lanes"]]


def measure(name, commands, expected, outfile, pairs):
    times = []
    for i in range(-6, pairs):
        pair = [None, None]
        for j in ((0, 1) if i % 2 == 0 else (1, 0)):
            start = perf_counter_ns()
            with outfile.open("wb") as f:
                # Blocking wait: timeout polling quantizes these short runs.
                subprocess.run(commands[j], stdout=f, check=True)
            pair[j] = perf_counter_ns() - start
            assert outfile.read_bytes() == expected, (name, i, j)
        if i >= 0:
            times.append(pair)
    one, four = (median(ms)/1e6 for ms in zip(*times))
    wall = 100 * median(b/a - 1 for a, b in times)
    print(f"{name}: serial {one:.3f} ms, parallel {four:.3f} ms, "
          f"paired wall {wall:+.2f}%", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    def resolved(s):
        return Path(s).resolve()

    p.add_argument("upstream", type=resolved)
    p.add_argument("dictionary", type=resolved)
    p.add_argument("--file-debug-preview", type=resolved, help="also time the complete Rust offline CLI")
    p.add_argument("--cores", help="comma-separated Linux CPU affinity")
    p.add_argument("--rounds", type=int, default=60)
    a = p.parse_args()
    if not 1 <= a.rounds <= 200:
        p.error("--rounds must be 1–200")
    if a.cores:
        os.sched_setaffinity(0, {int(c) for c in a.cores.split(",")})
    print("Effective Linux CPU affinity:", sorted(os.sched_getaffinity(0)), flush=True)
    assert subprocess.check_output(
        ["git", "-C", a.upstream, "rev-parse", "HEAD"], text=True).strip() == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    hz = int(json.loads(a.dictionary.read_text())["config"]["CLOCK_FREQ"])
    fixture = ROOT / "bench/shared_extruder_fixture"
    cfg = json.loads((fixture / "offline_model.json").read_text())
    assert hz == cfg["clock_hz"]
    with tempfile.TemporaryDirectory(prefix="captured-lanes-") as temp:
        t = Path(temp)
        exe, model, gcode, xyz, e, out = (t / s for s in (
            "capture_lanes", "model.json", "path.gcode", "xyz.txt", "e.txt", "lanes.out"))
        build_c(a.upstream, exe)
        for zero in (True, False):
            for case, script, count in cases(fixture):
                gcode.write_text(script)
                rows, stock = run_stock(a.upstream, a.dictionary, fixture, t, gcode, zero)
                _, repeat = run_stock(a.upstream, a.dictionary, fixture, t, gcode, zero)
                assert stock == repeat
                meta, = rows_with_prefix(rows, "XYZ_BIND")
                eid, = {row["oid"] for row in rows_with_prefix(rows, "SHARED_OUTCOME")}
                tolerance = meta["max_error"]
                assert tolerance == (0 if zero else int(hz * .000025))
                e_rows = rows_with_prefix(rows, "WINDOW")
                e_start = next(i for i, row in enumerate(e_rows) if row["op"] == "p")
                e.write_text(timeline(e_rows[e_start:], ATTRS))
                xyz.write_text(timeline(rows_with_prefix(rows, "XYZ"), XYZ_ATTRS))
                flattened = [flatten_file(stock[f"oid={oid}"]) for oid in [*meta["oids"], eid]]
                assert sum(steps for _, steps in flattened) == count
                lanes = [packets for packets, _ in flattened]
                expected = "".join("\n".join([*lane, "end", ""]) for lane in lanes).encode()
                args = [xyz, e, str(hz), str(tolerance),
                        *map(str, meta["d"]), *(str(int(v)) for v in meta["inverted"])]
                label = f"{tolerance} ticks, {case}, {count} exact stock steps"
                measure(f"C captured replay | {label}",
                        [[exe, mode, *args] for mode in ("1", "4")], expected, out, a.rounds)
                if a.file_debug_preview:
                    cfg["max_error_ticks"] = tolerance
                    model.write_text(json.dumps(cfg))
                    serial = [a.file_debug_preview, model, gcode]
                    workers = [a.file_debug_preview, "--lane-workers", "4", model, gcode]
                    rust = subprocess.check_output(serial, timeout=10)
                    assert rust_records(rust, hz) == lanes
                    measure(f"Rust full file-debug | {label}", [serial, workers], rust, out, a.rounds)


if __name__ == "__main__":
    main()
