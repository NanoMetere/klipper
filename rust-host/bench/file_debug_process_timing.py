#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Time pinned Klippy file debug and bounded Rust preview through identical JSON."""
import argparse
import contextlib
import io
import json
import os
from pathlib import Path
import resource
import statistics
import subprocess
import sys
import tempfile
import time

import hybrid_klippy_entry
from hybrid_klippy import motor_trace
from planner import PIN, ROOT
from shared_window_capture import flatten_file, independent_cases


LANES = ["corexy_a", "corexy_b", "z", "shared_e"]
OIDS = [3, 6, 9, 0]  # fixed pinned printer.cfg below
SCHEMA = "klipper.shared-corexy.file-debug.v1"


def stock_arm(upstream, dictionary, fixture, model, gcode):
    sys.path.insert(0, str(upstream / "klippy"))
    import stepper
    config = json.loads(model.read_text())
    hz = float(config["clock_hz"])
    if config["max_error_ticks"] == 0:
        stepper.MAX_STEPCOMPRESS_ERROR = 0.0
    else:
        assert int(hz * stepper.MAX_STEPCOMPRESS_ERROR) == config["max_error_ticks"]
    with tempfile.TemporaryDirectory(prefix="stock-normalized-debug-") as tmp:
        out = Path(tmp) / "steps.bin"
        log = Path(tmp) / "stock.log"
        sys.argv = [str(ROOT / "bench/hybrid_klippy_entry.py"), "--drain-debug-output",
                    str(upstream / "klippy/klippy.py"), "--stock-only",
                    str(fixture / "printer.cfg"), "-i", str(gcode), "-o", str(out),
                    "-d", str(dictionary), "-l", str(log)]
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            hybrid_klippy_entry.main()
        assert f'FILE_OUTPUT_DRAIN ["{out}"]' in output.getvalue()
        trace = motor_trace(upstream, dictionary, out.read_bytes())
    assert set(trace) == {f"oid={i}" for i in OIDS}
    normalized = []
    for oid in OIDS:
        lane = []
        for record in flatten_file(trace[f"oid={oid}"])[0]:
            tag, *fields = record.split()
            if tag == "d":
                lane.append({"kind": "direction", "positive": fields[0] == "1"})
            else:
                interval, count, add, last_clock = map(int, fields)
                lane.append({"kind": "steps", "interval": interval, "count": count,
                             "add": add, "last_clock": last_clock})
        normalized.append(lane)
    print(json.dumps({"schema": SCHEMA, "clock_hz": hz, "lane_order": LANES,
                      "lanes": normalized}, separators=(",", ":")))


def child_cpu():
    r = resource.getrusage(resource.RUSAGE_CHILDREN)
    return r.ru_utime + r.ru_stime


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=lambda s: Path(s).resolve())
    p.add_argument("dictionary", type=lambda s: Path(s).resolve())
    p.add_argument("file_debug_preview", type=lambda s: Path(s).resolve())
    p.add_argument("--rounds", type=int, default=20)
    p.add_argument("--cores")
    p.add_argument("--zero-error", action="store_true")
    a = p.parse_args()
    if not 1 <= a.rounds <= 100:
        p.error("--rounds must be 1–100")
    if a.cores:
        os.sched_setaffinity(0, {int(s) for s in a.cores.split(",")})
    print("Effective Linux CPU affinity:", sorted(os.sched_getaffinity(0)), flush=True)
    assert subprocess.check_output(["git", "-C", a.upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    fixture = ROOT / "bench/shared_extruder_fixture"
    source = (fixture / "path.gcode").read_text()
    long = source.replace("M400\n", "".join(
        f"G1 X{18 + (i % 2)*0.24:.3f} Y{39 + (1 - i % 2)*0.14:.3f} E0.002 F600\n"
        for i in range(216)) + "M400\n", 1)
    cases = [("alternating", long, 29992)]
    cases += [x for x in independent_cases(source, fixture, long)
              if x[0] in {"wide", "xyz-with-retract", "waits-and-dwell"}]
    cfg = json.loads((fixture / "offline_model.json").read_text())
    assert float(json.loads(a.dictionary.read_text())["config"]["CLOCK_FREQ"]) == cfg["clock_hz"]
    cfg["max_error_ticks"] = 0 if a.zero_error else int(cfg["clock_hz"] * 0.000025)
    with tempfile.TemporaryDirectory(prefix="paired-normalized-debug-") as d:
        model = Path(d) / "model.json"
        model.write_text(json.dumps(cfg))
        gcode = Path(d) / "fixture.gcode"
        commands = [[sys.executable, Path(__file__).resolve(), "--stock-arm", a.upstream, a.dictionary,
                     fixture, model, gcode], [a.file_debug_preview, model, gcode]]
        for name, script, steps in cases:
            gcode.write_text(script)
            reference = None
            wall, cpu = [[], []], [[], []]
            for pair in range(-2, a.rounds):
                for arm in (pair % 2, 1 - pair % 2):
                    c0, t0 = child_cpu(), time.perf_counter_ns()
                    output = subprocess.check_output(commands[arm], timeout=20)
                    elapsed = (time.perf_counter_ns() - t0) / 1e6
                    used = (child_cpu() - c0) * 1000
                    if reference is None:
                        reference = output
                    assert output == reference, (name, pair, arm)
                    if pair >= 0:
                        wall[arm].append(elapsed)
                        cpu[arm].append(used)
            lanes = json.loads(reference)["lanes"]
            assert sum(r.get("count", 0) for lane in lanes for r in lane) == steps
            print(name, cfg["max_error_ticks"], steps, sum(map(len, lanes)), len(reference),
                  "median wall/Cpu ms stock and Rust",
                  [[round(statistics.median(vals[arm]), 3) for vals in (wall, cpu)]
                   for arm in (0, 1)], flush=True)
    print("Includes startup and normalization; narrow offline predictor versus stock debug host, "
          "not a calibrated comparison of language, full host, or print speed.")


if __name__ == "__main__":
    if sys.argv[1:2] == ["--stock-arm"]:
        stock_arm(*(Path(x) for x in sys.argv[2:]))
    else:
        main()
