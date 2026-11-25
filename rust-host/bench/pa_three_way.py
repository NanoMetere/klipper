#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare stock C, prepared-coefficient C, and Rust on pinned offline traces."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tempfile

from machine_trace import CONFIG, WITH_E, RUNTIME_PRESSURE, rolling_pressure, traced
from planner import PIN, ROOT

HELPERS = ("itersolve", "stepcompress", "trapq", "kin_cartesian",
           "kin_corexy", "kin_extruder", "kin_shaper", "pyhelper", "msgblock")


def build(root, tmp):
    host = root / "klippy/chelper"
    source = ROOT / "bench/machine_trace.c"
    copy = tmp / "prepared"
    copy.mkdir()
    for path in host.iterdir():
        if path.suffix in (".c", ".h"):
            shutil.copyfile(path, copy / path.name)
    with (ROOT / "bench/pa_prepared_c.patch").open("rb") as patch:
        subprocess.run(["patch", "-p1", "--batch"], stdin=patch, cwd=copy,
                       stdout=subprocess.DEVNULL, check=True)

    compiler = os.environ.get("CC", "cc")
    stock, prepared, rust = [tmp / label for label in ("stock", "prepared-c", "prepared-rust")]
    for headers, binary, flags in ((host, stock, []), (copy, prepared, ["-DPREPARED_PA"])):
        subprocess.run([compiler, "-O3", "-ffp-contract=off", "-I", headers, *flags, source,
                        *(headers / f"{name}.c" for name in HELPERS), "-lm", "-o", binary],
                       check=True)
    lib, clock = tmp / "libklipper_host_protocol.rlib", tmp / "clock.o"
    subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                    "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib],
                   check=True)
    subprocess.run([compiler, "-O3", "-I", host, "-DMACHINE_TRACE_CLOCK_ONLY",
                    "-c", source, "-o", clock], check=True)
    subprocess.run(["rustc", "--edition=2021", "-O", "--cfg", "input_shaper_fixture",
                    ROOT / "bench/machine_trace.rs", "--extern", f"klipper_host_protocol={lib}",
                    "-C", f"link-arg={clock}", "-o", rust], check=True)
    return stock, prepared, rust


def equal(reference, candidate):
    left, duration, _ = reference
    right, elapsed, _ = candidate
    assert math.isclose(duration, elapsed, rel_tol=1e-12, abs_tol=1e-12), (duration, elapsed)
    for i, (a, b) in enumerate(zip(left, right, strict=True)):
        assert a == b, (i, a, b)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("--rounds", type=int, default=48, help="timed blocks, a multiple of 6")
    parser.add_argument("--json", type=Path, help="write individual CPU times in nanoseconds")
    args = parser.parse_args()
    if not 6 <= args.rounds <= 240 or args.rounds % 6:
        parser.error("--rounds must be 6–240 and a multiple of 6")
    upstream = args.upstream.resolve()
    assert subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(upstream / "klippy"))
    with tempfile.TemporaryDirectory(prefix="pa-three-way-") as d:
        tmp = Path(d)
        binaries = build(upstream, tmp)
        cfg = tmp / "machine.cfg"
        totals = [[], [], []]
        c_native = [[], []]

        def run(k, script, runtime=False, measure=False):
            if k < 2:
                return traced(cfg, binaries[k], script, True, not runtime,
                              runtime_pressure=runtime, native_samples=c_native[k] if measure else None)
            args = ["--shape", "--runtime-pressure"] if runtime else ["--pressure"]
            p = subprocess.run([binaries[k], cfg, *args], input=script, text=True,
                               capture_output=True, check=True)
            lines = p.stdout.splitlines()
            return lines[:-1], float(lines[-1].split()[1]), int(p.stderr.split()[1])

        for kin in ("cartesian", "corexy"):
            cfg.write_text(CONFIG.format(kinematics=kin))
            for script in (RUNTIME_PRESSURE, rolling_pressure()):
                want = traced(cfg, binaries[0], script, True, False, shaped=True, runtime_pressure=True)
                equal(want, traced(cfg, binaries[1], script, True, False,
                                   shaped=True, runtime_pressure=True))
                equal(want, run(2, script, runtime=True))

        cfg.write_text(CONFIG.format(kinematics="corexy") +
                       "[input_shaper]\nshaper_freq_x: 56.6\nshaper_type_y: ei\nshaper_freq_y: 48.4\n")
        reset = "G90\nM82\nM221 S100\nG1 X1 Y1 Z.2 F2400\nG92 E0\n"
        script = "\n".join([reset + WITH_E] * 12)
        order = ((0, 1, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0), (0, 2, 1), (1, 0, 2))
        for i in range(-1, args.rounds):
            outputs = [None] * 3
            for k in order[i % 6]:
                outputs[k] = run(k, script, measure=i >= 0)
                if i >= 0:
                    totals[k].append(outputs[k][2])
            for got in outputs[1:]:
                equal(outputs[0], got)
        count = len(outputs[0][0]) - 4
        print(f"{count} exact timed records, {outputs[0][1]:.9f} modeled seconds")
        labels = ("stock-c-plus-python", "prepared-c-plus-python", "prepared-rust")
        for title, samples in (("config-to-steps", totals), ("native-c-stage", c_native)):
            for label, values in zip(labels[:len(samples)], samples, strict=True):
                ratios = [v / b for v, b in zip(values, samples[0], strict=True)]
                print(f"{title} {label}: {statistics.median(values)/1e6:.4f} ms CPU; "
                      f"{(statistics.median(ratios)-1)*100:+.2f}% paired; "
                      f"{sum(v < 1 for v in ratios)}/{len(ratios)} wins")
        if args.json:
            report = {"pin": PIN, "records": count, "rounds": args.rounds,
                      "config_to_steps_cpu_ns": dict(zip(labels, totals, strict=True)),
                      "native_stage_cpu_ns": dict(zip(labels[:2], c_native, strict=True))}
            args.json.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
