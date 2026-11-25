#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify the opt-in offline preview against pinned Klippy and its C queue."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from hybrid_process_timing import CFG, SHAPED_EXTRUSION_CFG, gcode
from machine_trace import traced
from planner import PIN, ROOT


def capture(lanes):
    trace = []
    for lane in lanes:
        for record in lane:
            if record["kind"] == "direction":
                trace.append(f'd {int(record["positive"])}')
            else:
                assert record["kind"] == "steps"
                trace.append("s {} {} {} {}".format(
                    *(record[k] for k in ("interval", "count", "add", "last_clock"))))
        trace.append("end")
    return trace


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path)
    binary = p.add_mutually_exclusive_group(required=True)
    binary.add_argument("--binary", type=Path)
    binary.add_argument("--target-dir", type=Path, help="build the opt-in artifact separately")
    p.add_argument("--ordinary-binary", type=Path, help="verify stock Rust build rejects the opt-in flag")
    a = p.parse_args()
    upstream = a.upstream.resolve()
    assert subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    if a.target_dir:
        target = a.target_dir.resolve()
        subprocess.run(["cargo", "build", "--locked", "--release", "--features", "long-step-preview",
                        "--bin", "klipper-preview", "--target-dir", target], cwd=ROOT, check=True)
        cli = target / "release/klipper-preview"
    else:
        cli = a.binary.resolve()
    sys.path.insert(0, str(upstream / "klippy"))
    h = upstream / "klippy/chelper"
    with tempfile.TemporaryDirectory(prefix="long-step-preview-") as tmp:
        root = Path(tmp)
        native = root / "stock-motion"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", h, ROOT/"bench/machine_trace.c",
                        *(h/(s+".c") for s in ("itersolve", "stepcompress", "trapq", "kin_cartesian",
                           "kin_corexy", "kin_extruder", "kin_shaper", "pyhelper", "msgblock")),
                        "-lm", "-o", native], check=True)
        cfg, path = root / "printer.cfg", root / "path.gcode"
        controls = ("max_extrude_only_distance: 15\nmax_extrude_only_velocity: 20\n"
                    "max_extrude_only_accel: 300")
        section = SHAPED_EXTRUSION_CFG.replace("pressure_advance: 0.04",
                                              f"pressure_advance: 0.04\n{controls}")
        cfg.write_text(CFG + section)
        basic = [root, cfg.name, path, "20000000", "10", "10", "1"]
        extrude = ["--tool-temps", "0:210", "--fixed-pressure-script"]
        if a.ordinary_binary:
            path.write_text("G90\n")
            rejected = subprocess.run([a.ordinary_binary.resolve(), "--long-step-preview", *basic],
                                      capture_output=True)
            assert rejected.returncode and not rejected.stdout
            assert b"requires a build with --features long-step-preview" in rejected.stderr
        for profile, count in (("layered", 220), ("layered", 1200), ("shaped-extrusion", 420),
                               ("shaped-extrusion", 1400), ("layered", 3000), ("shaped-extrusion", 3000),
                               ("xyz", 2500)):
            # Stock fixture's force_move is represented by the explicit virtual start.
            lines = tuple(gcode(count, profile, 0))[1:]
            script = "\n".join(lines) + "\n"
            path.write_text(script)
            use_e = profile != "xyz"
            want, duration, _ = traced(cfg, native, script, use_e, False, preview_cli=True)
            args = [*extrude, *basic] if use_e else basic
            old = subprocess.run([cli, *args], capture_output=True)
            new = subprocess.check_output([cli, "--long-step-preview", *args])
            data = json.loads(new)
            lanes = data["motors"] + ([data["extruder_lanes"][0]["commands"]] if use_e else [])
            for i, (expected, actual) in enumerate(zip(want, capture(lanes), strict=True)):
                assert expected == actual, (profile, count, i, expected, actual)
            assert math.isclose(duration, data["planned_duration_seconds"], rel_tol=1e-12)
            if count in (220, 420):
                assert old.returncode == 0 and old.stdout == new, old.stderr
            else:
                assert old.returncode != 0 and b"EventLimit" in old.stderr and not old.stdout, old.stderr
            print(profile, count, len(want)-len(lanes), "exact C records;",
                  sum(r.get("count", 0) for lane in lanes for r in lane), "crossings")

        # Long idle generates a far singleton but no motion during the dwell.
        far_script = "G90\nG1 X20 Y15 F1800\nG4 P43000\nG1 X12 Y12 Z2\nM400\n"
        path.write_text(far_script)
        want, duration, _ = traced(cfg, native, far_script, False, False, preview_cli=True)
        data = json.loads(subprocess.check_output([cli, "--long-step-preview", *basic]))
        assert math.isclose(duration, data["planned_duration_seconds"], rel_tol=1e-12)
        assert capture(data["motors"]) == want
        assert any(r["kind"] == "steps" and r["count"] == 1 and r["interval"] >= (3 << 28)
                   for lane in data["motors"] for r in lane)
        print(len(want)-3, "exact C records around a 43-second dwell")

        excessive = far_script.replace("P43000", "P220000")
        path.write_text(excessive)
        rejected = subprocess.run([cli, "--long-step-preview", *basic], capture_output=True)
        assert rejected.returncode and not rejected.stdout and b"InvalidTime" in rejected.stderr
        path.write_text("G90\nG1 X12 Y12 F1800\n")
        for keys in (["--mixed-plan"], ["--expand-macros"], ["--virtual-sd"],
                     ["--mesh-profile", "unused"], ["--lane-workers", "1"],
                     ["--initial-tool", "0"], ["--tool-temps", "0:210"]):
            result = subprocess.run([cli, "--long-step-preview", *keys, *basic], capture_output=True)
            assert result.returncode != 0 and b"--long-step-preview" in result.stderr and not result.stdout


if __name__ == "__main__":
    main()
