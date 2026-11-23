#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare virtual sparse heated-tool phases with stock Python and C."""
import argparse
import os
from pathlib import Path
import random
import subprocess
import tempfile

from machine_trace import CONFIG
from mixed_step_grid import net
from planner import PIN, ROOT
from tool_motion import reference


def fixtures():
    rng = random.Random(12_052_206)
    for case in range(41):
        yield_code = ["G90", "M83"]
        order = [0, 2, 6, 2, 0] if case % 2 == 0 else [0, 6, 2, 0, 2]
        for j, tool in enumerate(order):
            yield_code.extend([f"T{tool}", f"M109 S{190+j*5}"])
            for k in range(4 + case % 2):
                dx, dy, e = rng.randint(4, 400), rng.randint(4, 400), rng.randint(1, 9)
                x = (j + 1) * 4 + 10 + dx / 1000
                y = 10 + dy / 1000
                # Keep the ratio safely below the configured extrusion cap.
                if k and abs(x - oldx) + abs(y - oldy) < .12:
                    x += .13
                oldx, oldy = x, y
                yield_code.append(f"G1 X{x:.4f} Y{y:.4f} E{e / 1000:.4f} F1200")
            yield_code.append("M104 T1 S190")
            yield_code.append("G1 E-.004 F120")
        yield yield_code


def without_heaters(lines):
    for line in lines:
        if line.startswith("M10"):
            # The Rust planner uses explicit stops and lets the previous
            # bounded shaper/extrusion history settle before the next phase.
            yield "M400"
            yield "G4 P550"
        else:
            if line.startswith("T"):
                yield "M400"
                yield "G4 P550"
            yield line


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, required=True)
    args = p.parse_args()
    stock = args.upstream.resolve()
    helpers = stock / "klippy/chelper"
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        compiler = root / "reference"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", helpers,
                        ROOT/"bench/machine_trace.c",
                        *(helpers/(n+".c") for n in ("itersolve", "stepcompress", "trapq", "kin_cartesian",
                                                  "kin_corexy", "kin_extruder", "kin_shaper",
                                                  "pyhelper", "msgblock")),
                        "-lm", "-o", compiler], check=True)
        cases = list(fixtures())
        cfg = root / "fixture.cfg"
        cfg.write_text(CONFIG.format(kinematics="corexy") +
                       "\n[input_shaper]\nshaper_freq_x: 56.6\nshaper_type_y: ei\nshaper_freq_y: 48.4\n")
        text = "\n--case--\n".join("\n".join(lines) for lines in cases)
        results = subprocess.check_output([args.binary.resolve(), cfg], input=text, text=True)
        for n, (row, script) in enumerate(zip(results.splitlines(), cases, strict=True)):
            expected, _ = reference(stock, compiler, True, list(without_heaters(script)),
                                    parked=(0., 1.3, 2.1))
            want = net(expected)
            got = list(map(int, row.split()))
            assert got == want, (n, got, want, script)
        print(f"{len(cases)} shaped sparse tool jobs matched signed stock steps on all six lanes")


if __name__ == "__main__":
    main()
