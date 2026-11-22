#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare mixed-file motion stages against stock with explicit barriers."""
import argparse
import math
from pathlib import Path
import random
import subprocess
import sys

from gcode_preview import stock
from planner import PIN, ROOT

HEAT = ("M109 S210", "M190 S60", "M104 S195", "TURN_OFF_HEATERS")


def fixtures():
    r = random.Random(7731027)
    for _ in range(180):
        commands = ["G90", "M82"]
        for phase in range(4):
            if phase == 2:
                commands.extend(["G91", "M83"])
            for __ in range(3):
                signs = 1 if phase < 2 else -1
                x, y, z = (signs * r.randint(4, 40) / 10 for ___ in range(3))
                e = r.randrange(0, 20) / 60
                commands.append(f"G1 X{x:.5f} Y{y:.5f} Z{z:.5f} E{e:.5f} F{r.randint(500,2000)}")
            if phase & 1:
                commands.append(f"G4 P{r.randint(5,200)}")
            commands.append(HEAT[phase])
        yield commands


def reference(lines):
    command_stream, records = [], []
    opened = False
    for n, command in enumerate(lines, start=1):
        if command in HEAT:
            if opened:
                command_stream.append("M400")  # deliberately conservative
                opened = False
            command_stream.append(f"!heater {n}")
        else:
            if not opened:
                opened = True
                command_stream.append(f"!motion {n}")
            command_stream.append(command)

    def record(line, _, __):
        records.append(line[1:])

    def motion(moves, dwell):
        records.extend(" ".join(map(str, ["move", int(m.is_kinematic_move),
                      *m.start_pos, *m.end_pos, *m.axes_r,
                      m.start_v,m.cruise_v,m.end_v,m.accel,m.accel_t,m.cruise_t,m.decel_t]))
                       for m in moves)
        if dwell:
            records.append(f"dwell {dwell}")

    stock(command_stream, False, emit_profiles=motion, extend_directive=record)
    return [*records,"next"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream",type=Path,help=f"checkout at {PIN}")
    p.add_argument("--binary",type=Path,default=ROOT/"target/release/examples/mixed_preflight")
    a = p.parse_args()
    sys.path.insert(0,str(a.upstream.resolve()/"klippy"))
    files = list(fixtures())
    want = [line for code in files for line in reference(code)]
    source = "\n--case--\n".join("\n".join(code) for code in files)
    got = subprocess.check_output([a.binary],input=source,text=True).splitlines()
    for i, (w,g) in enumerate(zip(want,got,strict=True)):
        prefix = w.split()[0]
        if prefix in ("move","dwell"):
            a, b = w.split(), g.split()
            assert a[0] == b[0]
            for n,m in zip(map(float,a[1:]),map(float,b[1:]),strict=True):
                assert math.isclose(n,m,abs_tol=2e-12,rel_tol=2e-12),(i,w,g)
        else:
            assert w == g, (i,w,g)
    print(f"{len(files)} mixed files, {sum(l.startswith('move ') for l in got)} equal "
          f"profiles and {sum(l.startswith('heater ') for l in got)} explicit boundaries")


if __name__ == "__main__":
    main()
