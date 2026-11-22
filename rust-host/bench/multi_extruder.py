#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare GCodeMove extruder switching on a mock toolhead, no extruder I/O."""
import argparse
from pathlib import Path
import math
import random
import subprocess
import sys

from planner import PIN, ROOT
from gcode_preview import stock


def fixtures(upstream):
    sys.path.insert(0, str(upstream.resolve() / "klippy"))
    rng = random.Random(73095)
    corpus, expect = [], []
    for i in range(160):
        setup = {"active": 0, "positions": [0., 1.3, -.5]}
        def extension(line, coords, head):
            cmd, *args = line.split()
            active = setup["active"]
            parked = setup["positions"]
            if cmd == "!use":
                tool = int(args[0])
                if active == tool:
                    return
                head._flush_lookahead()
                parked[active] = head.commanded_pos[3]
                head.commanded_pos[3] = parked[tool]
                coords._handle_activate_extruder()
                setup["active"] = tool
            elif cmd == "!state":
                parked[active] = head.commanded_pos[3]
                expect.append([active, coords._get_gcode_position()[3], head.commanded_pos[3], *parked])
            else:
                raise ValueError(cmd)
        gcode = ["G90", "M82", "G1 X12 Y17 E1 F1500", "!state"]
        for n in range(120):
            if n % 11 == 0:
                gcode.extend([f"M221 S{rng.choice([78,100,123])}", "G92 E0",
                              "SAVE_GCODE_STATE NAME=switch", f"!use {rng.randrange(3)}",
                              "!state", "RESTORE_GCODE_STATE NAME=switch"])
            elif n % 4 == 0:
                gcode.extend(["M83", "G1 E-.12 F600", "M82"])
            else:
                gcode.append(f"G1 X{12+(n%9)*.5} Y{17+(n%13)*.4} E{rng.randrange(8,35)*.08:.3f} F1500")
            gcode.append("!state")
        stock(gcode, False, extend_directive=extension)
        corpus.extend(["!reset_tools", *gcode])
    return corpus, expect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/multi_extruder_replay")
    a = p.parse_args()
    script, want = fixtures(a.upstream)
    output = subprocess.check_output([a.binary], text=True, input=chr(10).join(script)+chr(10))
    for i, (row, expected) in enumerate(zip(output.splitlines(), want, strict=True)):
        for actual, desired in zip(map(float, row.split()), expected, strict=True):
            assert math.isclose(actual, desired, rel_tol=1e-12, abs_tol=1e-12), (i, row, expected)
    print(f"{len(want)} multi-extruder logical/parked-position snapshots match stock")


if __name__ == "__main__":
    main()
