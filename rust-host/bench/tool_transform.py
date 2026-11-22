#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify a synthetic Toolchanger-style translation lifecycle, not the plugin."""
import argparse
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from contextlib import redirect_stdout
from io import StringIO

from gcode_preview import stock
from planner import PIN, ROOT

FIXTURE = """G90
!wrap 1 0.05 -0.03 0.012 0
!reset
!wrap 2 0.16 0.48 -0.04 1
!reset
SET_GCODE_OFFSET Z_ADJUST=0.3
G1 X12.1 Y25.4 Z0.25 E10 F1800
M220 S80
M221 S125
SAVE_GCODE_STATE NAME=_toolchange_state
!set 2 0 0 0
!reset
SET_GCODE_OFFSET X=0 Y=0 Z=0
G0 X35 Y18 Z4.5
G0 X29 Y16
!set 2 -0.3 0.062 0.07
!reset
RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0
G90
G1 X12.1 Y25.4
RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0
G1 X25 Y20 Z0.25 E12
!unwrap 2
!reset
G91
G1 X3 Y-.4
!unwrap 1
!reset
G90
M400
"""


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    args = p.parse_args()
    sys.path.insert(0, str(args.upstream.resolve() / "klippy"))
    out = StringIO()
    with redirect_stdout(out):
        stock(FIXTURE.splitlines(), True)
    want = out.getvalue().splitlines()
    with tempfile.TemporaryDirectory() as dir_:
        t = Path(dir_)
        rust, lib = t / "rust", t / "libklipper_host_protocol.rlib"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/gcode_preview.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        actual = subprocess.check_output([rust], input=FIXTURE, text=True).splitlines()
        stats = {}
        for i, (a, b) in enumerate(zip(want, actual, strict=True)):
            x, y = a.split(), b.split()
            assert x[0] == y[0], (i, a, b)
            assert all(math.isclose(v, w, abs_tol=2e-12, rel_tol=2e-12)
                       for v, w in zip(map(float, x[1:]), map(float, y[1:]), strict=True)), (i, a, b)
            stats[x[0]] = stats.get(x[0], 0) + 1
        print(f"Matched fixture: {stats}")


if __name__ == "__main__":
    main()
