#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline arc-to-line and motion semantics with real stock G-code handlers."""
import argparse
from contextlib import redirect_stdout
from io import StringIO
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile

from gcode_preview import stock
from planner import PIN, ROOT


def corpus():
    yield from ("G17", "G90", "M82", "G0 X1 Y0 Z0 E0 F1200", "G3 X0 Y1 Z.2 I-1 J0 E.25",
                "G2 X0 Y1 Z.3 I0 J-1 E.44", "G18",
                "G3 X1 Z.2 Y2 I0 K-1", "G19", "M83",
                "G2 Y0 Z1 X2 J-1 K.05 E.5", "M400")
    rng = random.Random(47543)
    for j in range(145):
        axes = ("XYZ", "XZY", "YZX")[j % 3]
        offsets = ("IJ", "IK", "JK")[j % 3]
        yield f"G{17 + j%3}"
        yield "M82" if j % 2 else "M83"
        yield "G1 X20 Y20 Z20 F1200"
        radius = rng.choice([.05, .33, .72, 2., 5., 11.])
        yield f"G0 {axes[0]}{20 + radius}"
        angle = (0., .45, math.pi*.8, -1.1, math.pi)[rng.randrange(5)]
        target = [radius*math.cos(angle), radius*math.sin(angle), rng.uniform(-2., 2.)]
        xy = " ".join(f"{k}{20+v:.12f}" for k,v in zip(axes,target))
        yield f"G{2+j%2} {xy} {offsets[0]}{-radius} {offsets[1]}0 E{rng.random()*2} F{rng.randrange(500,1800)}"
        if j % 3:
            yield "G92 E0"
        if j % 5 == 0:
            yield "M400"
    yield "M400"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    a = p.parse_args()
    sys.path.insert(0,str(a.upstream.resolve()/"klippy"))
    lines = "\n".join(corpus())+"\n"
    out = StringIO()
    with redirect_stdout(out):
        stock(lines.splitlines(),True,arc_resolution=.28)
    want = out.getvalue().splitlines()
    with tempfile.TemporaryDirectory() as d:
        t=Path(d)
        lib,rust=t/"libklipper_host_protocol.rlib",t/"rust"
        subprocess.run(["rustc","--edition=2021","-O","--crate-type=rlib","--crate-name=klipper_host_protocol",
                        ROOT/"src/lib.rs","-o",lib],check=True)
        subprocess.run(["rustc","--edition=2021","-O",ROOT/"bench/gcode_preview.rs","--extern",
                        f"klipper_host_protocol={lib}","-o",rust],check=True)
        got=subprocess.check_output([rust,"--arcs"],input=lines,text=True).splitlines()
        counts={}
        worst=0.
        for i,(x,y) in enumerate(zip(want,got,strict=True)):
            p,q=x.split(),y.split()
            assert p[0]==q[0],(i,x,y)
            counts[p[0]]=counts.get(p[0],0)+1
            for v,w in zip(map(float,p[1:]),map(float,q[1:]),strict=True):
                assert math.isclose(v,w,abs_tol=2e-10,rel_tol=2e-11),(i,x,y)
                worst=max(worst,abs(v-w))
        print(f"{counts}; maximum absolute numeric difference {worst:g}")


if __name__=="__main__":
    main()
