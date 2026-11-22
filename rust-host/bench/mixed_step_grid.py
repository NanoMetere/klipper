#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Match staged Rust motor positions to stock C after fractional barriers."""
import argparse
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile

from machine_trace import CONFIG, traced
from planner import PIN, ROOT


def files():
    yield ["G91","G1 X.006 F300","M109 S200","G1 X.006"]
    rng=random.Random(815891)
    for i in range(72):
        code=["G91"]
        for j in range(3+i%13):
            a,b,c=(rng.randrange(-18,19)/1000. for _ in range(3))
            code.extend([f"G1 X{a:.4f} Y{b:.4f} Z{c:.4f} F300",
                         "M109 S200" if j%2==0 else "M104 S215"])
        yield code


def net(records):
    out, direction, count = [], 0, 0
    for row in records:
        a,*fields = row.split()
        if a=="d":
            direction=1 if int(fields[0]) else -1
        elif a=="s":
            count += direction*int(fields[1])
        else:
            assert a=="end",row
            out.append(count)
            direction,count=0,0
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream",type=Path,help=f"checkout at {PIN}")
    p.add_argument("--binary",type=Path,required=True)
    a=p.parse_args()
    up=a.upstream.resolve()
    sys.path.insert(0,str(up/"klippy"))
    h=up/"klippy/chelper"
    with tempfile.TemporaryDirectory() as d:
        d=Path(d)
        c=d/"reference"
        subprocess.run([os.environ.get("CC","cc"),"-O3","-I",h,ROOT/"bench/machine_trace.c",
                          *(h/(n+".c") for n in ("itersolve","stepcompress","trapq","kin_cartesian",
                            "kin_corexy","kin_extruder","kin_shaper","pyhelper","msgblock")),
                          "-lm","-o",c],check=True)
        cases=list(files())
        text="\n--case--\n".join("\n".join(lines) for lines in cases)
        for shaped in (False,True):
            setting=CONFIG.format(kinematics="corexy")
            if shaped:
                setting+="\n[input_shaper]\nshaper_freq_x: 56.6\nshaper_type_y: ei\nshaper_freq_y: 48.4\n"
            cfg=d/"fixture.cfg"
            cfg.write_text(setting)
            got=subprocess.check_output([a.binary,cfg],input=text,text=True).splitlines()
            for idx,(g,code) in enumerate(zip(got,cases,strict=True)):
                stock=[line if not line.startswith("M10") else "M400\nG4 P550" for line in code]
                directions,_,_=traced(cfg,c,"\n".join(stock),False,False)
                assert list(map(int,g.split()))==net(directions),(shaped,idx,g,net(directions),code)
            print(f"{len(cases)} phase sequences: signed stock step positions match (shaped={shaped})")


if __name__=="__main__":
    main()
