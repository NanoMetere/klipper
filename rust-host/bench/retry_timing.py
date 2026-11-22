#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare stock adaptive retry timer states using an isolated socketpair."""
import argparse
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile

from planner import PIN, ROOT


def fixture():
    rng = random.Random(73012)
    for i in range(78):
        wire, speed = [(0,0), (1,250000), (2,1000000)][i % 3]
        yield ("n", wire, speed)
        now = 1.
        pending = 0
        for _ in range(240):
            if pending == 0 or (pending < 12 and rng.randrange(3) == 0):
                yield ("s", rng.randrange(5,65), now + rng.uniform(0, .003))
                pending += 1
            elif rng.randrange(4) == 0:
                yield ("r", int(rng.randrange(3)==0), now)
            else:
                n = rng.randint(1, pending)
                yield ("a", n, now)
                pending -= n
            now += rng.choice([.0001, .002, .02, .13, .25, 1.])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    a = p.parse_args()
    h = a.upstream.resolve()/"klippy/chelper"
    data = "\n".join(" ".join(map(str,r)) for r in fixture())+"\n"
    with tempfile.TemporaryDirectory() as d:
        t=Path(d)
        c,r,lib=t/"c",t/"rust",t/"libklipper_host_protocol.rlib"
        subprocess.run([os.environ.get("CC","cc"),"-O3","-ffunction-sections","-fdata-sections",
                        "-Wl,--gc-sections","-pthread","-I",h,ROOT/"bench/retry_timing.c",
                        h/"msgblock.c",h/"pyhelper.c",h/"pollreactor.c","-lm","-o",c],check=True)
        subprocess.run(["rustc","--edition=2021","-O","--crate-type=rlib","--crate-name=klipper_host_protocol",
                        ROOT/"src/lib.rs","-o",lib],check=True)
        subprocess.run(["rustc","--edition=2021","-O",ROOT/"bench/retry_timing.rs",
                        "--extern",f"klipper_host_protocol={lib}","-o",r],check=True)
        x,y=[subprocess.check_output([exe],input=data,text=True,timeout=20).splitlines() for exe in (c,r)]
        max_diff=0.
        for i,(first,second) in enumerate(zip(x,y,strict=True)):
            for j,k in zip(map(float,first.split()),map(float,second.split()),strict=True):
                assert math.isclose(j,k,abs_tol=2e-12,rel_tol=2e-12),(i,first,second)
                max_diff=max(max_diff,abs(j-k))
        print(f"{len(x)} timer/mean/variance states; maximum absolute difference {max_diff:g}s")


if __name__=="__main__":
    main()
