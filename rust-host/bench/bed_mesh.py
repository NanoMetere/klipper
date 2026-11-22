#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare synthetic mesh grid evaluation, fade inversion and moves against stock."""
import argparse
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace

from planner import PIN, ROOT


def corpus():
    rng = random.Random(85083)
    for i in range(84):
        algo = i%3
        x, y = rng.choice([3,4,5,6]), rng.choice([3,4,5,6])
        bounds = [rng.uniform(-20,30), rng.uniform(-20,30), rng.uniform(180,420), rng.uniform(180,420)]
        pps = (0,0) if algo==0 else (rng.randrange(5),rng.randrange(5))
        tension=rng.choice([.08,.2,.5,1.0])
        yield ("m", algo, *bounds, x, y, *pps, tension, *(rng.uniform(-.24,.27) for _ in range(x*y)))
        yield ("o", rng.uniform(-8,8),rng.uniform(-8,8))
        for _ in range(42):
            yield ("z", rng.uniform(-100,460),rng.uniform(-100,460))
        faded=int(i%4!=0)
        yield ("f", faded, .8, 4., 0.0, rng.uniform(-.2,.2), .025 if i%2 else .01, 5. if i%2 else 3.)
        for _ in range(38):
            here=[rng.uniform(-40,400),rng.uniform(-40,400),rng.uniform(-.3,5.),rng.uniform(-2,2)]
            yield ("g", *here)
            there=[rng.uniform(-40,400),rng.uniform(-40,400),rng.uniform(-.3,5.),rng.uniform(-2,2)]
            yield ("s",*here,*there)


def stock(rows):
    from extras.bed_mesh import ZMesh, MoveSplitter, BedMesh
    out=[]
    for op,*v in rows:
        if op=="m":
            algo, xmin, ymin, xmax, ymax, nx, ny, ppsx, ppsy, tension, *z=v
            args=dict(min_x=xmin,min_y=ymin,max_x=xmax,max_y=ymax,x_count=nx,
                      y_count=ny,mesh_x_pps=ppsx,mesh_y_pps=ppsy,algo=("direct","lagrange","bicubic")[algo],
                      tension=tension)
            mesh=ZMesh(args,"synthetic")
            mesh.build_mesh([z[i:i+nx] for i in range(0,len(z),nx)])
        elif op=="o":
            mesh.set_mesh_offsets(v)
        elif op=="z":
            out.append(("z",mesh.calc_z(*v)))
        elif op=="f":
            enabled,start,end,target,offset,zcheck,dcheck=v
            if not enabled:
                start=end=BedMesh.FADE_DISABLE
                target=0.
            temp=SimpleNamespace(z_mesh=mesh,fade_start=start,fade_end=end,
                                 fade_dist=end-start,fade_target=target,tool_offset=offset,
                                 last_position=[0.0]*4)
            getter=lambda key, default, **kw: {"split_delta_z":zcheck,"move_check_distance":dcheck}[key]
            splitter=MoveSplitter(SimpleNamespace(getfloat=getter),None)
            splitter.initialize(mesh,target)
        elif op=="g":
            temp.toolhead=SimpleNamespace(get_position=lambda: v)
            out.append(("g",*BedMesh.get_position(temp)))
        else:
            a,b=v[:4],v[4:]
            factor=BedMesh.get_z_factor(temp,b[2])
            results=[]
            if factor:
                splitter.build_move(a,b,factor)
                while not splitter.traverse_complete:
                    results.append(splitter.split())
            else:
                b[2]+=target
                results.append(b)
            out.append(("s",len(results)))
            out.extend(("p",*row) for row in results)
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream",type=Path,help=f"checkout at {PIN}")
    a=p.parse_args()
    sys.path.insert(0,str(a.upstream.resolve()/"klippy"))
    rows=list(corpus())
    data="\n".join(" ".join(map(str,r)) for r in rows)+"\n"
    want=stock(rows)
    with tempfile.TemporaryDirectory() as d:
        t=Path(d)
        lib,r=t/"libklipper_host_protocol.rlib",t/"rust"
        subprocess.run(["rustc","--edition=2021","-O","--crate-type=rlib","--crate-name=klipper_host_protocol",
                        ROOT/"src/lib.rs","-o",lib],check=True)
        subprocess.run(["rustc","--edition=2021","-O",ROOT/"bench/bed_mesh.rs","--extern",
                        f"klipper_host_protocol={lib}","-o",r],check=True)
        got=subprocess.check_output([r],input=data,text=True).splitlines()
        counts={}
        error=0.
        for i,(x,y) in enumerate(zip(want,got,strict=True)):
            kind,*v=x
            label,*w=y.split()
            assert kind==label,(i,x,y)
            counts[kind]=counts.get(kind,0)+1
            for f,g in zip(v,map(float,w),strict=True):
                assert math.isclose(f,g,rel_tol=2e-12,abs_tol=2e-12),(i,x,y)
                error=max(error,abs(f-g))
        print(f"{counts}; maximum numeric difference {error:g} mm")


if __name__=="__main__":
    main()
