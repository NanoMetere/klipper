#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline trigger projection/aggregation with both stock Python probe helpers."""
import argparse
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace

from planner import PIN, ROOT


def fixture():
    r=random.Random(31862)
    for i in range(700):
        n=r.randrange(1,15)
        retry=r.randrange(5)
        tolerance=r.choice([0.,.025,.075,.50])
        yield ("r",i%2,n,tolerance,retry,r.uniform(-15.,15.),r.uniform(-15.,15.),r.uniform(-2.,1.))
        for j in range(n*(retry+1)):
            if j==3 and i%13==0:
                yield ("x",)
            yield ("s",r.uniform(10,27),r.uniform(65,80),.65+r.choice([0.,.01,.02,.06,.11,.4]))


def stock(root):
    sys.path.insert(0,str(root/"klippy"))
    from gcode import CommandError
    from extras import probe, manual_probe
    cls=getattr(probe,"SampleAveragingHelper",None) or probe.ProbeSessionHelper

    def case(params,samples):
        mode,n,eps,r,*offsets=params
        active=object.__new__(cls)
        active.hw_probe_session=True
        active.results=[]
        active.param_helper=SimpleNamespace(get_probe_params=lambda _: dict(
            samples=int(n),samples_tolerance=eps,samples_tolerance_retries=int(r),
            samples_result=("average","median")[int(mode)],sample_retract_dist=2.,lift_speed=4.))
        tool=SimpleNamespace(get_position=lambda:[10.,20.,.5,0.],manual_move=lambda *_:None)
        active.printer=SimpleNamespace(lookup_object=lambda _: tool)
        reads=retries=0
        pending=iter(samples)

        def read(_):
            nonlocal reads
            reads+=1
            msg=next(pending)
            if msg[0]=="x":
                raise CommandError("virtual timeout")
            if hasattr(manual_probe, "create_probe_result"):
                return manual_probe.create_probe_result(msg[1:],offsets)
            fake=SimpleNamespace(x_offset=offsets[0],y_offset=offsets[1],z_offset=offsets[2])
            return probe.ProbeOffsetsHelper.create_probe_result(fake,msg[1:])
        def note(_):
            nonlocal retries
            retries+=1
        active._probe=read
        cmd=SimpleNamespace(error=CommandError,respond_info=note)
        try:
            active.run_probe(cmd)
        except CommandError as e:
            label="timeout" if str(e)=="virtual timeout" else "tolerance"
            return (label,reads,retries)
        return ("done",reads,retries,*active.results[0])

    numbered=[]
    params=None
    for line in sys.stdin:
        op,*fields=line.split()
        if op=="r":
            if params is not None:
                print(*case(params,numbered))
            params=list(map(float,fields))
            numbered=[]
        else:
            numbered.append((op,*map(float,fields)))
    if params is not None:
        print(*case(params,numbered))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream",type=lambda s:Path(s).resolve(),help=f"checkout at {PIN}")
    p.add_argument("--legacy",type=lambda s:Path(s).resolve())
    a=p.parse_args()
    data="\n".join(" ".join(map(str,row)) for row in fixture())+"\n"
    with tempfile.TemporaryDirectory() as d:
        t=Path(d)
        r,lib=t/"rust",t/"libklipper_host_protocol.rlib"
        subprocess.run(["rustc","--edition=2021","-O","--crate-type=rlib","--crate-name=klipper_host_protocol",
                        ROOT/"src/lib.rs","-o",lib],check=True)
        subprocess.run(["rustc","--edition=2021","-O",ROOT/"bench/probe.rs","--extern",
                        f"klipper_host_protocol={lib}","-o",r],check=True)
        got=subprocess.check_output([r],input=data,text=True).splitlines()
        for stock_path in filter(None,(a.upstream,a.legacy)):
            want=subprocess.check_output([sys.executable,__file__,"--stock",stock_path],input=data,text=True).splitlines()
            stats={}
            for i,(x,y) in enumerate(zip(want,got,strict=True)):
                x,y=x.split(),y.split()
                assert x[:3]==y[:3],(i,x,y)
                stats[x[0]]=stats.get(x[0],0)+1
                for j,k in zip(map(float,x[3:]),map(float,y[3:]),strict=True):
                    assert math.isclose(j,k,rel_tol=2e-12,abs_tol=2e-12),(i,x,y)
            print(f"{stock_path.name}: {len(want)} probe outcomes match ({stats})")


if __name__=="__main__":
    if len(sys.argv)==3 and sys.argv[1]=="--stock":
        stock(Path(sys.argv[2]))
    else:
        main()
