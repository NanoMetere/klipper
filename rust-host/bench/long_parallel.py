#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Check complete Rust CLI and C role-equivalent output with 1–4 workers."""
import argparse
import json
import math
import os
import random
import resource
import statistics as st
import subprocess as sp
import sys
import tempfile
import time
from pathlib import Path
from hybrid_process_timing import CFG, SHAPED_EXTRUSION_CFG, gcode
from long_step_preview import capture
from machine_trace import traced
from planner import PIN, ROOT


def build(stock, destination, source):
    helpers = stock / "klippy/chelper"
    units = ("itersolve", "stepcompress", "trapq", "kin_cartesian", "kin_corexy",
             "kin_extruder", "kin_shaper", "pyhelper", "msgblock")
    sp.run([os.environ.get("CC", "cc"), "-O3", "-pthread", "-I", helpers,
            ROOT / "bench" / source, *(helpers / f"{n}.c" for n in units),
            "-lm", "-o", destination], check=True)

def cpu():
    ref=resource.getrusage(resource.RUSAGE_CHILDREN)
    return ref.ru_utime+ref.ru_stime

def timed(label, arms, want, diff_file, rounds, source=None):
    vals=[]
    cpus=[]
    rng=random.Random(899297)
    for i in range(-3,rounds):
        order=[(i+j)%len(arms) for j in range(len(arms))]
        samples=[0]*len(arms)
        loads=[0]*len(arms)
        for j in (order if i%2==0 else order[::-1]):
            # Vary the unmeasured gap so successive launches do not align
            # with an external scheduler's periodic accounting interval.
            time.sleep(rng.uniform(.002, .012))
            was_cpu=cpu()
            started=time.perf_counter_ns()
            with diff_file.open("wb") as out:
                sp.run(arms[j],stdout=out,input=source,check=True,timeout=25,stderr=sp.DEVNULL)
            samples[j]=time.perf_counter_ns()-started
            loads[j]=cpu()-was_cpu
            assert diff_file.read_bytes()==want,(label,i,j)
        if i>=0:
            vals.append(samples)
            cpus.append(loads)
    ms=[round(st.median(x)/1e6,3) for x in zip(*vals)]
    delta=lambda pairs:[round(100*st.median(x[j]/x[0]-1 for x in pairs),2)
                        for j in range(1,len(arms))]
    print(label,rounds,"exact pairs; median ms",ms,"paired wall delta",delta(vals),
          "CPU delta",delta(cpus),flush=True)



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("stock", "serial", "parallel"):
        parser.add_argument(name, type=lambda p: Path(p).resolve())
    parser.add_argument("--rounds", type=int, default=0)
    parser.add_argument("--cpu-mask", help="comma-separated logical CPUs")
    args = parser.parse_args()
    if not 0 <= args.rounds <= 80:
        parser.error("--rounds must be 0–80")
    if args.cpu_mask:
        os.sched_setaffinity(0, set(map(int, args.cpu_mask.split(","))))
    stock, old, new, rounds = args.stock, args.serial, args.parallel, args.rounds
    mask = args.cpu_mask or "default"
    assert sp.check_output(["git", "-C", stock, "rev-parse", "HEAD"], text=True).strip() == PIN
    sp.run(["git", "-C", stock, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(stock / "klippy"))
    with tempfile.TemporaryDirectory(prefix="parallel-long-trace-") as td:
        root=Path(td)
        cfg,path,out,native=(root/p for p in ("printer.cfg","job.gcode","out.json","native.in"))
        seq_c, mult_c = root / "stock", root / "parallel"
        build(stock, seq_c, "machine_trace.c")
        build(stock, mult_c, "long_parallel.c")
        controls=("max_extrude_only_distance: 15\nmax_extrude_only_velocity: 20\n"
                  "max_extrude_only_accel: 300")
        shape=SHAPED_EXTRUSION_CFG.replace("pressure_advance: 0.04","pressure_advance: 0.04\n"+controls)
        cfg.write_text(CFG+shape)
        mode=["--long-step-preview"]
        base=[root,cfg.name,path,"20000000","10","10","1"]
        reverse = ["G90", "M83"]
        for i in range(360):
            reverse.append(f"G1 X{10+i%2*.125} Y{12-i%2*.111} Z1 E.012 F1800")
            if i % 45 == 0:
                reverse.extend(("G4 P32", "G1 E-.014 F800", "G1 E.014 F800"))
        reverse.append("M400")
        for which, moves, extrusion in (("layered",220,True),("layered",3000,True),
            ("shaped-extrusion",3000,True),("xyz",2500,False),("reversal-dwell",360,True)):
            script = ("\n".join(reverse) if which == "reversal-dwell" else
                      "\n".join(tuple(gcode(moves,which,0))[1:])) + "\n"
            path.write_text(script)
            opts=["--tool-temps","0:210","--fixed-pressure-script"] if extrusion else []
            params=[*mode,*opts,*base]
            want=sp.check_output([old,*params],timeout=15)
            labels=[f"{which}-{moves}",mask]
            for opt in ([],["--lane-workers","1"],["--lane-workers","2"],["--lane-workers","4"]):
                got=sp.check_output([new,*opt,*params],timeout=15)
                assert got==want,(*labels,opt)
            for count in (0,5,8):
                invalid=sp.run([new,"--lane-workers",str(count),*params],capture_output=True,timeout=10)
                assert invalid.returncode and not invalid.stdout,(*labels,count)
            off=sp.run([old,"--lane-workers","4",*params],capture_output=True,timeout=10)
            assert off.returncode and not off.stdout
            lines,duration,_=traced(cfg, seq_c, script, extrusion, False, preview_cli=True, native_plan=native)
            expected=("\n".join(lines)+"\n").encode()
            payload=native.read_bytes()
            for c in (1,2,4):
                assert sp.check_output([mult_c,str(c)],input=payload,stderr=sp.DEVNULL,timeout=15)==expected,(*labels,c)
            data=json.loads(want)
            assert math.isclose(data["planned_duration_seconds"],duration,rel_tol=1e-12)
            trace=data["motors"]+([data["extruder_lanes"][0]["commands"]] if extrusion else [])
            assert lines==capture(trace),(*labels,"cross-language")
            print(*labels,len(lines)-len(trace),"exact C and Rust records for all workers",flush=True)
            if rounds:
                timed("Rust "+labels[0], [[new,"--lane-workers",str(n),*params] for n in (1,2,4)], want, out, rounds)
                timed("C "+labels[0], [[mult_c,str(n)] for n in (1,2,4)], expected, out, rounds, payload)
    sp.run(["git", "-C", stock, "diff", "--quiet", "HEAD", "--"], check=True)


if __name__ == "__main__":
    main()
