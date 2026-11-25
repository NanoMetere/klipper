#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Synthetic offline A/B/A pickup and reference pressure-advance replay."""
import argparse
import sys
from pathlib import Path
import json, os, shutil, subprocess, tempfile
from file_debug_lane_timing import rows_with_prefix,timeline
from runtime_shaper import START, CONFIG0, PROFILE0, PROFILE1, HOOK, SCHEMA, shape_rows, wire_records
from file_debug_process_timing import stock_arm
from shaped_file_debug import paired
from shared_window_capture import ROOT, PIN, ATTRS, flatten_file, run_stock
def pickup(e,gain,shape,offset):
 result=f'ACTIVATE_EXTRUDER EXTRUDER={e}\nSYNC_EXTRUDER_MOTION EXTRUDER=shared_extruder MOTION_QUEUE={e}\nSET_PRESSURE_ADVANCE EXTRUDER=shared_extruder ADVANCE={gain}\n'
 if offset:
  x,y,z=('.0625','-.035','.015') if e=='extruder1' else ('0','0','0')
  result+=f'SET_GCODE_OFFSET X={x} Y={y} MOVE=1 MOVE_SPEED=120\nSET_GCODE_OFFSET Z={z} MOVE=1 MOVE_SPEED=8\n'
 return result+shape


def main(argv):
    if len(argv) == 6 and argv[0] == "--stock-arm":
        stock_arm(*(Path(v).resolve() for v in argv[1:]), schema=SCHEMA)
        return
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("stock", "dictionary", "cli"):
        parser.add_argument(name, type=lambda v: Path(v).resolve())
    parser.add_argument("--rounds", type=int, default=0)
    parser.add_argument("--core", type=int)
    args = parser.parse_args(argv)
    if not 0 <= args.rounds <= 80:
        parser.error("--rounds must be 0–80")
    if args.core is not None:
        os.sched_setaffinity(0, {args.core})
    stock, dictionary, cli = args.stock, args.dictionary, args.cli
    assert subprocess.check_output(["git", "-C", stock, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", stock, "diff", "--quiet", "HEAD", "--"], check=True)
    helper=stock/'klippy/chelper'
    units=('itersolve','trapq','kin_corexy','kin_cartesian','kin_extruder','kin_shaper','stepcompress','msgblock','pyhelper')
    old=(ROOT/'bench/shared_windows.c').read_text()
    a='''            double at, amount;
            if (fscanf(in, "%lf %lf", &at, &amount) != 2)
                return 2;
            extruder_set_pressure_advance(sk, at, amount, .04);'''
    b='''            double at, amount, smooth;
            if (fscanf(in, "%lf %lf %lf", &at, &amount, &smooth) != 3
                || !(smooth == 0. || smooth == .04) || (amount == 0.) != (smooth == 0.))
                return 2;
            extruder_set_pressure_advance(sk, at, amount, smooth);'''
    assert old.count(a)==1
    with tempfile.TemporaryDirectory(prefix='pickup-sideband-') as td:
        t=Path(td); c=t/'windows.c'; exe=t/'capture-c'
        c.write_text(old.replace(a,b))
        subprocess.run(['cc','-O3','-DWIRE','-DCAPTURE_LANES','-DCAPTURE_SHAPE','-pthread','-I',helper,ROOT/'bench/file_debug_capture_lanes.c',c,*(helper/f'{s}.c' for s in units),'-lm','-o',exe],check=True)
        f=t/'fixture'; shutil.copytree(ROOT/'bench/shared_extruder_fixture',f)
        entry=f/'stock_entry.py'; raw=entry.read_text()
        marker='sys.argv = [entry, "--drain-debug-output", stock, "--stock-only", *args]'
        assert raw.count(marker)==1 and raw.count('gain=gain, smooth=smooth')==2
        entry.write_text(raw.replace(marker,HOOK+'\n'+marker).replace('gain=gain, smooth=smooth','gain=gain, smooth=smooth if gain else 0.'))
        cfg=f/'printer.cfg'; raw=cfg.read_text()+CONFIG0
        md=json.loads((f/'offline_model.json').read_text()); hz=md['clock_hz']
        g,settings,m,x,y=[t/s for s in ('input.gcode','shape.cfg','model.json','xyz.txt','e.txt')]
        settings.write_text(CONFIG0)
        for label,initial,fwd,back,offset in (('positive',.04,.06,.085,False),('zero',0,0,0,False),('crossing',.04,0,.085,False),('offset-crossing',.04,0,.085,True)):
            cfg.write_text(raw.replace('pressure_advance: 0.04',f'pressure_advance: {initial}',1))
            md['initial_pressure_advance']=initial
            g.write_text(START+'G90\nM83\nG1 X21 Y17 Z1.2 E.5 F780\nG1 X31 Y22 Z1.2 E.7 F840\n'+pickup('extruder1',fwd,PROFILE1,offset)+'G1 X49 Y39 Z1.4 E.8 F960\nG1 X28 Y50 Z1.25 E.91 F840\nG1 E-.27 F840\n'+pickup('extruder',back,PROFILE0,offset)+'G1 X21 Y23 Z1.7 E.6 F900\nG1 X19 Y31 Z1.7 E.72 F830\nM400\n')
            for tol in (0,int(hz*.000025)):
                rows,gold=run_stock(stock,dictionary,f,t,g,tol==0)
                _,repeat=run_stock(stock,dictionary,f,t,g,tol==0); assert gold==repeat
                meta,=rows_with_prefix(rows,'XYZ_BIND'); assert meta['max_error']==tol
                eid,={q['oid'] for q in rows_with_prefix(rows,'SHARED_OUTCOME')}
                lanes=[flatten_file(gold[f'oid={j}']) for j in [*meta['oids'],eid]]
                w=rows_with_prefix(rows,'WINDOW'); assert [q['gain'] for q in w if q['op']=='p']==[initial,fwd,back]
                shapes=rows_with_prefix(rows,'SHAPE_STATE'); assert len(shapes)==3 and shapes[0]==shapes[2]!=shapes[1]
                i=next(i for i,q in enumerate(w) if q['op']=='p')
                x.write_text(shape_rows(rows));y.write_text(timeline(w[i:],dict(ATTRS,p='at gain smooth')))
                expect=''.join('\n'.join([*rows,'end','']) for rows,_ in lanes).encode()
                c_args=[x,y,str(hz),str(tol),*map(str,meta['d']),*(str(int(v)) for v in meta['inverted'])]
                for count in ('1','4'): assert subprocess.check_output([exe,count,*c_args])==expect
                md['max_error_ticks']=tol; m.write_text(json.dumps(md))
                rust=subprocess.check_output([cli,m,g,settings])
                assert wire_records(rust,hz)==[p for p,_ in lanes],(label,tol)
                print(label,tol,sum(a for _,a in lanes),'steps; exact pinned stock, native C 1/4, independent Rust sideband',flush=True)
                if args.rounds:
                    arm = [sys.executable, Path(__file__).resolve(), "--stock-arm", stock, dictionary, f, m, g]
                    paired([arm, [cli, m, g, settings]], t/"output.json", rust, args.rounds)
            good = g.read_text()
            needle = f"SET_PRESSURE_ADVANCE EXTRUDER=shared_extruder ADVANCE={fwd}"
            for bad in ("ADVANCE=-1", "ADVANCE=NaN", "ADVANCE=0 FOO=1", "ADVANCE=0 SMOOTH_TIME=.1"):
                g.write_text(good.replace(needle, f"SET_PRESSURE_ADVANCE EXTRUDER=shared_extruder {bad}", 1))
                invalid = subprocess.run([cli, m, g, settings], capture_output=True, timeout=10)
                assert invalid.returncode == 2 and not invalid.stdout, (label, bad)
        print("8 stock/Rust/C cases and 16 invalid pressure controls")


if __name__ == "__main__":
    main(sys.argv[1:])
