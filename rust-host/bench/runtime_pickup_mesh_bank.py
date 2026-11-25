#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Exact offline A/B/A pickup parity when switching distinct saved mesh profiles."""
import argparse
import sys
from pathlib import Path
import json, os, shutil, subprocess, tempfile
from file_debug_lane_timing import rows_with_prefix,timeline
from runtime_shaper import START, CONFIG0, PROFILE0, PROFILE1, HOOK, SCHEMA, shape_rows, wire_records
from file_debug_process_timing import stock_arm
from shaped_file_debug import paired
from runtime_mesh import NAME, SAVED, profiles
from runtime_pickup_mesh import pickup
from shared_window_capture import ROOT, PIN, ATTRS, flatten_file, run_stock
ALT = "alternate_surface"


def rejects(cli, model, gcode, settings, mesh, saved, names):
    scratch = mesh.with_name("invalid.cfg")
    script = gcode.read_text()
    text = mesh.read_text()

    def run(cfg, profiles):
        proc = subprocess.run([cli, model, gcode, settings, cfg, *profiles],
                              timeout=10, capture_output=True)
        assert proc.returncode == 2 and not proc.stdout, (cfg, profiles, proc.stderr)

    for args in ([NAME], [NAME, NAME], [NAME, "unknown"], [NAME, "../escape"],
                 [NAME, "Z" * 65], [NAME, *map(str, range(8))]):
        run(mesh, args)
    for edit in (
        text + "\n[DEFAULT]\nfade_start:0.5\n",
        text + "\n[include *.cfg]\n",
        text + "\n[extruder]\n",
        text + f"\n[bed_mesh {ALT}]\n",
        text + "\n[bed_mesh]\n",
        text.replace(f"[bed_mesh {ALT}]", "[bed_mesh unauthorized]", 1),
        text.replace("version: 1", "version: 0", 1),
        saved.read_text() + "#*# [bed_mesh unauthorized]\n",
        saved.read_text() + "#*# [include *.cfg]\n",
        saved.read_text() + f"#*# [bed_mesh {NAME}]\n",
        saved.read_text() + "#*# [bed_mesh]\n",
        saved.read_text().replace(SAVED, SAVED.replace("DO NOT EDIT", "DO NOT USE"), 1),
        text + SAVED + f"#*# [bed_mesh {NAME}]\n",
    ):
        scratch.write_text(edit)
        run(scratch, names)
    load = f"BED_MESH_PROFILE LOAD={ALT}"
    assert load in script
    for replacement in ("BED_MESH_PROFILE LOAD=unknown", f"BED_MESH_PROFILE LOAD={ALT.upper()}",
                        f"BED_MESH_PROFILE LOAD={ALT} LOAD={NAME}", f"{load} Z=1",
                        f"BED_MESH_PROFILE SAVE={ALT}", f"BED_MESH_PROFILE REMOVE={ALT}",
                        "BED_MESH_CALIBRATE", "BED_MESH_OFFSET X=NaN",
                        "BED_MESH_CLEAR X=1"):
        gcode.write_text(script.replace(load, replacement, 1))
        run(mesh, names)
    gcode.write_text(script)


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
        cfg=f/'printer.cfg'; mesh, _=profiles()
        head, tail = mesh.split('[bed_mesh test_surface]', 1)
        second = '[bed_mesh alternate_surface]' + tail
        start = second.index('points:')
        end = second.index('x_count:',start)
        second = second[:start]+"points:\n    -0.05, 0.05, 0.12\n    0.01, 0.02, 0.09\n    0.08, 0.12, 0.19\n"+second[end:]
        cfgbase = cfg.read_text() + CONFIG0 + "\n"
        explicit = mesh + "\n" + second
        saved_profile = lambda v: "".join(f"#*# {line}\n" for line in v.splitlines())
        saved = head + SAVED + saved_profile(f"[bed_mesh {NAME}]" + tail + "\n" + second)
        mixed = mesh + "\n" + SAVED + saved_profile(second)
        meshfile = t/"mesh.cfg"; meshfile.write_text(explicit)
        savedfile = t/"saved.cfg"; savedfile.write_text(saved)
        mixedfile = t/"mixed.cfg"; mixedfile.write_text(mixed)
        names = [NAME, ALT]

        md=json.loads((f/'offline_model.json').read_text()); hz=md['clock_hz']
        g,settings,m,x,y=[t/s for s in ('input.gcode','shape.cfg','model.json','xyz.txt','e.txt')]
        settings.write_text(CONFIG0)
        for label,initial,fwd,back,offset in (('positive',.04,.06,.085,False),('zero',0,0,0,False),('crossing',.04,0,.085,False),('offset-crossing',.04,0,.085,True)):
            base = cfgbase.replace('pressure_advance: 0.04',f'pressure_advance: {initial}',1)
            md['initial_pressure_advance']=initial
            g.write_text(START+'G90\nM83\nG1 X21 Y17 Z1.2 E.5 F780\nG1 X31 Y22 Z1.2 E.7 F840\n'+pickup('extruder1',fwd,PROFILE1,offset)+'G1 X49 Y39 Z1.4 E.8 F960\nG1 X28 Y50 Z1.25 E.91 F840\nG1 E-.27 F840\n'+pickup('extruder',back,PROFILE0,offset)+'G1 X21 Y23 Z1.7 E.6 F900\nG1 X19 Y31 Z1.7 E.72 F830\nM400\n')
            program=g.read_text()
            program=program.replace("G90\n", f"BED_MESH_PROFILE LOAD={NAME}\nG90\n", 1)
            program=program.replace(PROFILE1, "BED_MESH_PROFILE LOAD=alternate_surface\nBED_MESH_OFFSET X=.5 Y=-.3 ZFADE=.22\n"+PROFILE1, 1)
            program=program.replace(PROFILE0, PROFILE0+f"BED_MESH_CLEAR\nG1 Z3.7 F600\nBED_MESH_PROFILE LOAD={NAME}\n", 1)
            g.write_text(program)
            for tol in (0,int(hz*.000025)):
                cfg.write_text(base + explicit)
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
                other = t/"one-profile.gcode"
                other.write_text(program.replace("LOAD=alternate_surface","LOAD=test_surface"))
                _, onemesh = run_stock(stock,dictionary,f,t,other,tol==0)
                assert onemesh != gold, (label, tol)
                rust_args = [cli, m, g, settings]
                rust = subprocess.check_output([*rust_args, meshfile, *names], timeout=10)
                assert wire_records(rust,hz) == [packets for packets, _ in lanes], (label, tol)
                for path, config_text in ((savedfile, saved), (mixedfile, mixed)):
                    cfg.write_text(base + config_text)
                    _, saved_gold = run_stock(stock,dictionary,f,t,g,tol==0)
                    assert gold == saved_gold, (label, tol, path.name, "stock")
                    assert rust == subprocess.check_output(
                        [*rust_args, path, *reversed(names)], timeout=10), (label, tol, path.name, "Rust")
                print(label, tol, sum(a for _,a in lanes),
                      "steps; exact stock, C (1/4), and Rust (explicit/SAVE_CONFIG/mixed)", flush=True)
                if args.rounds:
                    cfg.write_text(base + explicit)
                    arm = [sys.executable, Path(__file__).resolve(), "--stock-arm",
                           stock, dictionary, f, m, g]
                    paired([arm, [*rust_args, meshfile, *names]], t/"output.json", rust, args.rounds)
        rejects(cli, m, g, settings, meshfile, savedfile, names)
        print("8 two-profile cases across three formats; 28 invalid invocations refused")


if __name__ == "__main__":
    main(sys.argv[1:])
