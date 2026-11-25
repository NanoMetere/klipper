#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Pinned offline stock/C/Rust trace parity for dynamic shared pressure smoothing."""
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
                || !(smooth >= 0. && smooth <= .2) || (amount == 0. && smooth != 0.))
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
        assert raw.count('command.get_float("ADVANCE")') == 1
        # The previous fixture's observer, not stock, required ADVANCE.
        entry.write_text(raw.replace(marker, HOOK+"\n"+marker)
                         .replace('gain=gain, smooth=smooth','gain=gain, smooth=smooth if gain else 0.')
                         .replace('command.get_float("ADVANCE")','value.pressure_advance'))
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
        pa = "SET_PRESSURE_ADVANCE EXTRUDER=shared_extruder "
        # expected (gain, effective smooth) in stock callback application order.
        cases = (
            ("positive-retune", .04, ("ADVANCE=.06 SMOOTH_TIME=.075",),
             ("ADVANCE=.085 SMOOTH_TIME=.025",), ((.04,.04),(.06,.075),(.085,.025))),
            ("smooth-only", .04, ("SMOOTH_TIME=.075",),
             ("SMOOTH_TIME=.075",), ((.04,.04),(.04,.075),(.04,.075))),
            ("zero-remembers-configured", 0., ("SMOOTH_TIME=.09","ADVANCE=.06"),
             ("ADVANCE=.085",), ((0.,0.),(0.,0.),(.06,.09),(.085,.09))),
            ("pending-longer-window", .04, ("ADVANCE=.055",),
             ("ADVANCE=.085 SMOOTH_TIME=.025",),
             ((.04,.04),(.055,.04),(.06,.04),(.06,.2),(.085,.025))),
            ("positive-zero-smooth", .04, ("ADVANCE=.06 SMOOTH_TIME=0",),
             ("ADVANCE=.085 SMOOTH_TIME=.105",), ((.04,.04),(.06,0.),(.085,.105))),
        )
        for label, initial, fwd, back, expected in cases:
            base = cfgbase.replace("pressure_advance: 0.04", f"pressure_advance: {initial}", 1)
            def change(queue, commands, shape):
                original = pickup(queue, 0., shape, True)
                return original.replace(pa+"ADVANCE=0.0\n", "".join(pa+c+"\n" for c in commands), 1)
            script = (START+f"BED_MESH_PROFILE LOAD={NAME}\nG90\nM83\n"
                      "G1 X21 Y17 Z1.2 E.5 F780\nG1 X31 Y22 Z1.2 E.7 F840\n"
                      +change("extruder1", fwd, PROFILE1)
                      +"G1 X49 Y39 Z1.4 E.8 F960\nG1 X28 Y50 Z1.25 E.91 F840\nG1 E-.27 F840\n"
                      +change("extruder", back, PROFILE0)
                      +"G1 X21 Y23 Z1.7 E.6 F900\nG1 X19 Y31 Z1.7 E.72 F830\nM400\n")
            script = script.replace(PROFILE1, f"BED_MESH_PROFILE LOAD={ALT}\n"+PROFILE1, 1)
            script = script.replace(PROFILE0, f"BED_MESH_PROFILE LOAD={NAME}\n"+PROFILE0, 1)
            if label == "pending-longer-window":
                marker = "G1 X49 Y39 Z1.4 E.8 F960\n"
                inserted = pa + "ADVANCE=.06\n" + pa + "SMOOTH_TIME=.2\n"
                script = script.replace(marker, marker + inserted, 1)
            g.write_text(script)
            for tol in (0,int(hz*.000025)):
                cfg.write_text(base + explicit)
                rows,gold=run_stock(stock,dictionary,f,t,g,tol==0)
                _,repeat=run_stock(stock,dictionary,f,t,g,tol==0); assert gold==repeat
                meta,=rows_with_prefix(rows,'XYZ_BIND'); assert meta['max_error']==tol
                eid,={q['oid'] for q in rows_with_prefix(rows,'SHARED_OUTCOME')}
                lanes=[flatten_file(gold[f'oid={j}']) for j in [*meta['oids'],eid]]
                w=rows_with_prefix(rows,'WINDOW'); applied = [q for q in w if q['op']=='p']
                assert tuple((q['gain'],q['smooth']) for q in applied) == expected, (label, applied)
                shapes=rows_with_prefix(rows,'SHAPE_STATE'); assert len(shapes)==3 and shapes[0]==shapes[2]!=shapes[1]
                i=next(i for i,q in enumerate(w) if q['op']=='p')
                x.write_text(shape_rows(rows));y.write_text(timeline(w[i:],dict(ATTRS,p='at gain smooth')))
                expect=''.join('\n'.join([*rows,'end','']) for rows,_ in lanes).encode()
                c_args=[x,y,str(hz),str(tol),*map(str,meta['d']),*(str(int(v)) for v in meta['inverted'])]
                for count in ('1','4'): assert subprocess.check_output([exe,count,*c_args])==expect
                md["initial_pressure_advance"] = initial
                md["max_error_ticks"] = tol
                m.write_text(json.dumps(md))
                rust_args = [cli, "--dynamic-pressure", m, g, settings]
                data = subprocess.check_output([*rust_args, meshfile, *names], timeout=10)
                actual = wire_records(data, hz)
                wanted = [records for records, _ in lanes]
                for lane, (a, b) in enumerate(zip(actual, wanted)):
                    assert a == b, (label, tol, lane, len(a), len(b),
                        next(((i,x,y) for i,(x,y) in enumerate(zip(a,b)) if x!=y), None))
                for path in (savedfile, mixedfile):
                    cfg.write_text(base + path.read_text())
                    _, saved_gold = run_stock(stock, dictionary, f, t, g, tol == 0)
                    assert saved_gold == gold, (label, tol, path.name)
                    assert data == subprocess.check_output(
                        [*rust_args, path, *names], timeout=10)
                cfg.write_text(base + explicit)
                print(label, tol, sum(a for _,a in lanes),
                      "steps; exact stock, pinned C (1/4), and Rust (three formats)", flush=True)
                if args.rounds:
                    arm = [sys.executable, Path(__file__).resolve(), "--stock-arm",
                           stock, dictionary, f, m, g]
                    paired([arm, [*rust_args, meshfile, *names]], t/"output.json", data, args.rounds)
        valid = g.read_text()
        p = pa + "ADVANCE=.06 SMOOTH_TIME=0"
        assert p in valid
        malformed = ("SMOOTH_TIME=NaN", "SMOOTH_TIME=inf", "SMOOTH_TIME=.2001",
                     "SMOOTH_TIME=-0.01", "SMOOTH_TIME=.000000001",
                     "SMOOTH_TIME=.04 SMOOTH_TIME=.05", "SMOOTH_TIME=", "ADVANCE=-.1",
                     "ADVANCE=NaN", "SMOOTH_TIME=.04 FOREIGN=1")
        for change in malformed:
            g.write_text(valid.replace(p, pa+change, 1))
            result = subprocess.run([*rust_args, meshfile, *names],
                                    timeout=10, capture_output=True)
            assert result.returncode == 2 and not result.stdout, change
        wrong = p.replace("EXTRUDER=shared_extruder", "EXTRUDER=extruder1")
        g.write_text(valid.replace(p, wrong, 1))
        result = subprocess.run([*rust_args, meshfile, *names], timeout=10, capture_output=True)
        assert result.returncode == 2 and not result.stdout
        g.write_text(valid)
        for bad_initial in (0., .21):
            invalid = dict(md, smooth_time_s=bad_initial)
            m.write_text(json.dumps(invalid))
            result = subprocess.run([*rust_args, meshfile, *names], timeout=10, capture_output=True)
            assert result.returncode == 2 and not result.stdout, bad_initial
        m.write_text(json.dumps(md))
        result = subprocess.run([cli, m, g, settings, meshfile, *names],
                                timeout=10, capture_output=True)
        assert result.returncode == 2 and not result.stdout, "Dynamic retuning must be opt-in"
        print("10 runtime retune cases with exact outputs; 14 negative/opt-in controls")



if __name__ == "__main__":
    main(sys.argv[1:])
