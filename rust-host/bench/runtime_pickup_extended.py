#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Stock proof of repeated offline A/B/A shared extrusion and mesh pickup."""
import argparse
import sys
from pathlib import Path
import json, os, shutil, subprocess, tempfile, time, statistics, hashlib
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
    parser.add_argument("--arcs", action="store_true", help="exercise bounded arc support")
    parser.add_argument("--fast-json", action="store_true",
                        help="exercise the optional compact writer; time writers, not workers")
    parser.add_argument("--c-json-shadow", action="store_true",
                        help="compare the C writers over the same validated records")
    parser.add_argument("--rounds", type=int, default=0)
    parser.add_argument("--core", type=int)
    parser.add_argument("--parallel-rounds", type=int, default=0)
    parser.add_argument("--affinity", type=int, nargs="+")
    parser.add_argument("--parallel-jsonl", type=Path)
    parser.add_argument("--baseline-cli", type=Path)
    args = parser.parse_args(argv)
    if not 0 <= args.rounds <= 80:
        parser.error("--rounds must be 0–80")
    if args.c_json_shadow and not (args.fast_json and args.parallel_rounds):
        parser.error("--c-json-shadow requires --fast-json and --parallel-rounds")
    if not 0 <= args.parallel_rounds <= 80:
        parser.error("--parallel-rounds must be 0–80")
    if args.parallel_rounds and not args.parallel_jsonl:
        parser.error("--parallel-rounds requires --parallel-jsonl")
    if args.parallel_jsonl and args.parallel_jsonl.exists():
        parser.error("--parallel-jsonl must be a new file")
    if args.core is not None:
        os.sched_setaffinity(0, {args.core})
    if args.affinity:
        os.sched_setaffinity(0, set(args.affinity))
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
        shadow = t / "serialize-c"
        if args.c_json_shadow:
            subprocess.run(["cc", "-std=c11", "-O3", "-Wall", "-Wextra", "-Werror",
                            ROOT/"bench/pickup_json_shadow.c", "-o", shadow], check=True)
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
        arc_flags = ["--arc-resolution=1"] if args.arcs else []
        if args.arcs:
            cfgbase += "\n[gcode_arcs]\nresolution: 1\n"
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
        for rounds in (4, 16):
            base = cfgbase
            initial = .04
            script = START + f"BED_MESH_PROFILE LOAD={NAME}\nG90\nM83\n"
            expected = [(.04,.04)]
            for hop in range(rounds):
                first = ("G1 X21 Y17 Z1.2 E.5 F780\nG1 X31 Y22 Z1.2 E.7 F840\n"
                         + "".join(f"G1 X{(27 if i%2 else 28)+hop*.001:.5f} Y{24+i*.14:.5f} Z1.2 E.019 F960\n"
                                   for i in range(11)))
                second = ("G1 X49 Y39 Z1.4 E.8 F960\nG1 X28 Y50 Z1.25 E.91 F840\n"
                          + "".join(f"G1 X{(27 if i%2 else 28)+hop*.001:.5f} Y{43+i*.14:.5f} Z1.25 E.019 F960\n"
                                    for i in range(11)) + "G1 E-.27 F840\n")
                a = pickup("extruder1", .06, PROFILE1, True)
                b = pickup("extruder", .085, PROFILE0, True)
                a = a.replace(PROFILE1, f"BED_MESH_PROFILE LOAD={ALT}\n"+PROFILE1, 1)
                b = b.replace(PROFILE0, f"BED_MESH_PROFILE LOAD={NAME}\n"+PROFILE0, 1)
                a = a.replace(pa+"ADVANCE=0.06",pa+"ADVANCE=.06 SMOOTH_TIME=.075",1)
                b = b.replace(pa+"ADVANCE=0.085",pa+"ADVANCE=.085 SMOOTH_TIME=.025",1)
                script += first + a + second + b
                expected.extend(((.06,.075),(.085,.025)))
            expected = tuple(expected)
            if args.arcs:
                script += ("G1 X20 Y20 Z1.5 E.25 F830\n"
                           "G17\nG3 X25 Y20 I2.5 J0 Z1.55 E.20 F840\n"
                           "G2 X20 Y20 I-2.5 J0 Z1.50 E.20 F840\n"
                           "G18\nG3 X22 Z1.50 I1 K0 Y20.10 E.07 F420\n"
                           "G19\nG3 Y22.10 Z1.50 J1 K0 X22.10 E.07 F420\nG17\nM400\n")
            else:
                script += "G1 X19 Y31 Z1.7 E.72 F830\nM400\n"
            # Many short motion lines without changing the published 120s cap.
            script = "\n".join(
                line.rsplit(" F", 1)[0]+" F2700" if line.startswith("G1 X") else line
                for line in script.splitlines())+"\n"
            g.write_text(script)
            label = f"{rounds}-aba-loops"
            print(label, len(script.splitlines()), len(script.encode()), "lines/bytes", flush=True)
            for tol in (0,int(hz*.000025)):
                cfg.write_text(base + explicit)
                rows,gold=run_stock(stock,dictionary,f,t,g,tol==0)
                _,repeat=run_stock(stock,dictionary,f,t,g,tol==0); assert gold==repeat
                meta,=rows_with_prefix(rows,'XYZ_BIND'); assert meta['max_error']==tol
                eid,={q['oid'] for q in rows_with_prefix(rows,'SHARED_OUTCOME')}
                lanes=[flatten_file(gold[f'oid={j}']) for j in [*meta['oids'],eid]]
                w=rows_with_prefix(rows,'WINDOW'); applied = [q for q in w if q['op']=='p']
                print("last generation",max(q["at"] for q in w if q["op"]=="f"),flush=True)
                assert tuple((q['gain'],q['smooth']) for q in applied) == expected, (label, applied)
                shapes=rows_with_prefix(rows,'SHAPE_STATE'); assert len(shapes)==1+rounds*2 and shapes[0]==shapes[2]!=shapes[1]
                i=next(i for i,q in enumerate(w) if q['op']=='p')
                x.write_text(shape_rows(rows));y.write_text(timeline(w[i:],dict(ATTRS,p='at gain smooth')))
                expect=''.join('\n'.join([*rows,'end','']) for rows,_ in lanes).encode()
                c_args=[x,y,str(hz),str(tol),*map(str,meta['d']),*(str(int(v)) for v in meta['inverted'])]
                for count in ('1','4'): assert subprocess.check_output([exe,count,*c_args])==expect
                md["initial_pressure_advance"] = initial
                md["max_error_ticks"] = tol
                m.write_text(json.dumps(md))
                old = [cli, "--dynamic-pressure", m, g, settings]
                rust_args = [cli, "--extended-pickup", *arc_flags, m, g, settings]
                parallel_args = [cli, "--extended-pickup-parallel", *arc_flags, m, g, settings]
                data = subprocess.check_output([*rust_args, meshfile, *names], timeout=25)
                assert subprocess.check_output([*parallel_args, meshfile, *names], timeout=25) == data
                fast_index = 2 + len(arc_flags)
                fast_args = [*rust_args[:fast_index], "--fast-json", *rust_args[fast_index:]]
                if args.fast_json:
                    assert subprocess.check_output([*fast_args, meshfile, *names], timeout=25) == data
                    fast_parallel = [*parallel_args[:fast_index], "--fast-json", *parallel_args[fast_index:]]
                    assert subprocess.check_output([*fast_parallel, meshfile, *names], timeout=25) == data
                    wrong = subprocess.run([cli, "--fast-json", m, g, settings, meshfile, *names],
                                           capture_output=True, timeout=25)
                    assert wrong.returncode == 2 and not wrong.stdout and b"requires" in wrong.stderr
                want = [packets for packets, _ in lanes]
                for lane, (grab, expected_lane) in enumerate(zip(wire_records(data, hz), want, strict=True)):
                    assert grab == expected_lane, (label, tol, lane, len(grab), len(expected_lane),
                        next(((i, a, b) for i, (a, b) in enumerate(zip(grab, expected_lane)) if a != b), None))
                narrow = subprocess.run([*old, meshfile, *names], capture_output=True, timeout=25)
                if rounds == 4 and not args.arcs:
                    assert narrow.returncode == 0 and narrow.stdout == data
                else:
                    assert narrow.returncode == 2 and not narrow.stdout
                for path in (savedfile, mixedfile):
                    cfg.write_text(base + path.read_text())
                    _, saved_gold = run_stock(stock,dictionary,f,t,g,tol==0)
                    assert saved_gold == gold
                    assert subprocess.check_output(
                        [*rust_args, path, *names], timeout=25) == data
                    assert subprocess.check_output(
                        [*parallel_args, path, *names], timeout=25) == data
                    if args.fast_json:
                        assert subprocess.check_output([*fast_args, path, *names], timeout=25) == data
                cfg.write_text(base + explicit)
                narrow_case = rounds == 4 and not args.arcs
                cap = 512 if narrow_case else 1024
                target = old if narrow_case else rust_args
                padded = script + ";\n" * (cap - len(script.splitlines()))
                g.write_text(padded)
                assert subprocess.check_output([*target, meshfile, *names], timeout=25) == data
                def reject(source, entry=rust_args):
                    g.write_text(source)
                    rejected = subprocess.run([*entry, meshfile, *names], capture_output=True, timeout=25)
                    assert rejected.returncode == 2 and not rejected.stdout
                if rounds == 16:
                    assert subprocess.check_output([*parallel_args, meshfile, *names], timeout=25) == data
                    reject(padded + ";\n", parallel_args)
                    reject(script.replace("M400\n", PROFILE0+"M400\n", 1), parallel_args)
                reject(padded + ";\n", target)
                reject(script + "; " + "x"*4096 + "\n")
                reject(script + "; unicode é\n")
                reject(script.rstrip("\n"))
                reject(script.replace("M400\n", "G4 P120000\nM400\n", 1))
                if rounds == 16:
                    reject(script.replace("M400\n", PROFILE0+"M400\n", 1))
                g.write_text(script)
                print(label, tol, sum(a for _,a in lanes),
                      "steps; pinned stock, native C 1/4, and Rust exact", flush=True)
                if args.parallel_rounds:
                    frozen = [args.baseline_cli or cli, *rust_args[1:], meshfile, *names]
                    candidate = [*(fast_args if args.fast_json else parallel_args), meshfile, *names]
                    assert subprocess.check_output(frozen) == data
                    assert subprocess.check_output(candidate) == data
                    for command in (frozen, candidate):
                        for path in (savedfile, mixedfile):
                            alt = command.copy()
                            alt[-3] = path
                            assert subprocess.check_output(alt) == data
                    output = (t/"whole.json")
                    def elapsed(command, expected):
                        with output.open("wb") as dest:
                            started = time.perf_counter_ns()
                            # A Python subprocess timeout polls with exponential
                            # sleeps and quantizes sub-100 ms measurements.
                            subprocess.run(command, stdout=dest, check=True)
                            elapsed_ns = time.perf_counter_ns() - started
                        assert output.read_bytes() == expected
                        return elapsed_ns / 1e6
                    cases = [("rust_json" if args.fast_json else "rust", [frozen, candidate], data)]
                    if not args.fast_json:
                        cases.append(("native_c", [[exe,"1",*c_args], [exe,"4",*c_args]], expect))
                    if args.c_json_shadow:
                        source = t / "validated_records.txt"
                        source.write_bytes(expect)
                        generic, compact = ([shadow, arm, source, json.dumps(hz)] for arm in ("generic", "compact"))
                        cases.append(("native_c_json", [generic, compact], data))
                    def measurement(name, command, expected):
                        if name != "native_c_json":
                            return elapsed(command, expected)
                        with output.open("wb") as dest:
                            proc = subprocess.run(command, stdout=dest, stderr=subprocess.PIPE, check=True)
                        assert output.read_bytes() == expected
                        return int(proc.stderr) / 1e6
                    for name, commands, expected_data in cases:
                        for _ in range(3):
                            for cmd in commands:
                                measurement(name, cmd, expected_data)
                        times = [[], []]
                        win = 0
                        for index in range(args.parallel_rounds):
                            order = (0,1) if index % 2 == 0 else (1,0)
                            pair = [0,0]
                            for arm in order:
                                pair[arm] = measurement(name, commands[arm], expected_data)
                                times[arm].append(pair[arm])
                            win += pair[1] < pair[0]
                            with args.parallel_jsonl.open("a") as out:
                                print(json.dumps({"engine":name, "case":label, "tolerance":tol,
                                      "cores":sorted(os.sched_getaffinity(0)), "round":index,
                                      "order":order, "ms":pair,
                                      "sha256":hashlib.sha256(expected_data).hexdigest()}),file=out)
                        print(name, label, tol, [round(statistics.median(v),3) for v in times],
                              "candidate wins",win,"of",args.parallel_rounds,flush=True)
                if args.rounds:
                    stock_run = [sys.executable, Path(__file__).resolve(), "--stock-arm",
                                 stock, dictionary, f, m, g]
                    paired([stock_run,[*rust_args,meshfile,*names]],t/"whole.json",data,args.rounds)
        if args.arcs:
            circles = START + "G90\nG1 X20 Y20 Z1.5 F1500\n"
            raw_arc = "G3 X20 Y20 I1 J0 F3600\n"
            def grouped_arcs(count):
                body = "".join(raw_arc * min(8, count-i) + PROFILE0 for i in range(0, count, 8))
                return circles + body + "M400\n"
            base = [cli, "--extended-pickup", "--arc-resolution=0.25", m, g, settings, meshfile, *names]
            g.write_text(grouped_arcs(163))
            assert subprocess.check_output(base, timeout=25)
            g.write_text(grouped_arcs(164))
            too_many = subprocess.run(base, capture_output=True, timeout=25)
            assert too_many.returncode == 2 and not too_many.stdout and b"ArcLimit" in too_many.stderr
            g.write_text(circles + raw_arc + "M400\n")
            for command in (
                [cli, "--extended-pickup", m, g, settings, meshfile, *names],
                [cli, "--arc-resolution=1", m, g, settings, meshfile, *names],
                *([cli, "--extended-pickup", "--arc-resolution=" + invalid, m, g, settings, meshfile, *names]
                  for invalid in ("NaN", ".249", "5.001", "inf")),
            ):
                rejected = subprocess.run(command, capture_output=True, timeout=25)
                assert rejected.returncode == 2 and not rejected.stdout
            for unsupported in ("G91\nG3 X1 Y1 I1 J0\n", "G2 X24 Y20 R2\n",
                                "G3 X310 Y20 I145 J0\n", "G18 F70\n"):
                g.write_text(circles + unsupported + "M400\n")
                rejected = subprocess.run(base, capture_output=True, timeout=25)
                assert rejected.returncode == 2 and not rejected.stdout
            print("Arc distance, resolution, motion mode and cumulative bounds passed", flush=True)
        print("Both pickup bounds and four longer-job stock oracles passed", flush=True)


if __name__ == "__main__":
    main(sys.argv[1:])
