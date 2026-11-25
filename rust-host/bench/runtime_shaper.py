#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Synthetic runtime shaper updates; no device, stock changes or print control."""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

from file_debug_lane_timing import rows_with_prefix, timeline
from file_debug_process_timing import stock_arm
from shaped_file_debug import build_c, paired, paired_workers
from shared_window_capture import ATTRS, XYZ_ATTRS, PIN, ROOT, flatten_file, run_stock

SCHEMA = "klipper.shared-corexy.runtime-shaper.v1"

HOOK = """
from extras.input_shaper import InputShaper
old_shape_update = InputShaper._update_input_shaping
def observe_shape_update(self, *args, **kwargs):
    result = old_shape_update(self, *args, **kwargs)
    emit("SHAPE_STATE", lanes=[list(zip(axis.A, axis.T)) for axis in self.get_shapers()])
    return result
InputShaper._update_input_shaping = observe_shape_update
"""
START = "SET_KINEMATIC_POSITION X=10 Y=10 Z=1\n"
PROFILE0 = "SET_INPUT_SHAPER SHAPER_TYPE_X=mzv SHAPER_FREQ_X=67.6 SHAPER_TYPE_Y=mzv SHAPER_FREQ_Y=44.8\n"
PROFILE1 = "SET_INPUT_SHAPER SHAPER_TYPE_X=mzv SHAPER_FREQ_X=69.4 SHAPER_TYPE_Y=ei SHAPER_FREQ_Y=57\n"
CONFIG0 = """
[input_shaper]
shaper_type_x: mzv
shaper_freq_x: 67.6
shaper_type_y: mzv
shaper_freq_y: 44.8
"""


def scripts(fixture):
    one = (START+"G90\nM83\n"
           +"G1 X25 Y20 E.6 F750\nG1 X42 Y30 E.7 F1000\n"
           +PROFILE1
           +"G1 X28 Y45 E.7 F900\nG1 Z1.2 F600\n"
           +PROFILE0
           +"G1 X40 Y35 E.6 F1000\nG1 X26 Y29 E.5 F900\nM400\n")
    base = (fixture/"path.gcode").read_text()
    both = base.replace("ACTIVATE_EXTRUDER EXTRUDER=extruder1\n",
                        "ACTIVATE_EXTRUDER EXTRUDER=extruder1\n"
                        "SET_GCODE_OFFSET X=0.0625 Y=-0.035 Z=0.015\n"+PROFILE1, 1)
    both = both.replace("ACTIVATE_EXTRUDER EXTRUDER=extruder\n",
                        "ACTIVATE_EXTRUDER EXTRUDER=extruder\n"
                        "SET_GCODE_OFFSET X=0 Y=0 Z=0\n"+PROFILE0, 1)
    slow = one.replace("SHAPER_FREQ_X=69.4 SHAPER_TYPE_Y=ei SHAPER_FREQ_Y=57",
                       "SHAPER_FREQ_X=35 SHAPER_TYPE_Y=ei SHAPER_FREQ_Y=20")
    prefix = both.index(PROFILE1) + len(PROFILE1)
    boundaries = []
    for end in (4095,4096,4097):
        padded = both[:prefix-1] + " ;" + "b"*(end-prefix-2) + both[prefix-1:]
        assert padded.index("\n", prefix-len(PROFILE1)) + 1 == end
        boundaries.append((f"shared-boundary-{end}",padded,3))
    disabled = one.replace(PROFILE1, "SET_INPUT_SHAPER SHAPER_FREQ_X=0 SHAPER_FREQ_Y=0\n")
    partial = one.replace(PROFILE1, "SET_INPUT_SHAPER SHAPER_FREQ_Y=0\n")
    priority = one.replace(PROFILE1,
        "set_input_shaper shaper_type=zv shaper_type_x=ei damping_ratio_x=0.14\n").replace(
        PROFILE0, PROFILE0.rstrip()+" DAMPING_RATIO_X=0.1\n")
    return (("one-queue",one,3), ("shared-a-b-a",both,3), ("wider-window",slow,3),
            ("disabled",disabled,3), ("partial",partial,3), ("global-priority",priority,3),
            ("report-only",base.replace("G1 X46 Y21", "SET_INPUT_SHAPER\nG1 X46 Y21",1),1),
            *boundaries)



def scaled(fixture):
    _, shared, _ = next(s for s in scripts(fixture) if s[0] == "shared-a-b-a")
    for label, lines in (("alternating", 216), ("wide", 240), ("mixed-waits", 200)):
        normalized = []
        for i in range(lines):
            if i == lines//3:
                normalized.append(PROFILE1.rstrip())
            if i == 2*lines//3:
                normalized.append(PROFILE0.rstrip())
            if label == "mixed-waits" and i in (30,73,181):
                normalized.extend(("M400","G4 P20"))
            if label == "wide":
                normalized.append(f"G1 X{18+(i%2)*.61:.3f} Y{39+(1-i%2)*.29:.3f} E.006 F1020")
            elif label == "mixed-waits":
                normalized.append(f"G1 X{18+(i%2)*.28:.3f} Y{39+(1-i%2)*.67:.3f} "
                                  f"Z{1.42+(i%3)*.017:.3f} E{.008 if i%11 else -.006} F480")
            else:
                normalized.append(f"G1 X{18+(i%2)*.24:.3f} Y{39+(1-i%2)*.14:.3f} E.002 F600")
        yield label, shared.replace("M400\n","\n".join(normalized)+"\nM400\n",1)



def shape_rows(rows):
    parts = []
    for row in rows:
        if row.startswith("XYZ "):
            parts.append(timeline([json.loads(row.split(" ",1)[1])], XYZ_ATTRS))
        if row.startswith("SHAPE_STATE "):
            shape=json.loads(row.split(" ",1)[1])
            for axis, pairs in zip("xy",shape["lanes"][:2],strict=True):
                parts.append(" ".join(map(str,("h",axis,len(pairs),*(v for p in pairs for v in p))))+"\n")
    return "".join(parts)


def wire_records(raw, hz):
    trace = json.loads(raw)
    assert trace["schema"] == SCHEMA and trace["clock_hz"] == hz
    assert trace["lane_order"] == ["corexy_a", "corexy_b", "z", "shared_e"]
    return [
        [f"d {int(item['positive'])}" if item["kind"] == "direction" else
         "s {interval} {count} {add} {last_clock}".format(**item)
         for item in lane] for lane in trace["lanes"]
    ]


def check_rejected(cli, model, script, settings):
    for invalid in [
        "SHAPER_FREQ_Z=44",
        "SHAPER_FREQ_X=NaN",
        "SHAPER_FREQ_Y=-2",
        "SHAPER_FREQ_Y=1e15",
        "SHAPER_TYPE_Y=ei DAMPING_RATIO_Y=0.50",
        "SHAPER_TYPE_X=unknown",
        "SHAPER_TYPE_Y=mzv#trailing",
        "SHAPER_FREQ_X=50 SHAPER_FREQ_X=60",
        "SHAPER_FREQ_X",
        "UNRECOGNIZED=1",
    ]:
        script.write_text(START+f"SET_INPUT_SHAPER {invalid}\nG1 X20 F500\nM400\n")
        result = subprocess.run([cli, model, script, settings], capture_output=True, timeout=5)
        assert result.returncode == 2 and not result.stdout, (invalid, result.stderr)
    script.write_text(START + PROFILE0 * 33 + "G1 X20 F500\nM400\n")
    result = subprocess.run([cli, model, script, settings], capture_output=True, timeout=5)
    assert result.returncode == 2 and not result.stdout, "too many updates"
    script.write_text(START+"G1 X20 F500\nM400\n")
    settings.write_text(CONFIG0+"shaper_freq_z: 45\n")
    result = subprocess.run([cli, model, script, settings], capture_output=True, timeout=5)
    assert result.returncode == 2 and not result.stdout, "unsupported start configuration"


def main(argv):
    if len(argv) == 6 and argv[0] == "--stock-arm":
        stock_arm(*(Path(w).resolve() for w in argv[1:]), schema=SCHEMA)
        return
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("dictionary", type=Path)
    parser.add_argument("cli", type=Path)
    parser.add_argument("--rounds", type=int, default=0,
                        help="time stock vs Rust with identical JSON on three profiles")
    parser.add_argument("--parallel-cli", type=Path, help="optional four-worker runtime preview")
    parser.add_argument("--worker-rounds", type=int, default=0)
    parser.add_argument("--baseline-cli", type=Path, help="prior runtime preview to time as the first Rust arm")
    parser.add_argument("--cores", help="comma-separated Linux CPU affinity")
    a = parser.parse_args(argv)
    if not 0 <= a.rounds <= 80:
        parser.error("--rounds must be 0–80")
    if not 0 <= a.worker_rounds <= 200:
        parser.error("--worker-rounds must be 0–200")
    if a.worker_rounds and not a.parallel_cli:
        parser.error("--worker-rounds requires --parallel-cli")
    parallel = a.parallel_cli.resolve() if a.parallel_cli else None
    if a.baseline_cli and not a.worker_rounds:
        parser.error("--baseline-cli requires --worker-rounds")
    baseline = a.baseline_cli.resolve() if a.baseline_cli else None
    if a.cores:
        os.sched_setaffinity(0, {int(c) for c in a.cores.split(",")})
    upstream, dictionary, cli = (p.resolve() for p in (a.upstream, a.dictionary, a.cli))
    assert subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    hz = int(json.loads(dictionary.read_text())["config"]["CLOCK_FREQ"])
    fixture0 = ROOT / "bench/shared_extruder_fixture"
    model_cfg = json.loads((fixture0 / "offline_model.json").read_text())
    assert model_cfg["clock_hz"] == hz
    checked = 0
    with tempfile.TemporaryDirectory(prefix="dynamic-shape-stock-") as d:
        t = Path(d)
        fixture = t / "fixture"
        shutil.copytree(fixture0, fixture)
        entry = fixture / "stock_entry.py"
        source = entry.read_text()
        marker = 'sys.argv = [entry, "--drain-debug-output", stock, "--stock-only", *args]'
        assert source.count(marker) == 1
        entry.write_text(source.replace(marker, HOOK + "\n" + marker))
        cfg = fixture / "printer.cfg"
        cfg.write_text(cfg.read_text() + "\n" + CONFIG0)
        c = t / "worker-c"
        build_c(upstream, c)
        script, xyz, e, model, settings, out = (t/f for f in
            ("input.gcode", "xyz.txt", "e.txt", "model.json", "shape.cfg", "out.json"))
        settings.write_text(CONFIG0)
        short = list(scripts(fixture))
        long = [(label, program, 5) for label, program in scaled(fixture)]
        for label, program, n in short + long:
            script.write_text(program)
            for tolerance in (0, int(hz*.000025)):
                rows, gold = run_stock(upstream, dictionary, fixture, t, script, tolerance == 0)
                r2, g2 = run_stock(upstream, dictionary, fixture, t, script, tolerance == 0)
                assert g2 == gold
                states = rows_with_prefix(rows, "SHAPE_STATE")
                assert len(states) == n and states == rows_with_prefix(r2, "SHAPE_STATE"), (label, len(states))
                if n >= 3:
                    assert all(state == states[index % 2] for index, state in enumerate(states))
                    assert states[0] != states[1]
                e_rows = rows_with_prefix(rows, "WINDOW")
                i = next(i for i, r in enumerate(e_rows) if r["op"] == "p")
                e.write_text(timeline(e_rows[i:], ATTRS))
                xyz.write_text(shape_rows(rows))
                meta, = rows_with_prefix(rows, "XYZ_BIND")
                assert meta["max_error"] == tolerance
                eid, = {r["oid"] for r in rows_with_prefix(rows, "SHARED_OUTCOME")}
                expected = [flatten_file(gold[f"oid={i}"]) for i in [*meta["oids"], eid]]
                payload = "".join("\n".join([*lane, "end", ""]) for lane, _ in expected).encode()
                c_args = [xyz, e, str(hz), str(tolerance),
                          *map(str, meta["d"]), *(str(int(v)) for v in meta["inverted"])]
                for workers in ("1", "4"):
                    ran = subprocess.check_output([c, workers, *c_args], timeout=10)
                    assert ran == payload, (label, tolerance, workers)
                model_cfg["max_error_ticks"] = tolerance
                model.write_text(json.dumps(model_cfg))
                rust = [cli, model, script, settings]
                output = subprocess.check_output(rust, timeout=10)
                if parallel:
                    workers = [parallel, model, script, settings]
                    assert subprocess.check_output(workers, timeout=10) == output, (label, tolerance)
                assert wire_records(output, hz) == [lane for lane, _ in expected], (label, tolerance)
                print(label, tolerance, "ticks:", sum(n for _, n in expected),
                      "exact steps, shape states", n, flush=True)
                checked += 1
                if a.rounds and label in {"shared-a-b-a", "disabled", "shared-boundary-4096"}:
                    stock = [sys.executable, Path(__file__).resolve(), "--stock-arm",
                             upstream, dictionary, fixture, model, script]
                    paired([stock, rust], out, output, a.rounds)
                if a.worker_rounds and n == 5:
                    arms = [rust, workers]
                    if baseline:
                        arms.insert(0, [baseline, model, script, settings])
                    paired_workers("Rust whole CLI", arms, out, output, a.worker_rounds)
                    paired_workers("C captured replay", [[c, m, *c_args] for m in ("1", "4")],
                                   out, payload, a.worker_rounds)
        for tool in (cli, parallel):
            if tool:
                check_rejected(tool, model, script, settings)
    print(f"{checked} runtime shaper cases, exact stock/C/Rust; invalid inputs rejected")


if __name__ == "__main__":
    main(sys.argv[1:])
