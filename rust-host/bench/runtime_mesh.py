#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline runtime shaper and saved-mesh packets with pinned stock."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from file_debug_lane_timing import rows_with_prefix, timeline
from file_debug_process_timing import stock_arm
from runtime_shaper import CONFIG0, HOOK, PROFILE0, PROFILE1, SCHEMA, scripts, shape_rows, wire_records
from shaped_file_debug import build_c, paired
from shared_window_capture import ATTRS, PIN, ROOT, flatten_file, run_stock

NAME = "test_surface"
LOAD = f"BED_MESH_PROFILE LOAD={NAME}"
CLEAR = "BED_MESH_CLEAR"
SAVED = ("#*# <---------------------- SAVE_CONFIG ---------------------->\n"
        "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n"
        "#*#\n")


def profiles():
    fixture = ROOT / "examples/preview/mesh.cfg"
    mesh = fixture.read_text().split("\n", 1)[1].replace(
        "[bed_mesh]", "[bed_mesh]\nmesh_min: 0,0\nmesh_max: 35,35\nprobe_count: 3,3")
    name = f"[bed_mesh {NAME}]"
    header, details = mesh.split(name, 1)
    return mesh, header + SAVED + "".join(f"#*# {line}\n" for line in (name + details).splitlines())


def jobs(fixture):
    shared = next(source for label, source, _ in scripts(fixture) if label == "shared-a-b-a")
    first, body = shared.split("\n", 1)
    fixed = f"{first}\n{LOAD}\n{body}"
    offset = fixed.replace(PROFILE1, PROFILE1 + "BED_MESH_OFFSET X=0.5 Y=-0.3 ZFADE=0.22\n", 1)
    reset = offset.replace(PROFILE0, f"{CLEAR}\n{PROFILE0}", 1).replace(
        "M400\n", f"G1 Z3.7 F600\n{LOAD}\nG1 X15 Y14 Z1.7 E0.045 F900\nM400\n", 1)
    return (("fixed-load", fixed), ("mesh-offset", offset), ("clear-reload-fade", reset))


def rejected(cli, model, script, settings, raw, scratch, program):
    def run(mesh, name):
        args = [cli, model, script, settings, mesh, name]
        subprocess_result = subprocess.run(args, timeout=10, capture_output=True)
        assert subprocess_result.returncode == 2 and not subprocess_result.stdout, args

    script.write_text(program)
    for name in ("test_other", "../test_surface", "", "z" * 65):
        run(raw, name)
    source = raw.read_text()
    header, _ = source.split(f"[bed_mesh {NAME}]", 1)
    for text in (header, header + "[include foreign.cfg]\n",
                 source + "\n[DEFAULT]\nfade_start:0.5\n", source + "\n[extruder]\n",
                 source + "\n" + profiles()[1].removeprefix(header),
                 source.replace("version: 1", "version: 0", 1)):
        scratch.write_text(text)
        run(scratch, NAME)
    for wrong in ("BED_MESH_PROFILE SAVE=test_surface", "BED_MESH_PROFILE LOAD=other",
                  LOAD + " Z=1", CLEAR + " Z=1", "BED_MESH_OFFSET X=NaN",
                  "BED_MESH_OFFSET ZFADE=1001", "BED_MESH_CALIBRATE", "BED_MESH_OUTPUT"):
        script.write_text(program.replace(LOAD, wrong, 1))
        run(raw, NAME)


def main(argv):
    if len(argv) == 6 and argv[0] == "--stock-arm":
        stock_arm(*(Path(v).resolve() for v in argv[1:]), schema=SCHEMA)
        return
    parser = argparse.ArgumentParser(description=__doc__)
    for label in ("upstream", "dictionary", "mesh_cli"):
        parser.add_argument(label, type=lambda v: Path(v).resolve())
    parser.add_argument("--rounds", type=int, default=0)
    parser.add_argument("--core", type=int)
    args = parser.parse_args(argv)
    if not 0 <= args.rounds <= 80:
        parser.error("--rounds must be 0–80")
    if args.core is not None:
        os.sched_setaffinity(0, {args.core})
    upstream, dictionary, mesh_cli = args.upstream, args.dictionary, args.mesh_cli
    assert subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    hz = int(json.loads(dictionary.read_text())["config"]["CLOCK_FREQ"])
    with tempfile.TemporaryDirectory(prefix="runtime-saved-mesh-stock-") as scratch:
        tmp = Path(scratch)
        fixture = tmp / "fixture"
        shutil.copytree(ROOT / "bench/shared_extruder_fixture", fixture)
        entry = fixture / "stock_entry.py"
        source = entry.read_text()
        marker = 'sys.argv = [entry, "--drain-debug-output", stock, "--stock-only", *args]'
        assert source.count(marker) == 1
        entry.write_text(source.replace(marker, HOOK + "\n" + marker))
        mesh, saved = profiles()
        cfg = fixture / "printer.cfg"
        cfg.write_text(cfg.read_text() + "\n" + CONFIG0 + "\n" + mesh)
        runner, script, xyz, e, model, settings, meshfile, autosave, out, bad = (
            tmp / label for label in ("worker-c", "input.gcode", "xyz.txt", "e.txt", "model.json",
                                      "shape.cfg", "mesh.cfg", "save.cfg", "out.json", "bad.cfg"))
        meshfile.write_text(mesh)
        autosave.write_text(saved)
        settings.write_text(CONFIG0)
        build_c(upstream, runner)
        model_cfg = json.loads((fixture / "offline_model.json").read_text())
        assert model_cfg["clock_hz"] == hz
        for label, program in jobs(fixture):
            script.write_text(program)
            for tolerance in (0, int(hz * .000025)):
                rows, stock = run_stock(upstream, dictionary, fixture, tmp, script, tolerance == 0)
                _, repeated = run_stock(upstream, dictionary, fixture, tmp, script, tolerance == 0)
                assert stock == repeated
                shapes = rows_with_prefix(rows, "SHAPE_STATE")
                assert len(shapes) == 3 and shapes[0] == shapes[2] != shapes[1]
                windows = rows_with_prefix(rows, "WINDOW")
                start = next(i for i, row in enumerate(windows) if row["op"] == "p")
                e.write_text(timeline(windows[start:], ATTRS))
                xyz.write_text(shape_rows(rows))
                meta, = rows_with_prefix(rows, "XYZ_BIND")
                assert meta["max_error"] == tolerance
                eid, = {r["oid"] for r in rows_with_prefix(rows, "SHARED_OUTCOME")}
                gold = [flatten_file(stock[f"oid={i}"]) for i in [*meta["oids"], eid]]
                serialized = "".join("\n".join([*lane, "end", ""]) for lane, _ in gold).encode()
                c_args = [xyz, e, str(hz), str(tolerance),
                          *map(str, meta["d"]), *(str(int(i)) for i in meta["inverted"])]
                for workers in ("1", "4"):
                    got = subprocess.check_output([runner, workers, *c_args], timeout=10)
                    assert got == serialized, (label, tolerance, workers)
                model_cfg["max_error_ticks"] = tolerance
                model.write_text(json.dumps(model_cfg))
                rust = [mesh_cli, model, script, settings, meshfile, NAME]
                normal = subprocess.check_output(rust, timeout=10)
                assert subprocess.check_output([*rust[:-2], autosave, NAME], timeout=10) == normal
                assert wire_records(normal, hz) == [packets for packets, _ in gold], (label, tolerance)
                print(label, tolerance, sum(count for _, count in gold), "exact stock/Rust/C steps", flush=True)
                if args.rounds:
                    arm = [sys.executable, Path(__file__).resolve(), "--stock-arm",
                           upstream, dictionary, fixture, model, script]
                    paired([arm, rust], out, normal, args.rounds)
        rejected(mesh_cli, model, script, settings, meshfile, bad, jobs(fixture)[0][1])
    print("6 stock/Rust/C cases through both profile forms; 18 invalid inputs rejected")


if __name__ == "__main__":
    main(sys.argv[1:])
