#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare selected saved mesh motion with actual Klippy+C in memory."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace

from gcode_preview import stock
from planner import PIN, ROOT

PROFILE = "test_surface"


def reference(upstream, executable, cfg, script, runtime=False):
    sys.path.insert(0, str(upstream/"klippy"))
    from configfile import ConfigAutoSave, ConfigFileReader, ConfigWrapper
    from extras.bed_mesh import BedMesh, MoveSplitter
    from extras.bed_mesh import ProfileManager
    from gcode import GCodeCommand, GCodeDispatch
    from kinematics.corexy import CoreXYKinematics
    from stepper import parse_step_distance

    raw, saved = ConfigAutoSave._find_autosave_data(None, cfg.read_text())
    cfg_data = ConfigFileReader().build_fileconfig_with_includes(raw, cfg)
    if saved.strip():
        cfg_data.read_string(saved)
    wrap = lambda name, printer=None: ConfigWrapper(printer, cfg_data, {}, name)
    rails = [wrap("stepper_"+axis) for axis in "xyz"]
    distances = [r/s for r,s in (parse_step_distance(config, False) for config in rails)]
    origin = (1.,1.,0.2)
    settings = (CoreXYKinematics, [(0.,200.),(0.,180.),(0.,150.)],
                180.,3100.,5.,.5,9.,160.,origin)
    rows, clock = [], 2.

    bed = None

    def activated(line, coords, head):
        nonlocal bed
        if line.startswith("!mesh_cmd "):
            assert runtime and bed is not None
            cmd = line.removeprefix("!mesh_cmd ")
            name = cmd.split()[0]
            dispatcher = object.__new__(GCodeDispatch)
            handlers = {"BED_MESH_PROFILE": bed.pmgr.cmd_BED_MESH_PROFILE,
                        "BED_MESH_CLEAR": bed.cmd_BED_MESH_CLEAR}
            command = GCodeCommand(bed.gcode, name, cmd, {}, False)
            saved_pos, saved_queue = list(head.commanded_pos), tuple(head.lookahead.queue)
            handlers[name](dispatcher._get_extended_params(command))
            assert head.commanded_pos == saved_pos and tuple(head.lookahead.queue) == saved_queue
            return
        assert line == "!use_profile"
        real_gcode = SimpleNamespace(register_command=lambda *a, **kw: None, error=ValueError,
                                     respond_info=lambda *_a, **_kw: None, respond_raw=lambda _: None)
        objects = {"gcode_move": coords, "gcode": real_gcode}
        printer = SimpleNamespace(lookup_object=objects.__getitem__)
        mc = wrap("bed_mesh",printer)
        bed = BedMesh.__new__(BedMesh)
        bed.toolhead = SimpleNamespace(move=coords.move_with_transform, get_position=coords.position_with_transform)
        bed.printer, bed.gcode = printer, real_gcode
        bed.fade_start = mc.getfloat("fade_start",1.)
        bed.fade_end = mc.getfloat("fade_end",0.)
        bed.fade_dist = bed.fade_end - bed.fade_start
        if bed.fade_dist <= 0.:
            bed.fade_start = bed.fade_end = bed.FADE_DISABLE
        bed.base_fade_target = mc.getfloat("fade_target",None)
        bed.last_position = [0.]*4
        bed.z_mesh = None
        bed.log_fade_complete = False
        bed.splitter = MoveSplitter(mc, real_gcode)
        coords.set_move_transform(bed, force=True)
        bed.pmgr = ProfileManager(mc, bed)
        if runtime:
            bed.set_mesh(None)
        else:
            bed.pmgr.load_profile(PROFILE)
        assert head.commanded_pos[:3] == list(origin)

    def moves(items, dwell):
        nonlocal clock
        for m in items:
            cruise = 0. if -1e-12 <= m.cruise_t < 0. else m.cruise_t
            rows.append(("m", clock, m.accel_t, cruise, m.decel_t,
                         *m.start_pos[:3], *m.axes_r[:3], m.start_v, m.cruise_v, m.accel))
            clock += m.accel_t
            clock += cruise
            clock += m.decel_t
        clock += dwell

    commands = script.read_text().splitlines()
    if runtime:
        commands = [f"!mesh_cmd {line}" if line.startswith("BED_MESH_") else line for line in commands]
    stock(["!use_profile", *commands], False,
          settings=settings, emit_profiles=moves, extend_directive=activated)
    for (axis,distance) in zip(("+","-","z"),distances,strict=True):
        rows.append(("s", axis, *origin, distance, clock+.01,20e6,0,0))
    data = "\n".join(" ".join(map(str,row)) for row in rows)+"\n"
    result = subprocess.run([executable], input=data, text=True, capture_output=True, check=True)
    return result.stdout.splitlines(), clock-2.


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, required=True)
    a = p.parse_args()
    upstream = a.upstream.resolve()
    fixture = ROOT/"examples/preview"
    script = fixture/"path_mesh.gcode"
    h = upstream/"klippy/chelper"
    with tempfile.TemporaryDirectory() as d:
        c = Path(d)/"reference"
        subprocess.run([os.environ.get("CC","cc"),"-O3","-I",h,ROOT/"bench/machine_trace.c",
                        *(h/(n+".c") for n in ("itersolve","stepcompress","trapq","kin_cartesian",
                            "kin_corexy","kin_extruder","kin_shaper","pyhelper","msgblock")),
                        "-lm","-o",c], check=True)
        seen = []
        for cfg in ("mesh.cfg","mesh-saved.cfg"):
            want, duration = reference(upstream,c,fixture/cfg,script)
            wire = subprocess.check_output([a.binary,"--mesh-profile",PROFILE,fixture,cfg,script,
                                           "20000000","1","1",".2","printer.cfg","rails.cfg"])
            parsed = json.loads(wire)
            assert parsed["mesh_profile"] == PROFILE
            assert math.isclose(parsed["planned_duration_seconds"],duration,rel_tol=1e-12,abs_tol=1e-12)
            actual = []
            for lane in parsed["motors"]:
                for row in lane:
                    if row["kind"] == "direction":
                        actual.append(f'd {int(row["positive"])}')
                    else:
                        actual.append(f's {row["interval"]} {row["count"]} {row["add"]} {row["last_clock"]}')
                actual.append("end")
            for i,(x,y) in enumerate(zip(want,actual,strict=True)):
                assert x == y,(cfg,i,x,y)
            seen.append(wire)
            print(f"{cfg}: {len(actual)-3} exact motor records; {duration:.9f} simulated seconds")
        assert seen[0] == seen[1], "explicit and autosaved mesh differ"

        path = Path(d)/"runtime.gcode"
        commands = ["G90", "G1 X15 Y12 Z0.32 F1800", f"BED_MESH_PROFILE LOAD={PROFILE}",
                    "G1 X30 Y24 Z0.5", "G1 X50 Y32", "BED_MESH_CLEAR", "G1 X72 Y45",
                    "G91", "G1 X12 Y-3 Z.1", f"BED_MESH_PROFILE LOAD={PROFILE}",
                    "G1 X23 Y10 Z-.2", "G90", "G1 X130 Y60 Z1.3", "BED_MESH_CLEAR",
                    "G1 X125 Y78 Z2.3", "M400", "BED_MESH_CLEAR"]
        path.write_text("\n".join(commands) + "\n")
        cfg = fixture/"mesh-saved.cfg"
        cmd = [a.binary, "--mesh-script", "--mesh-profile", PROFILE, fixture,
               cfg.name, path, "20000000", "1", "1", ".2", "printer.cfg", "rails.cfg"]
        expected, duration = reference(upstream, c, cfg, path, runtime=True)
        got = json.loads(subprocess.check_output(cmd))
        assert got["mesh_script_profile"] == PROFILE and "mesh_profile" not in got
        assert math.isclose(got["planned_duration_seconds"], duration, rel_tol=1e-12, abs_tol=1e-12)
        actual = []
        for lane in got["motors"]:
            for row in lane:
                if row["kind"] == "direction":
                    actual.append(f'd {int(row["positive"])}')
                else:
                    actual.append(f's {row["interval"]} {row["count"]} {row["add"]} {row["last_clock"]}')
            actual.append("end")
        for i, (want, actual_row) in enumerate(zip(expected, actual, strict=True)):
            assert want == actual_row, (i, want, actual_row)
        print(f"runtime saved mesh load/clear: {len(actual)-3} exact timed stock motor records")
        for invalid in ("BED_MESH_PROFILE LOAD=missing", f"BED_MESH_PROFILE SAVE={PROFILE}",
                        f"BED_MESH_PROFILE REMOVE={PROFILE}", "BED_MESH_CALIBRATE",
                        "BED_MESH_CLEAR X=1"):
            path.write_text(f"G90\n{invalid}\n")
            denied = subprocess.run(cmd, capture_output=True)
            assert denied.returncode and not denied.stdout, (invalid, denied.stdout)


if __name__ == "__main__":
    main()
