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



def stock_mesh_commands(cfg_path, coords):
    """Install the pinned stock transform and return its real command seam."""
    from configfile import ConfigAutoSave, ConfigFileReader, ConfigWrapper
    from extras.bed_mesh import BedMesh, MoveSplitter, ProfileManager
    from gcode import GCodeCommand, GCodeDispatch

    raw, saved = ConfigAutoSave._find_autosave_data(None, cfg_path.read_text())
    config = ConfigFileReader().build_fileconfig_with_includes(raw, cfg_path)
    if saved.strip():
        config.read_string(saved)
    gcode = SimpleNamespace(register_command=lambda *_a, **_kw: None, error=ValueError,
                            respond_info=lambda *_a, **_kw: None, respond_raw=lambda _: None)
    objects = {"gcode_move": coords, "gcode": gcode}
    printer = SimpleNamespace(lookup_object=objects.__getitem__)
    wrap = ConfigWrapper(printer, config, {}, "bed_mesh")
    bed = BedMesh.__new__(BedMesh)
    bed.toolhead = SimpleNamespace(move=coords.move_with_transform, get_position=coords.position_with_transform)
    bed.printer, bed.gcode = printer, gcode
    bed.fade_start, bed.fade_end = wrap.getfloat("fade_start", 1.), wrap.getfloat("fade_end", 0.)
    bed.fade_dist = bed.fade_end - bed.fade_start
    if bed.fade_dist <= 0.:
        bed.fade_start = bed.fade_end = bed.FADE_DISABLE
    bed.base_fade_target = wrap.getfloat("fade_target", None)
    bed.last_position = [0.] * 4
    bed.z_mesh, bed.log_fade_complete = None, False
    bed.splitter = MoveSplitter(wrap, gcode)
    coords.set_move_transform(bed, force=True)
    bed.pmgr = ProfileManager(wrap, bed)
    bed.set_mesh(None)
    dispatcher = object.__new__(GCodeDispatch)
    handlers = {"BED_MESH_PROFILE": bed.pmgr.cmd_BED_MESH_PROFILE,
                "BED_MESH_CLEAR": bed.cmd_BED_MESH_CLEAR,
                "BED_MESH_OFFSET": bed.cmd_BED_MESH_OFFSET}

    def apply(line, head):
        name = line.split()[0]
        cmd = GCodeCommand(gcode, name, line, {}, False)
        pos, queued, clock = list(head.commanded_pos), tuple(head.lookahead.queue), head.print_time
        handlers[name](dispatcher._get_extended_params(cmd))
        assert head.commanded_pos == pos and tuple(head.lookahead.queue) == queued
        assert head.print_time == clock

    return apply

def reference(upstream, executable, cfg, script, runtime=False, runtime_e=False):
    sys.path.insert(0, str(upstream/"klippy"))
    from configfile import ConfigAutoSave, ConfigFileReader, ConfigWrapper
    from kinematics.corexy import CoreXYKinematics
    from kinematics.extruder import ExtruderStepper, PrinterExtruder
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

    gains = []
    extrusion = None
    if runtime_e:
        ext_cfg = wrap("extruder")
        extrusion = object.__new__(PrinterExtruder)
        extrusion.heater = SimpleNamespace(can_extrude=True)
        extrusion.instant_corner_v = 1.
        extrusion.nozzle_diameter = .4
        extrusion.filament_area = math.pi * (1.75 * .5)**2
        extrusion.max_extrude_ratio = .64 / extrusion.filament_area
        extrusion.max_e_dist, extrusion.max_e_velocity, extrusion.max_e_accel = 15., 20., 300.
        extrusion.trapq = None
        extrusion.trapq_append = lambda _, *fields: rows.append(
            ("e", *(int(x) if isinstance(x, bool) else x for x in fields)))

    apply_mesh = None

    def activated(line, coords, head):
        nonlocal apply_mesh
        if line.startswith("!mesh_cmd "):
            assert runtime and apply_mesh is not None
            apply_mesh(line.removeprefix("!mesh_cmd "), head)
            return
        assert line == "!use_profile"
        apply_mesh = stock_mesh_commands(cfg, coords)
        if not runtime:
            apply_mesh(f"BED_MESH_PROFILE LOAD={PROFILE}", head)
        assert head.commanded_pos[:3] == list(origin)

    def moves(items, dwell):
        nonlocal clock
        for m in items:
            cruise = 0. if -1e-12 <= m.cruise_t < 0. else m.cruise_t
            m.cruise_t = cruise
            if m.is_kinematic_move:
                rows.append(("m", clock, m.accel_t, cruise, m.decel_t,
                             *m.start_pos[:3], *m.axes_r[:3], m.start_v, m.cruise_v, m.accel))
            if m.axes_d[3]:
                assert extrusion is not None
                extrusion.process_move(clock, m, 3)
            clock += m.accel_t
            clock += cruise
            clock += m.decel_t
            if runtime_e:
                for callback in m.timing_callbacks:
                    callback(clock)
        clock += dwell

    commands = script.read_text().splitlines()
    if runtime:
        commands = [f"!mesh_cmd {line}" if line.startswith("BED_MESH_") else line for line in commands]
    options = dict(settings=settings, emit_profiles=moves, extend_directive=activated,
                   simulated_extruder=extrusion)
    if runtime_e:
        import chelper
        from unittest.mock import patch
        stepper = object.__new__(ExtruderStepper)
        stepper.name = "extruder"
        stepper.sk_extruder = object()
        stepper.pressure_advance = ext_cfg.getfloat("pressure_advance")
        stepper.pressure_advance_smooth_time = .04
        gains.append((0., stepper.pressure_advance, stepper.pressure_advance_smooth_time))
        ffi = SimpleNamespace(extruder_set_pressure_advance=lambda _, *args: gains.append(args))

        def pressure(head, command):
            head.get_last_move_time = lambda: clock
            stepper.printer = SimpleNamespace(lookup_object=lambda _: head, set_rollover_info=lambda *_: None)
            stepper.cmd_SET_PRESSURE_ADVANCE(command)

        with patch.object(chelper, "get_ffi", return_value=(None, ffi)):
            stock(["!use_profile", *commands], False, pressure_command=pressure, **options)
        assert [gain for _, gain, _ in gains] == [.05, .071, .033], gains
    else:
        stock(["!use_profile", *commands], False, **options)
    for (axis,distance) in zip(("+","-","z"),distances,strict=True):
        rows.append(("s", axis, *origin, distance, clock+.01,20e6,0,0))
    if runtime_e:
        rows.extend(("g", *gain) for gain in gains)
        rows.append(("s", "e", 0., 0., 0., .004, clock+.08, 20e6, 0, 0))
    data = "\n".join(" ".join(map(str,row)) for row in rows)+"\n"
    result = subprocess.run([executable], input=data, text=True, capture_output=True, check=True)
    return result.stdout.splitlines(), clock-2.



def cli_rows(result):
    lanes = [*result["motors"], *(e["commands"] for e in result.get("extruder_lanes", []))]
    rows = []
    for lane in lanes:
        for row in lane:
            if row["kind"] == "direction":
                rows.append(f'd {int(row["positive"])}')
            else:
                rows.append(f's {row["interval"]} {row["count"]} {row["add"]} {row["last_clock"]}')
        rows.append("end")
    return rows

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
            actual = cli_rows(parsed)
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
        actual = cli_rows(got)
        for i, (want, actual_row) in enumerate(zip(expected, actual, strict=True)):
            assert want == actual_row, (i, want, actual_row)
        print(f"runtime saved mesh load/clear: {len(actual)-3} exact timed stock motor records")
        before_load = "BED_MESH_OFFSET X=74 Y=56"
        while_clear = "BED_MESH_OFFSET X=-71 Y=-55"
        offset_commands = list(commands)
        for anchor, command in (
            ("G1 X15 Y12 Z0.32 F1800", before_load),
            ("G1 X30 Y24 Z0.5", "BED_MESH_OFFSET X=7.25 Y=-3.5"),
            ("G1 X50 Y32", "BED_MESH_OFFSET X=-4"),
            ("G1 X72 Y45", while_clear),
            ("G1 X130 Y60 Z1.3", "BED_MESH_OFFSET Y=4.5"),
            ("G1 X125 Y78 Z2.3", "BED_MESH_OFFSET"),
        ):
            offset_commands.insert(offset_commands.index(anchor), command)
        clear_index = offset_commands.index("BED_MESH_CLEAR")
        offset_commands[clear_index:clear_index] = [f"BED_MESH_PROFILE LOAD={PROFILE}", "G1 X57 Y37"]
        offset_commands.insert(offset_commands.index("G1 X50 Y32"), "BED_MESH_OFFSET")
        path.write_text("\n".join(offset_commands) + "\n")
        offset_expected, offset_duration = reference(upstream, c, cfg, path, runtime=True)
        offset_result = json.loads(subprocess.check_output(cmd))
        offset_rows = cli_rows(offset_result)
        assert math.isclose(offset_result["planned_duration_seconds"], offset_duration, rel_tol=1e-12, abs_tol=1e-12)
        for i, (want, row) in enumerate(zip(offset_expected, offset_rows, strict=True)):
            assert want == row, (i, want, row)
        assert offset_result["motors"][2] != got["motors"][2]
        # Omitted Y preserves -3.5; a fresh LOAD restores both to zero.
        # Inactive updates are inert and do not persist into the next load.
        explicit = {before_load: "", while_clear: "", "BED_MESH_OFFSET X=-4":
                    "BED_MESH_OFFSET X=-4 Y=-3.5", "G1 X23 Y10 Z-.2":
                    "BED_MESH_OFFSET X=0 Y=0\nG1 X23 Y10 Z-.2", "G1 X57 Y37":
                    "BED_MESH_OFFSET X=0 Y=0\nG1 X57 Y37"}
        path.write_text("\n".join(explicit.get(line, line) for line in offset_commands) + "\n")
        assert json.loads(subprocess.check_output(cmd)) == offset_result
        print(f"runtime saved mesh XY offsets: {len(offset_rows)-3} exact timed stock motor records")

        faded = {
            before_load: f"{before_load} ZFADE=4",
            "BED_MESH_OFFSET X=7.25 Y=-3.5": "BED_MESH_OFFSET X=7.25 Y=-3.5 ZFADE=0.8",
            "G1 X50 Y32": "BED_MESH_OFFSET ZFADE=-1.25\nG1 X50 Y32",
            while_clear: f"{while_clear} ZFADE=-4",
            "BED_MESH_OFFSET Y=4.5": "BED_MESH_OFFSET Y=4.5 ZFADE=3",
        }
        fade_commands = [faded.get(line, line) for line in offset_commands]
        path.write_text("\n".join(fade_commands) + "\n")
        fade_expected, fade_duration = reference(upstream, c, cfg, path, runtime=True)
        fade_result = json.loads(subprocess.check_output(cmd))
        assert math.isclose(fade_result["planned_duration_seconds"], fade_duration, rel_tol=1e-12, abs_tol=1e-12)
        fade_rows = cli_rows(fade_result)
        for i, (want, row) in enumerate(zip(fade_expected, fade_rows, strict=True)):
            assert want == row, (i, want, row)
        assert fade_result["motors"][2] != offset_result["motors"][2]
        # XY-only updates retain ZFADE; reloading an already-active mesh
        # discards all offsets, not just X/Y. Inactive commands are inert.
        explicit_fade = {
            faded[before_load]: "", faded[while_clear]: "",
            "BED_MESH_OFFSET X=-4": "BED_MESH_OFFSET X=-4 ZFADE=0.8",
            "G1 X57 Y37": "BED_MESH_OFFSET ZFADE=0\nG1 X57 Y37",
        }
        path.write_text("\n".join(explicit_fade.get(line, line) for line in fade_commands) + "\n")
        assert json.loads(subprocess.check_output(cmd)) == fade_result
        print(f"runtime saved mesh ZFADE: {len(fade_rows)-3} exact timed stock motor records")

        disabled_root = Path(d)
        for name in ("printer.cfg", "rails.cfg"):
            (disabled_root / name).write_text((fixture / name).read_text())
        disabled = disabled_root / "disabled-fade.cfg"
        disabled.write_text((fixture / "mesh.cfg").read_text().replace("fade_end: 3.5", "fade_end: 0.0"))
        dc = [*cmd[:4], disabled_root, disabled.name, *cmd[6:]]
        path.write_text("\n".join(fade_commands) + "\n")
        off_expected, off_duration = reference(upstream, c, disabled, path, runtime=True)
        off = json.loads(subprocess.check_output(dc))
        assert math.isclose(off["planned_duration_seconds"], off_duration, rel_tol=1e-12, abs_tol=1e-12)
        assert cli_rows(off) == off_expected
        path.write_text("\n".join(offset_commands) + "\n")
        assert json.loads(subprocess.check_output(dc)) == off

        path.write_text("G90\nBED_MESH_OFFSET X=2\n")
        denied = subprocess.run([x for x in cmd if x != "--mesh-script"], capture_output=True)
        assert denied.returncode and not denied.stdout
        for invalid in ("BED_MESH_PROFILE LOAD=missing", f"BED_MESH_PROFILE SAVE={PROFILE}",
                        f"BED_MESH_PROFILE REMOVE={PROFILE}", "BED_MESH_CALIBRATE",
                        "BED_MESH_CLEAR X=1", "BED_MESH_OFFSET ZFADE=1001",
                        "BED_MESH_OFFSET ZFADE=nan", "BED_MESH_OFFSET ZFADE=one",
                        "BED_MESH_OFFSET Y=infinity", "BED_MESH_OFFSET X=nan",
                        "BED_MESH_OFFSET X=bad", "BED_MESH_OFFSET X=1 X=2",
                        "BED_MESH_OFFSET X", "BED_MESH_OFFSET EXTRA=2"):
            path.write_text(f"G90\n{invalid}\n")
            denied = subprocess.run(cmd, capture_output=True)
            assert denied.returncode and not denied.stdout, (invalid, denied.stdout)

        e_root = Path(d)
        for name in ("printer.cfg", "rails.cfg"):
            (e_root/name).write_text((fixture/name).read_text())
        e_cfg = e_root/"mesh-plus-e.cfg"
        rear = (fixture/"single-pressure.cfg").read_text().split("[extruder]", 1)[1]
        e_cfg.write_text((fixture/"mesh.cfg").read_text() + "\n[extruder]" + rear +
                         "\nmax_extrude_only_distance: 15\nmax_extrude_only_velocity: 20\nmax_extrude_only_accel: 300\n")
        active = ["G90", "M83", "G1 X15 Y12 Z.32 E.12 F1800", f"BED_MESH_PROFILE LOAD={PROFILE}",
                  "G1 X22 Y20 E.18", "SET_PRESSURE_ADVANCE ADVANCE=.071",
                  "G1 X30 Y28 Z.55 E.20", "BED_MESH_CLEAR", "G1 X50 Y31 E.21",
                  "G1 E-.04 F240", f"BED_MESH_PROFILE LOAD={PROFILE}", "SET_PRESSURE_ADVANCE ADVANCE=.033",
                  "G1 X64 Y48 E.16 F1200", "BED_MESH_CLEAR", "G1 X70 Y61 Z1.2 E.25", "M400"]
        path.write_text("\n".join(active)+"\n")
        args = [a.binary, "--mesh-script", "--mesh-profile", PROFILE,
                "--fixed-pressure-script", "--tool-temps", "0:210",
                e_root, e_cfg.name, path, "20000000", "1", "1", ".2", "printer.cfg", "rails.cfg"]
        got = json.loads(subprocess.check_output(args))
        expected, duration = reference(upstream, c, e_cfg, path, runtime=True, runtime_e=True)
        assert math.isclose(got["planned_duration_seconds"], duration, rel_tol=1e-12, abs_tol=1e-12)
        decoded = cli_rows(got)
        for i, (want, row) in enumerate(zip(expected, decoded, strict=True)):
            assert want == row, (i, want, row)
        path.write_text("\n".join(line for line in active if not line.startswith("SET_PRESSURE_ADVANCE"))+"\n")
        constant = json.loads(subprocess.check_output(args))
        assert constant["motors"] == got["motors"]
        assert constant["planned_duration_seconds"] == got["planned_duration_seconds"]
        assert constant["extruder_lanes"] != got["extruder_lanes"]
        print(f"runtime saved mesh with fixed-window pressure changes: {len(decoded)-4} exact XYZ+E stock motor records")


if __name__ == "__main__":
    main()
