#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare an A-B-A shaped dual-extruder path against stock Python plus C."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

from gcode_preview import stock
from planner import PIN, ROOT
from preview_cli_timing import gcode as long_gcode


def reference(upstream, executable, sparse, long_script=None, parked=None, runtime=False):
    sys.path.insert(0, str(upstream / "klippy"))
    from kinematics.corexy import CoreXYKinematics
    from kinematics.extruder import ExtruderStepper, PrinterExtruder
    from toolhead import ToolHead
    import chelper
    from extras import shaper_defs
    origin = (10.,10.,.3) if long_script else (1.,1.,.2)
    cfg = CoreXYKinematics, [(0.,200.),(0.,180.),(0.,150.)], 180., 3100.,5.,.5,9.,160., origin
    compression_error = 0 if long_script else 20
    start = (0., 0., 0.) if long_script else (0., 1.3, 2.1) if sparse else (0., 1.3)
    if parked is not None:
        assert long_script and len(parked) == len(start)
        start = parked
    e_rows = [[] for _ in start]
    gains = [[("g", 0., gain, smooth)] for gain, smooth in
             ((.04, .04), (.04, .04), (.062, .055))[:len(start)]]
    xyz_rows = []
    clock = 2.
    active = 0
    positions = list(start)
    extruders = []
    for i in range(len(start)):
        e = object.__new__(PrinterExtruder)
        e.name = "extruder" + (str(i) if i else "")
        e.heater = SimpleNamespace(can_extrude=True)
        e.instant_corner_v = 1.0
        e.nozzle_diameter = .4
        e.filament_area = math.pi * (.5*1.75)**2
        e.max_extrude_ratio = .64/e.filament_area
        e.max_e_dist, e.max_e_velocity, e.max_e_accel = 15.,20.,300.
        e.last_position = start[i]
        e.trapq = object()
        e.trapq_append = lambda _, *fields, i=i: e_rows[i].append(
            ("e", *(int(v) if isinstance(v, bool) else v for v in fields)))
        if runtime:
            stepper = object.__new__(ExtruderStepper)
            stepper.name = e.name
            stepper.stepper = SimpleNamespace(get_trapq=lambda e=e: e.trapq)
            stepper.sk_extruder = i
            _, _, stepper.pressure_advance, stepper.pressure_advance_smooth_time = gains[i][0]
            e.extruder_stepper = stepper
        extruders.append(e)

    def out(moves, dwell):
        nonlocal clock
        for m in moves:
            cruise = 0. if -1e-12 <= m.cruise_t < 0. else m.cruise_t
            m.cruise_t = cruise
            if m.is_kinematic_move:
                xyz_rows.append(("m", clock, m.accel_t, cruise, m.decel_t,
                                 *m.start_pos[:3], *m.axes_r[:3], m.start_v, m.cruise_v, m.accel))
            if m.axes_d[3]:
                extruders[active].process_move(clock, m, 3)
            clock += m.accel_t
            clock += cruise
            clock += m.decel_t
            if runtime:
                for callback in m.timing_callbacks:
                    callback(clock)
        clock += dwell

    def switch(line, coords, head):
        nonlocal active
        ident = int(line.split()[1])
        if runtime:
            head.get_extruder = lambda: ToolHead.get_extruder(head)
            head._build_extra_axes_status = lambda: ToolHead._build_extra_axes_status(head)
            head.set_extruder = lambda *args: ToolHead.set_extruder(head, *args)

            def switch_flush():
                assert not head.lookahead.get_last(), "drain before switching traced lanes"
                head._flush_lookahead()

            # The activation handler may flush; the gain handler must not.
            old_flush, head.flush_step_generation = head.flush_step_generation, switch_flush
            e = extruders[ident]
            e.printer = SimpleNamespace(lookup_object=lambda name: head,
                                        send_event=lambda name: coords._handle_activate_extruder())
            e.cmd_ACTIVATE_EXTRUDER(SimpleNamespace(respond_info=lambda _: None))
            head.flush_step_generation = old_flush
            active = ident
            return
        positions[active] = head.commanded_pos[3]
        head.commanded_pos[3] = positions[ident]
        coords._handle_activate_extruder()
        active = ident
        head.extra_axes[0] = extruders[active]

    source = []
    if long_script:
        ids = {"T0": 0, "T2": 1, "T6": 2}
        selected = 0
        for line in long_script:
            if line in ids:
                if not runtime or selected != ids[line]:
                    source.append("M400")
                selected = ids[line]
                source.append(f"!use {selected}")
            else:
                source.append(line)
    else:
        order = [(0,"a"),(1,"b"),(2,"c"),(1,"d")] if sparse else [(0,"a"),(1,"b"),(0,"c")]
        for ident, name in order:
            source.extend(["M400", f"!use {ident}",
                           *(ROOT/f"examples/preview/multitool-{name}.gcode").read_text().splitlines()])
    def pressure_command(head, cmd):
        head.get_last_move_time = lambda: clock
        head.get_extruder = lambda: ToolHead.get_extruder(head)
        printer = SimpleNamespace(lookup_object=lambda name: head, set_rollover_info=lambda *_: None)
        for e in extruders:
            e.extruder_stepper.printer = printer
        # Always enter via stock's implicit-active handler, regardless of
        # which stepper registered the default mux command.
        extruders[0].extruder_stepper.cmd_default_SET_PRESSURE_ADVANCE(cmd)

    options = dict(settings=cfg, emit_profiles=out, simulated_extruder=extruders[0],
                   extend_directive=switch)
    if runtime:
        ffi = SimpleNamespace(extruder_set_pressure_advance=lambda i, *args: gains[i].append(("g", *args)))
        with patch.object(chelper, "get_ffi", return_value=(None, ffi)):
            stock(source, False, pressure_command=pressure_command, **options)
        assert [[row[2] for row in lane][1:] for lane in gains] == [
            [.034, .065, .084, .041], [.055, .043, .09], [.08, .032, .055],
        ], gains
        assert gains[0][2][1] == gains[0][3][1], gains
    else:
        stock(source, False, **options)
    focus = [shaper_defs.get_mzv_shaper(56.6, .1), shaper_defs.get_ei_shaper(48.4, .1)]
    for axis, (amplitudes, ticks) in zip("xy", focus, strict=True):
        xyz_rows.append(("h", axis, len(ticks),
                         *(v for pair in zip(ticks, amplitudes, strict=True) for v in pair)))
    for axis, step in zip(("+","-","z"), (.0125,.0125,.0025), strict=True):
        xyz_rows.append(("s", axis, *origin, step, clock+.51,20e6,compression_error,0))
    for i, (rows, start_e) in enumerate(zip(e_rows, start, strict=True)):
        rows.extend(gains[i] if runtime else [("g", 0., .04, .04)])
        rows.append(("s","e",start_e,0.,0.,.004,clock+.08,20e6,compression_error,
                     int(bool(long_script) and i == 1)))

    def encoded(rows):
        payload = chr(10).join(" ".join(map(str, row)) for row in rows)+chr(10)
        return subprocess.run([executable], input=payload, text=True, capture_output=True, check=True).stdout.splitlines()
    return [*encoded(xyz_rows), *(line for rows in e_rows for line in encoded(rows))], clock-2.


def compare_cli(upstream, executable, cli, directory):
    lines = list(long_gcode(2500, True))
    gcode = Path(directory) / "long-sparse.gcode"
    gcode.write_text("\n".join(lines) + "\n")
    command = ["--tool-temps", "0:210,2:210,6:210", ROOT/"examples/preview", "sparse-tool.cfg",
               gcode, "20000000", "10", "10", ".3", "rails.cfg", "printer.cfg"]
    raw = subprocess.check_output([cli, *command])
    for workers in range(2, 9):
        threaded = subprocess.check_output([cli, "--lane-workers", str(workers), *command])
        assert threaded == raw, workers
    trace = json.loads(raw)
    want, duration = reference(upstream, executable, True, lines)
    assert math.isclose(duration, trace["planned_duration_seconds"], rel_tol=1e-12, abs_tol=1e-12)
    received = cli_rows(trace)
    for i, (x,y) in enumerate(zip(want,received,strict=True)):
        assert x == y, (i,x,y)
    print(f"Long sparse CLI: {len(received)-6} exact motor records across six lanes")



def cli_rows(trace):
    received = []
    for lane in [*trace["motors"], *(t["commands"] for t in trace["extruder_lanes"])]:
        for cmd in lane:
            if cmd["kind"] == "direction":
                received.append(f'd {int(cmd["positive"])}')
            else:
                received.append(f's {cmd["interval"]} {cmd["count"]} {cmd["add"]} {cmd["last_clock"]}')
        received.append("end")
    return received


def compare_runtime(upstream, executable, cli, directory):
    base = ROOT / "examples/preview"
    root = Path(directory)
    for name in ("rails.cfg", "printer.cfg"):
        (root / name).write_text((base / name).read_text())
    original = (base / "sparse-tool.cfg").read_text()
    # T6 has a different fixed window. Tool profile overrides are not
    # updated by the script; the active lane's configured window is retained.
    (root / "pressure.cfg").write_text(original.replace("params_pressure_advance: 0.04",
        "params_pressure_advance: 0.062\nparams_pressure_advance_smooth_time: 0.055"))
    gcode = root / "gain.gcode"
    command = ["--tool-temps", "0:210,2:214,6:220", root, "pressure.cfg", gcode,
               "20000000", "10", "10", ".3", "rails.cfg", "printer.cfg"]

    def run(lines, flags=("--fixed-pressure-script",)):
        gcode.write_text("\n".join(lines) + "\n")
        return subprocess.run([cli, *flags, *command], capture_output=True, text=True)

    def decode(result):
        assert result.returncode == 0, result.stderr
        trace = json.loads(result.stdout)
        assert [lane["tool"] for lane in trace["extruder_lanes"]] == [0, 2, 6]
        return trace

    for count in (8, 96):
        moves = list(line for line in long_gcode(count, True) if line.startswith("G1"))
        def block(tag):
            # Preserve legal positive paths across repetitions without
            # repeating the same exact absolute starting point.
            return [line.replace("X10.", f"X{10+tag}.")
                    .replace("Z0.30000", f"Z{.3+tag*.01:.5f}") for line in moves]

        lines = [
            "G90", "M83", "SET_PRESSURE_ADVANCE ADVANCE=.034", *block(0),
            "G1 X11 F1500", "SET_PRESSURE_ADVANCE ADVANCE=.065",
            "SET_PRESSURE_ADVANCE ADVANCE=.084", "T2",
            "SET_PRESSURE_ADVANCE ADVANCE=.055", *block(1),
            "G1 E-.015 F270", "SET_PRESSURE_ADVANCE ADVANCE=.043", "T2", *block(2),
            "T6", "SET_PRESSURE_ADVANCE ADVANCE=.08", *block(3), "G4 P41",
            "SET_PRESSURE_ADVANCE ADVANCE=.032", *block(4), "T2", *block(5),
            "SET_PRESSURE_ADVANCE ADVANCE=.09", *block(6),
            "T0", *block(7), "SET_PRESSURE_ADVANCE ADVANCE=.041", *block(8),
            "T6", *block(9), "SET_PRESSURE_ADVANCE ADVANCE=.055", "T2",
        ]
        result = decode(run(lines))
        expected, duration = reference(upstream, executable, True, lines, runtime=True)
        assert math.isclose(result["planned_duration_seconds"], duration, rel_tol=1e-12, abs_tol=1e-12)
        rows = cli_rows(result)
        for i, (want, got) in enumerate(zip(expected, rows, strict=True)):
            assert want == got, (count, i, want, got)
        control = decode(run([line for line in lines if not line.startswith("SET_PRESSURE_ADVANCE")],
                             flags=()))
        assert result["motors"] == control["motors"]
        assert result["planned_duration_seconds"] == control["planned_duration_seconds"]
        assert all(a["commands"] != b["commands"] for a, b in
                   zip(result["extruder_lanes"], control["extruder_lanes"], strict=True))
        threaded = decode(run(lines, flags=("--fixed-pressure-script", "--lane-workers", "4")))
        assert threaded == result
        sizes = [len(lane) for lane in result["motors"]] + [
            len(lane["commands"]) for lane in result["extruder_lanes"]]
        print(f"Sparse runtime gain, {10*count} shaped moves: {sum(sizes)} exact timed stock "
              f"records (XYZ, T0/T2/T6: {sizes})")

    for bad, flags in (
        ("SET_PRESSURE_ADVANCE ADVANCE=0", ("--fixed-pressure-script",)),
        ("SET_PRESSURE_ADVANCE ADVANCE=.08 SMOOTH_TIME=.04", ("--fixed-pressure-script",)),
        ("SET_PRESSURE_ADVANCE EXTRUDER=extruder1 ADVANCE=.08", ("--fixed-pressure-script",)),
        ("SET_PRESSURE_ADVANCE ADVANCE=.08", ()),
    ):
        rejected = run(["T2", bad], flags=flags)
        assert rejected.returncode and not rejected.stdout, rejected
    for unsupported in ("INITIALIZE_TOOLCHANGER T=2", "T2 RESTORE_AXIS=XY"):
        rejected = run([unsupported])
        assert rejected.returncode and not rejected.stdout, rejected
    rejected = run(["T2", "SET_PRESSURE_ADVANCE ADVANCE=.08"],
                   flags=("--fixed-pressure-script", "--mixed-plan"))
    assert rejected.returncode and not rejected.stdout, rejected
    (root / "pressure.cfg").write_text(original.replace("params_pressure_advance: 0.04",
                                                        "params_pressure_advance: 0"))
    rejected = run(["T6", "SET_PRESSURE_ADVANCE ADVANCE=.08"])
    assert rejected.returncode and not rejected.stdout, rejected

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/tool_motion_replay")
    p.add_argument("--cli-binary", type=Path, help="also verify the 2,500-move sparse CLI regression")
    a = p.parse_args()
    upstream = a.upstream.resolve()
    h = upstream/"klippy/chelper"
    with tempfile.TemporaryDirectory() as d:
        c = Path(d)/"reference"
        subprocess.run([os.environ.get("CC","cc"),"-O3","-I",h,ROOT/"bench/machine_trace.c",
                        *(h/(name+".c") for name in ["itersolve","stepcompress","trapq","kin_cartesian",
                          "kin_corexy","kin_extruder","kin_shaper","pyhelper","msgblock"]),
                        "-lm","-o",c], check=True)
        for sparse in (False, True):
            want, expected_time = reference(upstream, c, sparse)
            got = subprocess.check_output([a.binary, *(["--sparse"] if sparse else [])], text=True).splitlines()
            duration = float(got.pop().split()[1])
            assert math.isclose(expected_time, duration, abs_tol=1e-12, rel_tol=1e-12), (duration, expected_time)
            for i, (x, y) in enumerate(zip(want, got, strict=True)):
                assert x == y, (sparse, i, x, y)
            lane_count = 6 if sparse else 5
            ids = "T0/T2/T6/T2" if sparse else "T0/T1/T0"
            print(f"{ids}: {len(got)-lane_count} exact step commands on {lane_count} lanes; "
                  f"{duration:.9f} simulated seconds")
        if a.cli_binary:
            compare_cli(upstream, c, a.cli_binary.resolve(), d)
            compare_runtime(upstream, c, a.cli_binary.resolve(), d)



if __name__ == "__main__":
    main()
