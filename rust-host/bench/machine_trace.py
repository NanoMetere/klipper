#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare config-driven virtual XYZ step traces against stock Python + C."""
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from gcode_preview import stock
from planner import PIN, ROOT

CONFIG = """[printer]
kinematics: {kinematics}
max_velocity: 180
max_accel: 3100
max_z_velocity: 9
max_z_accel: 160
square_corner_velocity: 5
minimum_cruise_ratio: .6
[stepper_x]
rotation_distance: 40
microsteps: 16
position_min: 0
position_max: 200
[stepper_y]
rotation_distance: 40
microsteps: 16
position_max: 180
[stepper_z]
rotation_distance: 8
microsteps: 8
position_max: 150
gear_ratio: 2:1
"""
GCODE = """G21
G90
G1 X4.125 Y5.25 Z0.5 F1800
G1 X5.75 Y5.25 F4800
G1 X7 Y2.125
G1 X3.75 Y4.5
G4 P117
G91
G1 X1.125 Y.875 Z-.065 F5400
M220 S90
G1 X-.8 Y.7
G92 X0 Y0
G90
G1 X.6 Y2
M400
G1 X.51 Y1.81 Z.61
G1 X.6 Y2
G1 X.54 Y1.92
"""

WITH_E = """G21
M82
G90
G1 X4.125 Y5.25 Z0.5 E.25 F1800
G1 X5.75 Y5.25 E.36 F4800
G1 X7 Y2.125 E.5
G91
M83
G1 E-.6 F600
G1 X-.8 Y.7 E-.12 F1200
G4 P117
G1 E.75 F900
M221 S110
G1 X.8 Y.2 E.03 F1600
G90
M82
G92 E0
G1 X3.8 Y4.7 E.22 F2100
M400
"""



def traced(config_path, c, script, use_extruder):
    from configfile import ConfigFileReader, ConfigWrapper
    from kinematics.cartesian import CartKinematics
    from kinematics.corexy import CoreXYKinematics
    from kinematics.extruder import PrinterExtruder
    from types import SimpleNamespace
    from stepper import parse_step_distance
    reader = ConfigFileReader()
    raw = reader.build_fileconfig_with_includes(reader.read_config_file(config_path), config_path)
    printer = ConfigWrapper(None, raw, {}, "printer")
    cart = printer.get("kinematics") == "cartesian"
    start = (1., 1., .2)
    rails = [ConfigWrapper(None, raw, {}, f"stepper_{a}") for a in "xyz"]
    ranges = [(r.getfloat("position_min", 0.), r.getfloat("position_max")) for r in rails]
    distances = [pos / micro for pos, micro in (parse_step_distance(r, False) for r in rails)]
    settings = ((CartKinematics if cart else CoreXYKinematics), ranges,
                *(printer.getfloat(k) for k in ("max_velocity", "max_accel", "square_corner_velocity",
                                                "minimum_cruise_ratio", "max_z_velocity", "max_z_accel")),
                start)
    rows = []
    time = 2.

    extruder = None
    if use_extruder:
        extruder = object.__new__(PrinterExtruder)
        extruder.heater = SimpleNamespace(can_extrude=True)
        extruder.instant_corner_v = 1.0
        extruder.nozzle_diameter = .4
        extruder.filament_area = math.pi * (1.75 * .5)**2
        extruder.max_extrude_ratio = 0.64 / extruder.filament_area
        extruder.max_e_dist, extruder.max_e_velocity, extruder.max_e_accel = 15., 20., 300.
        extruder.trapq = None
        extruder.trapq_append = lambda _, *fields: rows.append(("e", *(int(x) if isinstance(x, bool) else x for x in fields)))

    def output(moves, dwell):
        nonlocal time
        for m in moves:
            cruise = 0. if -1e-12 <= m.cruise_t < 0. else m.cruise_t
            m.cruise_t = cruise
            if m.is_kinematic_move:
                rows.append(("m", time, m.accel_t, cruise, m.decel_t,
                             *m.start_pos[:3], *m.axes_r[:3], m.start_v, m.cruise_v, m.accel))
            if m.axes_d[3]:
                assert extruder is not None
                extruder.process_move(time, m, 3)
            time = time + m.accel_t + cruise + m.decel_t
        time += dwell

    stock(script.splitlines(), False, settings=settings, emit_profiles=output,
          simulated_extruder=extruder)
    elapsed = time - 2.
    for axis, distance, invert in zip(("x", "y", "z") if cart else ("+", "-", "z"),
                                     distances, (0, 1, 0), strict=True):
        rows.append(("s", axis, *start, distance, time + .01, 20e6, 20, invert))
    if use_extruder:
        rows.append(("s", "e", 0., 0., 0., 0.004, time + .01, 20e6, 20, 1))
    trace = subprocess.check_output([c], text=True, input="\n".join(" ".join(map(str, r)) for r in rows)+"\n")
    return trace.splitlines(), elapsed


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    upstream = p.parse_args().upstream.resolve()
    sys.path.insert(0, str(upstream / "klippy"))
    h = upstream / "klippy/chelper"
    with tempfile.TemporaryDirectory() as dir_:
        d = Path(dir_)
        c, rust, lib = d / "stock", d / "rust", d / "libklipper_host_protocol.rlib"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", h, ROOT / "bench/machine_trace.c",
                        *(h / (n+".c") for n in ("itersolve", "stepcompress", "trapq", "kin_cartesian",
                                                 "kin_corexy", "kin_extruder", "pyhelper", "msgblock")), "-lm", "-o", c],
                        check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/machine_trace.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        cfg = d / "machine.cfg"
        for kin in ("cartesian", "corexy"):
            cfg.write_text(CONFIG.format(kinematics=kin))
            for e_enabled in (False, True):
                script = WITH_E if e_enabled else GCODE
                args = [rust, cfg] + (["--extruder"] if e_enabled else [])
                want, expected_time = traced(cfg, c, script, e_enabled)
                actual = subprocess.check_output(args, input=script, text=True).splitlines()
                timeline = float(actual.pop().split()[1])
                assert math.isclose(expected_time, timeline, abs_tol=1e-12, rel_tol=1e-12)
                for i, (x, y) in enumerate(zip(want, actual, strict=True)):
                    assert x == y, (kin, e_enabled, i, x, y)
                motor_count = 4 if e_enabled else 3
                print(f"{kin}, {motor_count} motors: {len(want)-motor_count} exact commands; "
                      f"{timeline:.9f} simulated seconds")


if __name__ == "__main__":
    main()
