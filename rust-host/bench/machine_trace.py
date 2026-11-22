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


def traced(config_path, c):
    from configfile import ConfigFileReader, ConfigWrapper
    from kinematics.cartesian import CartKinematics
    from kinematics.corexy import CoreXYKinematics
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

    def output(moves, dwell):
        nonlocal time
        for m in moves:
            if not m.is_kinematic_move:
                raise ValueError("unsupported fixture extrusion")
            cruise = 0. if -1e-12 <= m.cruise_t < 0. else m.cruise_t
            rows.append(("m", time, m.accel_t, cruise, m.decel_t,
                         *m.start_pos[:3], *m.axes_r[:3], m.start_v, m.cruise_v, m.accel))
            time = time + m.accel_t + cruise + m.decel_t
        time += dwell

    stock(GCODE.splitlines(), False, settings=settings, emit_profiles=output)
    elapsed = time - 2.
    for axis, distance, invert in zip(("x", "y", "z") if cart else ("+", "-", "z"),
                                     distances, (0, 1, 0), strict=True):
        rows.append(("s", axis, *start, distance, time + .01, 20e6, 20, invert))
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
                                                 "kin_corexy", "pyhelper", "msgblock")), "-lm", "-o", c],
                        check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/machine_trace.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        cfg = d / "machine.cfg"
        for kin in ("cartesian", "corexy"):
            cfg.write_text(CONFIG.format(kinematics=kin))
            want, expected_time = traced(cfg, c)
            actual = subprocess.check_output([rust, cfg], input=GCODE, text=True).splitlines()
            timeline = float(actual.pop().split()[1])
            assert math.isclose(expected_time, timeline, abs_tol=1e-12, rel_tol=1e-12)
            for i, (x, y) in enumerate(zip(want, actual, strict=True)):
                assert x == y, (kin, i, x, y)
            print(f"{kin}: {len(want)-3} exact direction/step commands, {timeline:.9f} simulated seconds")


if __name__ == "__main__":
    main()
