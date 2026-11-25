#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare config-driven virtual XYZ step traces against stock Python + C."""
import argparse
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time as perf

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

RUNTIME_LIMITS = """G90
M83
G1 X4 Y2 Z.3 E.15 F3000
M204 S700
G1 X7 Y3 E.13
SET_VELOCITY_LIMIT VELOCITY=34 ACCEL=950 SQUARE_CORNER_VELOCITY=2 MINIMUM_CRUISE_RATIO=.28
G1 X12 Y5 E.19 F5400
M204 P900 T420
G1 X6 Y8 E.21 F2400
SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=.4
G1 X5 Y4 E.11
M400
"""

RUNTIME_PRESSURE = """G90
M83
SET_PRESSURE_ADVANCE ADVANCE=.034
G1 X5 Y3 Z.3 E.19 F1600
SET_PRESSURE_ADVANCE ADVANCE=.065
SET_PRESSURE_ADVANCE ADVANCE=.084
G1 X9 Y4 E.22
G1 X11 Y5 E.15
G4 P37
SET_PRESSURE_ADVANCE ADVANCE=.052
G1 X12 Y7 E.12 F1900
G1 E-.04 F300
SET_PRESSURE_ADVANCE ADVANCE=.061
G1 X15 Y8 E.18 F1800
G1 X14 Y9 E.13
M400
"""


def rolling_pressure():
    # Hundreds of short moves drive the real lazy look-ahead path.
    script = ["G90", "M83", "SET_PRESSURE_ADVANCE ADVANCE=.034"]
    for i in range(320):
        if i == 51:
            script.extend(("SET_PRESSURE_ADVANCE ADVANCE=.065",
                           "SET_PRESSURE_ADVANCE ADVANCE=.084"))
        if i == 151:
            script.extend(("M400", "SET_PRESSURE_ADVANCE ADVANCE=.052"))
        if i == 279:
            script.append("SET_PRESSURE_ADVANCE ADVANCE=.061")
        script.append(f"G1 X{10 + (i % 47) * .0625:.4f} Y{12 + (i % 31) * .125:.4f} Z.3 E.003 F2100")
    return "\n".join((*script, "M400", ""))


SHAPES = [
    [(0.0, 1.0), (0.013, 0.8)],
    [(0.0, 0.9), (0.010, 1.6), (0.020, 0.7)],
    [(0.0, 1.0), (0.030, 0.6)],
]


def traced(config_path, c, script, use_extruder, pressure, shaped=False, runtime_pressure=False,
           native_samples=None):
    from configfile import ConfigFileReader, ConfigWrapper
    from kinematics.cartesian import CartKinematics
    from kinematics.corexy import CoreXYKinematics
    from kinematics.extruder import PrinterExtruder, ExtruderStepper
    from types import SimpleNamespace
    from stepper import parse_step_distance
    reader = ConfigFileReader()
    config_text = reader.read_config_file(config_path)
    started = perf.process_time_ns()
    raw = reader.build_fileconfig_with_includes(config_text, config_path)
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
    gains = []
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
            if runtime_pressure:
                for callback in m.timing_callbacks:
                    callback(time)
        time += dwell

    pressure_command = None
    if runtime_pressure:
        import chelper
        from unittest.mock import patch

        stepper = object.__new__(ExtruderStepper)
        stepper.name = "extruder"
        stepper.sk_extruder = object()
        stepper.pressure_advance = .05
        stepper.pressure_advance_smooth_time = .04

        def pressure_command(head, command):
            head.get_last_move_time = lambda: time
            stepper.printer = SimpleNamespace(
                lookup_object=lambda name: head,
                set_rollover_info=lambda *_: None,
            )
            ExtruderStepper.cmd_SET_PRESSURE_ADVANCE(stepper, command)

        ffi = SimpleNamespace(extruder_set_pressure_advance=lambda _, *args: gains.append(args))
        with patch.object(chelper, "get_ffi", return_value=(None, ffi)):
            stock(script.splitlines(), False, settings=settings, emit_profiles=output,
                  simulated_extruder=extruder, pressure_command=pressure_command)
        assert len(gains) == 5 and gains[1][0] == gains[2][0], gains
    else:
        stock(script.splitlines(), False, settings=settings, emit_profiles=output,
              simulated_extruder=extruder)
    elapsed = time - 2.
    shapes = SHAPES if shaped else None
    if raw.has_section("input_shaper"):
        from extras.input_shaper import InputShaperParams
        shape_config = ConfigWrapper(None, raw, {}, "input_shaper")
        shapes = []
        for axis in "xyz":
            _, amplitudes, times = InputShaperParams(axis, shape_config).get_shaper()
            shapes.append(list(zip(times, amplitudes, strict=True)))
    python_ns = perf.process_time_ns() - started
    if shapes:
        rows.extend(("h", axis, len(shape), *(p for pair in shape for p in pair))
                    for axis, shape in zip("xyz", shapes, strict=True))
    for axis, distance, invert in zip(("x", "y", "z") if cart else ("+", "-", "z"),
                                     distances, (0, 1, 0), strict=True):
        rows.append(("s", axis, *start, distance, time + (.51 if shapes else .01), 20e6, 20, invert))
    if use_extruder:
        if runtime_pressure:
            rows.append(("g", 0., .05, .04))
            rows.extend(("g", *entry) for entry in gains)
        elif pressure:
            rows.extend([("g", 0., .05, .04), ("g", 2.46, .075, .04)])
        rows.append(("s", "e", 0., 0., 0., 0.004, time + .08, 20e6, 20, 1))
    run = subprocess.run([c], text=True, capture_output=True, check=True,
                         input="\n".join(" ".join(map(str, r)) for r in rows)+"\n")
    native_ns = int(run.stderr.split()[1])
    if native_samples is not None:
        native_samples.append(native_ns)
    return run.stdout.splitlines(), elapsed, python_ns+native_ns


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--rounds", type=int, default=0,
                   help="additional interleaved CPU timing rounds for the longer CoreXY fixture")
    p.add_argument("--baseline-rust", type=Path,
                   help="optional previous rust-host checkout using the same benchmark ABI")
    p.add_argument("--shape-timing", action="store_true",
                   help="also configure input shaping for the timed long CoreXY fixture")
    a = p.parse_args()
    if not 0 <= a.rounds <= 1000:
        p.error("--rounds must be between 0 and 1000")
    if a.baseline_rust and not a.rounds:
        p.error("--baseline-rust requires --rounds")
    if a.baseline_rust and a.shape_timing and not (a.baseline_rust/"src/input_shaper.rs").is_file():
        p.error("a shaped baseline must include the fixed input-shaper module")
    upstream = a.upstream.resolve()
    sys.path.insert(0, str(upstream / "klippy"))
    h = upstream / "klippy/chelper"
    with tempfile.TemporaryDirectory() as dir_:
        d = Path(dir_)
        c, rust, lib = d / "stock", d / "rust", d / "libklipper_host_protocol.rlib"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", h, ROOT / "bench/machine_trace.c",
                        *(h / (n+".c") for n in ("itersolve", "stepcompress", "trapq", "kin_cartesian",
                                                 "kin_corexy", "kin_extruder", "kin_shaper", "pyhelper", "msgblock")), "-lm", "-o", c],
                        check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        clock = d / "clock.o"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", h,
                        "-DMACHINE_TRACE_CLOCK_ONLY", "-c", ROOT / "bench/machine_trace.c",
                        "-o", clock], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--cfg", "input_shaper_fixture", ROOT / "bench/machine_trace.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-C", f"link-arg={clock}", "-o", rust], check=True)
        executables = {"rust": rust}
        if a.baseline_rust:
            baseline = a.baseline_rust.resolve()
            lib_dir = d / "baseline"
            lib_dir.mkdir()
            lib = lib_dir / lib.name
            binary = lib_dir / "rust"
            subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                            "--crate-name=klipper_host_protocol", baseline / "src/lib.rs", "-o", lib],
                           check=True)
            # Use the exact same driver and timing clock for both libraries.
            subprocess.run(["rustc", "--edition=2021", "-O",
                            *(["--cfg", "input_shaper_fixture"] if a.shape_timing else []),
                            ROOT / "bench/machine_trace.rs", "--extern", f"klipper_host_protocol={lib}", "-C", f"link-arg={clock}",
                            "-o", binary], check=True)
            executables["baseline rust"] = binary
        cfg = d / "machine.cfg"
        for kin in ("cartesian", "corexy"):
            cfg.write_text(CONFIG.format(kinematics=kin))
            for e_enabled, pressure in ((False, False), (True, False), (True, True)):
                script = WITH_E if e_enabled else GCODE
                args = [rust, cfg] + (["--pressure" if pressure else "--extruder"] if e_enabled else [])
                want, expected_time, _ = traced(cfg, c, script, e_enabled, pressure)
                actual = subprocess.run(args, input=script, text=True,
                                        capture_output=True, check=True).stdout.splitlines()
                timeline = float(actual.pop().split()[1])
                assert math.isclose(expected_time, timeline, abs_tol=1e-12, rel_tol=1e-12)
                for i, (x, y) in enumerate(zip(want, actual, strict=True)):
                    assert x == y, (kin, e_enabled, i, x, y)
                motor_count = 4 if e_enabled else 3
                print(f"{kin}, {motor_count} motors, pressure={pressure}: {len(want)-motor_count} exact commands; "
                      f"{timeline:.9f} simulated seconds")

        for kin in ("cartesian", "corexy"):
            cfg.write_text(CONFIG.format(kinematics=kin))
            for extra in (False, True):
                script = WITH_E if extra else GCODE
                want, want_duration, _ = traced(cfg, c, script, extra, extra, shaped=True)
                run = subprocess.run([rust, cfg, "--shape", *(["--pressure"] if extra else [])],
                                     input=script, text=True, capture_output=True)
                if run.returncode:
                    raise RuntimeError(run.stderr)
                got = run.stdout.splitlines()
                duration = float(got.pop().split()[1])
                assert math.isclose(want_duration, duration, abs_tol=1e-12, rel_tol=1e-12)
                for i, (x, y) in enumerate(zip(want, got, strict=True)):
                    assert x == y, ("shaped", kin, extra, i, x, y)
                print(f"Shaped {kin}, pressure={extra}: {len(got) - 3 - extra} exact commands")
        for kin in ("cartesian", "corexy"):
            for kinds in (("mzv", "2hump_ei", "3hump_ei"), ("ei", "zvd", "zv")):
                shape_cfg = "[input_shaper]\n" + "\n".join(
                    f"shaper_type_{axis}: {kind}\nshaper_freq_{axis}: {freq}\ndamping_ratio_{axis}: {damp}"
                    for axis, kind, freq, damp in zip("xyz", kinds,
                                                       [53.4, 66.2, 41.3], [.13, .06, .17], strict=True))
                cfg.write_text(CONFIG.format(kinematics=kin) + shape_cfg + "\n")
                script = WITH_E
                want, want_duration, _ = traced(cfg, c, script, True, True)
                run = subprocess.run([rust, cfg, "--pressure"], input=script,
                                     text=True, capture_output=True)
                if run.returncode:
                    raise RuntimeError(run.stderr)
                got = run.stdout.splitlines()
                duration = float(got.pop().split()[1])
                assert math.isclose(want_duration, duration, abs_tol=1e-12, rel_tol=1e-12)
                for i, (x, y) in enumerate(zip(want, got, strict=True)):
                    assert x == y, ("configured shaper", kin, kinds, i, x, y)
                print(f"Configured shaper {kin}, {kinds}: {len(got) - 4} exact commands")
        for kin in ("cartesian", "corexy"):
            cfg.write_text(CONFIG.format(kinematics=kin))
            want, want_duration, _ = traced(cfg, c, RUNTIME_LIMITS, True, True, shaped=True)
            run = subprocess.run([rust, cfg, "--shape", "--pressure"], input=RUNTIME_LIMITS,
                                 text=True, capture_output=True, check=True)
            got = run.stdout.splitlines()
            duration = float(got.pop().split()[1])
            assert math.isclose(want_duration, duration, abs_tol=1e-12, rel_tol=1e-12)
            for i, (x, y) in enumerate(zip(want, got, strict=True)):
                assert x == y, ("runtime velocity limits", kin, i, x, y)
            print(f"Runtime limits, shaped {kin}: {len(got) - 4} exact commands")
        for kin in ("cartesian", "corexy"):
            cfg.write_text(CONFIG.format(kinematics=kin))
            for tag, script in (("boundaries", RUNTIME_PRESSURE), ("rolling", rolling_pressure())):
                want, want_duration, _ = traced(cfg, c, script, True, True,
                                                shaped=True, runtime_pressure=True)
                run = subprocess.run([rust, cfg, "--shape", "--runtime-pressure"],
                                     input=script, text=True, capture_output=True, check=True)
                got = run.stdout.splitlines()
                duration = float(got.pop().split()[1])
                assert math.isclose(want_duration, duration, abs_tol=1e-12, rel_tol=1e-12)
                for i, (x, y) in enumerate(zip(want, got, strict=True)):
                    assert x == y, ("runtime pressure advance", kin, tag, i, x, y)
                uncued = "\n".join(line for line in script.splitlines()
                                     if not line.startswith("SET_PRESSURE_ADVANCE")) + "\n"
                control = subprocess.run([rust, cfg, "--shape", "--runtime-pressure"],
                                         input=uncued, text=True, capture_output=True, check=True)
                no_gain_rows = control.stdout.splitlines()
                assert float(no_gain_rows.pop().split()[1]) == duration
                stop = [i for i, line in enumerate(got) if line == "end"][2] + 1
                assert got[:stop] == no_gain_rows[:stop], "gain change must not affect XYZ"
                assert got[stop:] != no_gain_rows[stop:], "gain change must affect E"
                print(f"Runtime pressure advance, shaped {kin}, {tag}: {len(got) - 4} exact commands")
        for args, command in (
            (("--runtime-pressure",), "SET_PRESSURE_ADVANCE ADVANCE=0"),
            (("--runtime-pressure",), "SET_PRESSURE_ADVANCE ADVANCE=.1 SMOOTH_TIME=.04"),
            (("--runtime-pressure",), "SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=.1"),
            (("--runtime-pressure",), "SET_PRESSURE_ADVANCE ADVANCE=nan"),
            (("--pressure",), "SET_PRESSURE_ADVANCE ADVANCE=.1"),
            ((), "SET_PRESSURE_ADVANCE ADVANCE=.1"),
        ):
            denied = subprocess.run([rust, cfg, *args], input=f"G90\n{command}\n",
                                    text=True, capture_output=True)
            assert denied.returncode and not denied.stdout and "line: 2" in denied.stderr, denied.stderr
        benchmark_shape = ("[input_shaper]\nshaper_freq_x: 56.6\nshaper_type_y: ei\n"
                           "shaper_freq_y: 48.4\n") if a.shape_timing else ""
        cfg.write_text(CONFIG.format(kinematics="corexy") + benchmark_shape)
        reset = "G90\nM82\nM221 S100\nG1 X1 Y1 Z.2 F2400\nG92 E0\n"
        script = "\n".join([reset + WITH_E] * 12)
        labels = ["stock", *executables]
        programs = {label: [] for label in labels}
        for iteration in range(a.rounds + 1):
            values = {}
            offset = iteration % len(labels)
            for label in labels[offset:] + labels[:offset]:
                if label == "stock":
                    trace, estimated, ns = traced(cfg, c, script, True, True)
                else:
                    run = subprocess.run([executables[label], cfg, "--pressure"], input=script,
                                         text=True, capture_output=True)
                    if run.returncode:
                        raise RuntimeError(run.stderr)
                    trace = run.stdout.splitlines()
                    estimated = float(trace.pop().split()[1])
                    ns = int(run.stderr.split()[1])
                values[label] = (trace, estimated)
                if iteration:  # First parity check warms the reference paths.
                    programs[label].append(ns)
            for label in executables:
                want, want_duration = values["stock"]
                got, got_duration = values[label]
                for i, (x, y) in enumerate(zip(want, got, strict=True)):
                    assert x == y, ("repeated CoreXY", iteration, label, i, x, y)
                assert math.isclose(want_duration, got_duration, abs_tol=1e-12, rel_tol=1e-12)
        count = len(values["rust"][0]) - 4
        print(f"Repeated CoreXY (shaped={a.shape_timing}): {count} exact pressure-advanced commands; "
              f"{values['rust'][1]:.9f} simulated seconds")
        if a.rounds:
            for kind, series in programs.items():
                print(f"{kind}: {statistics.median(series)/1e6:.3f} ms median CPU; "
                      f"{[round(v/1e6, 3) for v in series]}")


if __name__ == "__main__":
    main()
