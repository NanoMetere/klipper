#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare an A-B-A shaped dual-extruder path against stock Python plus C."""
import argparse
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace

from gcode_preview import stock
from planner import PIN, ROOT


def reference(upstream, executable, sparse):
    sys.path.insert(0, str(upstream / "klippy"))
    from kinematics.corexy import CoreXYKinematics
    from kinematics.extruder import PrinterExtruder
    from extras import shaper_defs
    cfg = CoreXYKinematics, [(0.,200.),(0.,180.),(0.,150.)], 180., 3100.,5.,.5,9.,160., (1.,1.,.2)
    e = object.__new__(PrinterExtruder)
    e.heater = SimpleNamespace(can_extrude=True)
    e.instant_corner_v = 1.0
    e.nozzle_diameter = .4
    e.filament_area = math.pi * (.5*1.75)**2
    e.max_extrude_ratio = .64/e.filament_area
    e.max_e_dist, e.max_e_velocity, e.max_e_accel = 15.,20.,300.
    e.trapq = None
    start = (0., 1.3, 2.1) if sparse else (0., 1.3)
    e_rows = [[] for _ in start]
    xyz_rows = []
    clock = 2.
    active = 0
    positions = list(start)
    e.trapq_append = lambda _, *fields: e_rows[active].append(
        ("e", *(int(v) if isinstance(v, bool) else v for v in fields)))

    def out(moves, dwell):
        nonlocal clock
        for m in moves:
            cruise = 0. if -1e-12 <= m.cruise_t < 0. else m.cruise_t
            m.cruise_t = cruise
            if m.is_kinematic_move:
                xyz_rows.append(("m", clock, m.accel_t, cruise, m.decel_t,
                                 *m.start_pos[:3], *m.axes_r[:3], m.start_v, m.cruise_v, m.accel))
            if m.axes_d[3]:
                e.process_move(clock, m, 3)
            clock += m.accel_t
            clock += cruise
            clock += m.decel_t
        clock += dwell

    def switch(line, coords, head):
        nonlocal active
        ident = int(line.split()[1])
        positions[active] = head.commanded_pos[3]
        head.commanded_pos[3] = positions[ident]
        coords._handle_activate_extruder()
        active = ident

    source = []
    order = [(0,"a"),(1,"b"),(2,"c"),(1,"d")] if sparse else [(0,"a"),(1,"b"),(0,"c")]
    for ident, name in order:
        source.extend(["M400", f"!use {ident}",
                       *(ROOT/f"examples/preview/multitool-{name}.gcode").read_text().splitlines()])
    stock(source, False, settings=cfg, emit_profiles=out, simulated_extruder=e, extend_directive=switch)
    focus = [shaper_defs.get_mzv_shaper(56.6, .1), shaper_defs.get_ei_shaper(48.4, .1)]
    for axis, (amplitudes, ticks) in zip("xy", focus, strict=True):
        xyz_rows.append(("h", axis, len(ticks),
                         *(v for pair in zip(ticks, amplitudes, strict=True) for v in pair)))
    for axis, step in zip(("+","-","z"), (.0125,.0125,.0025), strict=True):
        xyz_rows.append(("s", axis, 1.,1.,.2, step, clock+.51,20e6,20,0))
    for rows, start_e in zip(e_rows, start, strict=True):
        rows.extend([("g",0.0,.04,.04), ("s","e",start_e,0.,0.,.004,clock+.08,20e6,20,0)])

    def encoded(rows):
        payload = chr(10).join(" ".join(map(str, row)) for row in rows)+chr(10)
        return subprocess.run([executable], input=payload, text=True, capture_output=True, check=True).stdout.splitlines()
    return [*encoded(xyz_rows), *(line for rows in e_rows for line in encoded(rows))], clock-2.


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/tool_motion_replay")
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



if __name__ == "__main__":
    main()
