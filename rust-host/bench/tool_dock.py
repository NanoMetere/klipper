#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify coordinate restoration using the actual Toolchanger methods offline."""
import argparse
import importlib
import math
from pathlib import Path
import random
import subprocess
import sys
from contextlib import redirect_stdout
from io import StringIO
from types import SimpleNamespace

from gcode_preview import stock
from planner import ROOT
from toolchange_order import TOOLCHANGER


def run(lines):
    text = StringIO()
    with redirect_stdout(text):
        stock(lines, True)
    state = [l.split()[1:] for l in text.getvalue().splitlines() if l.startswith("state ")][-1]
    return list(map(float, state))


def fixture(module, old, new, start, target, mask, lanes=False):
    vec = lambda values: " ".join(f"{v:.13f}" for v in values)
    cmd = lambda fields, fmt: " ".join(f"{axis}{v:{fmt}}" for axis, v in fields)
    init = [
        f"!wrap 1 {vec(old)} 1", "!reset",
        "G1 " + cmd(zip("XYZ", start, strict=True), ".13f") + " E3 F1500",
        "SET_GCODE_OFFSET X=.3 Y=-.1 Z=.12", "M221 S110", "M400",
    ]
    state = run(init)
    restore_axes = "".join(axis for idx, axis in enumerate("XYZ") if mask & (1 << idx))
    tc = module.Toolchanger.__new__(module.Toolchanger)
    items = []
    tc.gcode = SimpleNamespace(
        run_script_from_command=items.append,
        create_gcode_command=lambda _, __, params: "G0 " + cmd(params.items(), ".17f"),
    )
    tc.gcode_move = SimpleNamespace(
        reset_last_position=lambda: items.append("!reset"),
        get_status=lambda: {"gcode_position":state[4:8], "homing_origin":state[8:12]},
        cmd_G1=items.append,
    )
    class Transform:
        @property
        def tool(self):
            return None
        @tool.setter
        def tool(self, tool):
            items.append("!set 1 " + vec(tool.offset if tool is not None else (0.,0.,0.)))
    tc.gcode_transform = Transform()
    next_tool = SimpleNamespace(offset=new)
    tc._save_state(restore_axes, next_tool)
    tc._set_toolchange_transform()
    items += ["G90", "G0 "+cmd(zip("XYZ", target, strict=True), ".13f"), "G91", "M83",
              "G1 E-.3 F600", "M400"]
    if lanes:
        tc.active_tool = SimpleNamespace(deactivate=lambda: None)
        tc.tool_probe_endstop = None
        next_tool.activate = lambda: items.extend(["M400", "!use"])
        tc._configure_toolhead_for_tool(next_tool)
        items += ["G91", "M83", "G1 E.1 F600", "G1 E.2 F600"]
    tc._restore_state_and_transform(next_tool)
    items += ["M400", *(["!end_groups"] if lanes else []), "G1 E3.1", "M400"]
    if not lanes:
        return run(init+items)
    active = 0
    old = None
    collect = True
    totals = [0.,0.]

    def visit(line, coords, head):
        nonlocal active, old, collect
        if line == "!use":
            old = head.commanded_pos[3]
            head.commanded_pos[3] = 1.3
            coords._handle_activate_extruder()
            active = 1
        else:
            assert line == "!end_groups"
            collect = False

    def profiles(moves, _):
        if collect:
            for m in moves:
                totals[active] += m.axes_d[3]
    sink = StringIO()
    with redirect_stdout(sink):
        stock(init+items, True, emit_profiles=profiles, extend_directive=visit)
    final = [l.split()[1:] for l in sink.getvalue().splitlines() if l.startswith("state ")][-1]
    final = list(map(float, final))
    return [*final, old, final[15], *totals]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("legacy_klipper", type=Path)
    p.add_argument("toolchanger", type=Path)
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/tool_dock_replay")
    a = p.parse_args()
    rev = subprocess.check_output(["git","-C",a.toolchanger,"rev-parse","HEAD"], text=True).strip()
    if rev != TOOLCHANGER:
        p.error("wrong pinned toolchanger")
    sys.path.insert(0, str(a.legacy_klipper.resolve()/"klippy"))
    import extras
    extras.__path__.insert(0, str(a.toolchanger.resolve()/"klipper/extras"))
    module = importlib.import_module("extras.toolchanger")
    rng = random.Random(1150)
    wanted, wanted_lanes, script = [], [], []
    for i in range(240):
        old = [rng.randint(-150, 150) / 100. for _ in "xyz"]
        new = [rng.randint(-150, 150) / 100. for _ in "xyz"]
        start = [rng.randint(300, 1500) / 100. for _ in "xyz"]
        target = [rng.randint(2000, 5000) / 100. for _ in "xyz"]
        mask = i % 8
        wanted.append(fixture(module, old, new, start, target, mask))
        wanted_lanes.append(fixture(module, old, new, start, target, mask, True))
        script.append(" ".join(map(str, [mask,*old,*new,*start,*target])))
    data = "\n".join(script)+"\n"
    for args, expected in [([],wanted),(["--lanes"],wanted_lanes)]:
        actual = subprocess.check_output([a.binary,*args], text=True, input=data)
        for i, (w, got) in enumerate(zip(expected, actual.splitlines(), strict=True)):
            for want, value in zip(w, map(float, got.split()), strict=True):
                assert math.isclose(want, value, abs_tol=3e-12, rel_tol=3e-12), (args, i, w, got)
        print(f"{len(expected)} stock dock/restore cases; {len(expected[0])*len(expected)} "
              f"observations {'including sparse extruder lane attribution' if args else 'without lane attribution'}")



if __name__ == "__main__":
    main()
