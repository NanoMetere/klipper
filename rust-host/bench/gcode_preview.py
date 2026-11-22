#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare a strict Rust offline G-code preview with the pinned Klippy code."""
import argparse
import math
import os
from pathlib import Path
import random
import statistics
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

from planner import PIN, ROOT


def fixture():
    yield from ("; clear", "G21", "M82", "g0x2Y2z.4F2400  ; inline",
                "G1X2Y2Z.4", "G92 X0 E0", "G1X0.00000000001 E.2", "G1X0",
                "M220 S50", "M221 S90", "G91", "G1 E-.15 F650", "G4 P20",
                "M400", "g90", "M220", "M221", "G92", "G1X0", "M83")
    r = random.Random(20261123)
    for i in range(7000):
        op = r.randrange(20)
        if op == 0:
            yield r.choice(["M82", "M83", "G90", "G91"])
        elif op == 1:
            yield r.choice(["G92", "G92 X0", "G92E0Y1"])
        elif op == 2:
            yield f"{r.choice(['M220','M221'])} S{r.randrange(70, 125)}"
        elif op == 3:
            yield "M400" if i % 2 else f"G4 P{r.randrange(1200)}"
        else:
            axes = r.sample("XYZEF", r.randrange(1, 6))
            values = {a: r.uniform(-0.35, 0.8) if a in "ZE" else r.uniform(-14, 14)
                      for a in "XYZE"}
            values["F"] = r.randrange(100, 13000)
            cmd = r.choice(["G0", "G1", "g1"])
            sep = r.choice(["", " ", "\t"])
            yield cmd + sep + sep.join(f"{a}{values[a]:.6f}" for a in axes)


def stock(lines, emit):
    from extras.gcode_move import GCodeMove
    from gcode import GCodeDispatch
    from kinematics.extruder import PrinterExtruder
    from toolhead import BUFFER_TIME_HIGH, LookAheadQueue, ToolHead, ToolHeadCommandHelper

    # Reuse real parser, handlers, ToolHead.move and queue methods. Stub only
    # equipment, kinematics/temperature checks and the delivery of profiles.
    dispatch = object.__new__(GCodeDispatch)
    dispatch.gcode_handlers = {"": lambda _: None}
    dispatch.output_callbacks = []
    dispatch.is_fileinput = False
    handlers = dispatch.gcode_handlers
    dispatch.register_command = lambda name, f, *args, **kwargs: handlers.update({name: f})
    printer = SimpleNamespace(lookup_object=lambda _: dispatch, register_event_handler=lambda *_: None)
    dispatch.printer = printer
    coords = GCodeMove(SimpleNamespace(get_printer=lambda: printer))

    axis = SimpleNamespace(instant_corner_v=1.0, check_move=lambda *_: None)
    axis.calc_junction = lambda a, b, i: PrinterExtruder.calc_junction(axis, a, b, i)
    kin = SimpleNamespace(check_move=lambda _: None)
    q = LookAheadQueue()
    q.set_flush_time(BUFFER_TIME_HIGH)
    head = SimpleNamespace(max_velocity=160., max_accel=2500., square_corner_velocity=5.,
                           min_cruise_ratio=.5, kin=kin, extra_axes=[axis],
                           commanded_pos=[0.]*4, lookahead=q, print_time=0., need_check_pause=1.)
    ToolHead._calc_junction_deviation(head)
    dispatch.toolhead = head
    moves = []
    dwell = None
    head._process_lookahead = lambda lazy=False: moves.extend(q.flush(lazy=lazy))
    head._flush_lookahead = lambda: ToolHead._flush_lookahead(head)
    head.get_last_move_time = lambda: head.print_time
    head._advance_move_time = lambda _: None
    head._check_pause = lambda: None

    def dwell_to(sec):
        nonlocal dwell
        dwell = sec
        ToolHead.dwell(head, sec)

    head.dwell = dwell_to
    head.wait_moves = head._flush_lookahead
    handlers["M400"] = lambda cmd: ToolHeadCommandHelper.cmd_M400(dispatch, cmd)
    handlers["G4"] = lambda cmd: ToolHeadCommandHelper.cmd_G4(dispatch, cmd)
    coords.move_with_transform = lambda p, v: ToolHead.move(head, p, v)

    total = 0.
    count = 0

    def log():
        nonlocal count, total
        count += len(moves)
        if emit:
            print("line", len(moves), -1. if dwell is None else dwell)
            print("state", *coords.last_position, *coords._get_gcode_position(), coords.speed)
        for m in moves:
            total += m.accel_t + m.cruise_t + m.decel_t
            if emit:
                print("move", int(m.is_kinematic_move), *m.start_pos, *m.end_pos, *m.axes_r,
                      m.start_v, m.cruise_v, m.end_v, m.accel, m.accel_t, m.cruise_t, m.decel_t)
        total += dwell or 0.
        moves.clear()

    for line in lines:
        dwell = None
        dispatch._process_commands([line], need_ack=False)
        log()
    dwell = None
    head._flush_lookahead()
    log()
    return count, total


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--rounds", default=7, type=int)
    args = p.parse_args()
    corpus = "\n".join(fixture()) + "\n"
    with tempfile.TemporaryDirectory() as dir_:
        tmp = Path(dir_)
        rust, lib = tmp / "preview", tmp / "libklipper_host_protocol.rlib"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/gcode_preview.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        env = os.environ.copy()
        env["PYTHONPATH"] = str(args.upstream.resolve() / "klippy") + os.pathsep + env.get("PYTHONPATH", "")
        commands = {"stock": [sys.executable, __file__, "--stock"], "rust": [rust]}

        def run(cmd):
            return subprocess.check_output(cmd, env=env, input=corpus, text=True).splitlines()

        want, got = (run(cmd) for cmd in commands.values())
        counts = {}
        max_abs = 0.
        for n, (expected, actual) in enumerate(zip(want, got, strict=True)):
            x, y = expected.split(), actual.split()
            assert x[0] == y[0], (n, expected, actual)
            counts[x[0]] = counts.get(x[0], 0) + 1
            if x[0] in ("line", "move"):
                assert x[1] == y[1], (n, expected, actual)
                x, y = x[1:], y[1:]
            for a, b in zip(map(float, x[1:]), map(float, y[1:]), strict=True):
                assert math.isclose(a, b, rel_tol=2e-12, abs_tol=2e-12), (n, expected, actual)
                max_abs = max(max_abs, abs(a-b))
        print(f"{counts}; maximum absolute difference {max_abs:g}")

        results = {kind: [] for kind in commands}
        total = None
        for i in range(args.rounds):
            for kind in tuple(commands)[::(-1 if i % 2 else 1)]:
                elapsed, count, duration = run([*commands[kind], "--bench"])[0].split()
                assert int(count) == counts["move"]
                duration = float(duration)
                if total is None:
                    total = duration
                assert math.isclose(duration, total, rel_tol=2e-12)
                results[kind].append(int(elapsed) / (counts["line"] - 1))
        for kind, times in results.items():
            print(f"{kind}: {statistics.median(times):.1f} ns/line median "
                  f"({min(times):.1f}–{max(times):.1f}); {times}")


if __name__ == "__main__":
    if "--stock" in sys.argv:
        lines = list(sys.stdin)
        if "--bench" in sys.argv:
            stock(lines, False)
            start = time.perf_counter_ns()
            count, total = stock(lines, False)
            print(time.perf_counter_ns() - start, count, total)
        else:
            stock(lines, True)
    else:
        main()
