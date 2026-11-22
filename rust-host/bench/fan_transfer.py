#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare actual toolchanger fan handoffs with a mock fan command sink."""
import argparse
import importlib
import math
from pathlib import Path
import random
import shlex
import subprocess
import sys
from types import SimpleNamespace

from planner import ROOT
from toolchange_order import TOOLCHANGER


def corpus(klipper, plugin):
    sys.path.insert(0, str(klipper.resolve()/"klippy"))
    import extras
    extras.__path__.insert(0, str(plugin.resolve()/"klipper/extras"))
    stock = importlib.import_module("extras.toolchanger")
    rng = random.Random(56898)
    commands, results = [], []
    for round_number in range(180):
        controller = stock.FanSwitcher.__new__(stock.FanSwitcher)
        controller.has_multi_fan = False
        controller.transfer_fan_speed = bool(round_number % 3)
        controller.active_fan = None
        controller.pending_speed = None
        text = lambda v: "-" if v is None else str(v)
        powers = [0.0]*3

        def run(command):
            directive, name, value = shlex.split(command)
            assert directive == "SET_FAN_SPEED" and name.startswith("FAN=test")
            fan_id = int(name[-1])
            powers[fan_id] = float(value.split("=")[1])
            results.append(("I", fan_id, powers[fan_id]))

        controller.gcode = SimpleNamespace(run_script_from_command=run)
        objects = {i: SimpleNamespace(fan_name=f"test{i}", ident=i,
                                       get_status=lambda _, i=i: {"speed":powers[i]}) for i in range(3)}
        commands.append(f"new {int(controller.transfer_fan_speed)}")
        for _ in range(65):
            idx = rng.choice([None, 0, 1, 2])
            obj = objects.get(idx)
            if rng.random() < .54:
                want = rng.choice([0.0, .10, .60, .79, 1.0, 2.5])
                commands.append(f"set {text(idx)} {want}")
                controller.set_speed(want, SimpleNamespace(fan=obj))
            else:
                old = None if controller.active_fan is None else controller.active_fan.get_status(0)["speed"]
                commands.append(f"activate {text(idx)} {text(old)}")
                controller.activate_fan(obj)
            active = None if controller.active_fan is None else controller.active_fan.ident
            results.append(("S", active, controller.pending_speed))
    return commands, results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("legacy_klipper", type=Path)
    p.add_argument("toolchanger", type=Path)
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/fan_transfer_replay")
    a = p.parse_args()
    if subprocess.check_output(["git","-C",a.toolchanger,"rev-parse","HEAD"],text=True).strip() != TOOLCHANGER:
        p.error("wrong pinned toolchanger")
    commands, want = corpus(a.legacy_klipper, a.toolchanger)
    run = subprocess.check_output([a.binary], input=chr(10).join(commands)+chr(10), text=True)
    for i, (line, record) in enumerate(zip(run.splitlines(), want, strict=True)):
        label, a, b = line.split()
        assert label == record[0]
        assert (None if a == "-" else int(a)) == record[1]
        if b == "-":
            assert record[2] is None
        else:
            assert math.isclose(float(b), record[2], rel_tol=1e-12, abs_tol=1e-12), (i, line, record)
    print(f"{sum(x[0] == 'S' for x in want)} state reports and {sum(x[0] == 'I' for x in want)} fan intents match")


if __name__ == "__main__":
    main()
