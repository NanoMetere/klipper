#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare print health accounting with the actual stock PrintStats class."""
import argparse
import json
import math
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

from planner import PIN, ROOT


def vectors(upstream):
    sys.path.insert(0, str(upstream.resolve() / "klippy"))
    from extras.print_stats import PrintStats
    rng = random.Random(6156)
    rows, want = [], []
    names = ["repeat.gcode", "tool-swap.gcode", "utf8-한글.gcode"]
    for i in range(260):
        stock = PrintStats.__new__(PrintStats)
        stock.reset()
        name = names[i % len(names)]
        stock.set_current_file(name)
        rows.append(f"file {name}")
        now, position, flow = 10., 0., 1.
        stock.reactor = SimpleNamespace(monotonic=lambda: now)
        stock.gcode_move = SimpleNamespace(get_status=lambda at=None:
            {"position": SimpleNamespace(e=position), "extrude_factor": flow})
        functions = {
            "start": stock.note_start,
            "pause": stock.note_pause,
            "switch": stock._handle_activate_extruder,
            "cancel": stock.note_cancel,
            "error": lambda: stock.note_error("fixture_error"),
            "finish": stock.note_complete,
        }
        def event(op):
            rows.append(f"{op} {now} {position} {flow}")
            if op == "status":
                want.append(stock.get_status(now))
            else:
                functions[op]()

        event("status")
        event("start")
        paused = False
        for slot in range(54):
            now += rng.uniform(.02, 1.0)
            if slot % 11 == 3:
                args = {}
                if rng.random() < .65:
                    args["TOTAL_LAYER"] = rng.randint(0, 200)
                if rng.random() < .8:
                    args["CURRENT_LAYER"] = rng.randint(0, 250)
                stock.cmd_SET_PRINT_STATS_INFO(SimpleNamespace(
                    get_int=lambda key, default, minval: args.get(key, default)))
                rows.append(f"layers {args.get('TOTAL_LAYER','-')} {args.get('CURRENT_LAYER','-')}")
            if slot % 13 == 2:
                event("start" if paused else "pause")
                paused = not paused
            elif slot % 13 == 7:
                position = rng.uniform(-10, 30)
                event("switch")
            else:
                position += rng.uniform(-1, 2.7) if i % 7 else 0.
                flow = rng.choice([.75, .95, 1., 1.1, 1.3])
                event("status")
        now += 1.
        event(["finish", "error", "cancel"][i % 3])
        event("status")
    return rows, want


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/print_stats_replay")
    a = p.parse_args()
    rows, expected = vectors(a.upstream)
    out = subprocess.check_output([a.binary], text=True, input="\n".join(rows)+"\n")
    for i, (want, actual) in enumerate(zip(expected, map(json.loads, out.splitlines()), strict=True)):
        assert want.keys() == actual.keys()
        for k, v in want.items():
            if isinstance(v, float):
                assert math.isclose(v, actual[k], rel_tol=1e-12, abs_tol=1e-12), (i, k, v, actual[k])
            else:
                assert v == actual[k], (i, k, v, actual[k])
    print(f"{len(expected)} print status snapshots across 260 jobs match stock")


if __name__ == "__main__":
    main()
