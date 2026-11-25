#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare virtual display status against pinned Klippy and prove motion isolation."""
import argparse
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace

from planner import PIN, ROOT


def fixture():
    yield from (
        (0, False, None, None),
        (.2, False, .01, "m73p-8 R30"),
        (1, False, .15, "M73 P140"),
        (6, False, .20, None),  # Exact expiration remains current.
        (6.001, True, .25, None),
        (6.002, True, .30, "M73 R20 C3"),
        (6.003, False, .35, "M117  30% 출력 ; still raw"),
        (7, False, .36, "M117Attached"),
        (8, False, .39, " m117\tLayer 2 "),
        (9, False, None, "M117"),
        (10, True, .40, "M73 P"),
        (11, True, .40, "M73 P-.0"),
        (18, True, .50, None),
        (19, False, .55, None),
    )
    rng = random.Random(11329)
    for i in range(280):
        cmd = rng.choice([None, "M117 안녕 ; layer", "M117", "M73 R2",
                          f"M73 P{rng.uniform(-75, 175):.5f} R4"])
        yield (20.0 + 1.75*i, bool(rng.randrange(2)), rng.random(), cmd)


def stock(upstream, rows):
    sys.path.insert(0, str(upstream / "klippy"))
    from extras.display_status import DisplayStatus
    from gcode import GCodeDispatch, CommandError
    now, printing, sd = 0., False, None
    dispatch = object.__new__(GCodeDispatch)
    dispatch.gcode_handlers = {}
    dispatch.register_command = lambda name, command, **_: dispatch.gcode_handlers.update({name: command})
    dispatch.error = CommandError
    dispatch.is_printer_ready = True
    dispatch._respond_error = lambda _: None
    services = {
        "gcode": dispatch,
        "idle_timeout": SimpleNamespace(get_status=lambda _: {"state": "Printing" if printing else "Idle"}),
    }
    def lookup(name, default=None):
        if name == "virtual_sdcard" and sd is not None:
            return SimpleNamespace(get_status=lambda _: {"progress": sd})
        return services.get(name, default)
    printer = SimpleNamespace(lookup_object=lookup, send_event=lambda *_: None,
                              get_reactor=lambda: SimpleNamespace(monotonic=lambda: now))
    dispatch.printer = printer
    display = DisplayStatus(SimpleNamespace(get_printer=lambda: printer))
    for now, printing, sd, command in rows:
        try:
            if command is not None:
                dispatch._process_commands([command], need_ack=False)
        except CommandError:
            yield None
        else:
            yield display.get_status(now)


def motion(cli):
    config = ROOT / "examples" / "preview"
    simple = ["G90", "M83", "G1 X6 Y5 E.15 F1400", "G1 X9 Y6 E.12",
              "G1 X12 Y6 E.18", "G4 P65", "G1 X14 Y8 E.14", "M400"]
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "job.gcode"
        command = [cli, "--tool-temps", "0:210", "--fixed-pressure-script",
                   config, "single-pressure.cfg", path, "20000000", "1", "1", ".2",
                   "printer.cfg", "rails.cfg"]
        def run(lines):
            path.write_text("\n".join(lines) + "\n")
            return subprocess.run(command, capture_output=True, check=True).stdout
        control = run(simple)
        mixed = run([*simple[:3], "M117 G1 X190 E20 ; do not move",
                     "M73 P40 R2", *simple[3:4], "M117 Layer 3",
                     *simple[4:], "M73 P100", "M117"])
        assert mixed == control, "display lines alter the timed motion trace"
        result = json.loads(mixed)
        return sum(map(len, result["motors"])) + sum(len(l["commands"]) for l in result["extruder_lanes"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path)
    p.add_argument("binary", type=Path)
    p.add_argument("cli", type=Path)
    a = p.parse_args()
    upstream = a.upstream.resolve()
    assert subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip() == PIN
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    rows = list(fixture())
    data = "\n".join(map(json.dumps, rows)) + "\n"
    actual = subprocess.check_output([a.binary.resolve()], input=data, text=True)
    for case, got, expected in zip(rows, map(json.loads, actual.splitlines()), stock(upstream, rows), strict=True):
        if expected is None:
            assert got is None, (case, got, expected)
        else:
            assert got.keys() == expected.keys() and got["message"] == expected["message"], (case, got, expected)
            assert math.isclose(got["progress"], expected["progress"], rel_tol=0., abs_tol=2e-16), (case, got, expected)
    print(f"{len(rows)} actual stock display observations; {motion(a.cli.resolve())} unchanged timed motor records")


if __name__ == "__main__":
    main()
