#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare Rust heater commands and waits with actual Klippy handlers and inert observations."""
import argparse
from pathlib import Path
import random
import subprocess
import sys
from threading import Lock
from types import SimpleNamespace

from planner import PIN, ROOT


class CommandError(Exception):
    pass


def cases():
    rng = random.Random(62397)
    rows = [
        "M104 - 180", "M109 1 210 30 207 208", "select 1",
        "M109 - 200 199", "M109 - 0", "M104 5 0", "M104 5 -10", "M104 5 195",
        "M140 75", "M190 83 50 70 81", "M190 0", "M140 131", "set chamber 35",
        "set chamber 91", "set missing 30", "range ambient 20 25 18 26 20",
        "range ambient 30 - 25 29 30", "range ambient - 30 33 30",
        "range heater_bed 30 40 28 42 40", "range missing 30 - 30",
        "range ambient - -", "range ambient 40 40", "off", "select 4",
    ]
    for _ in range(900):
        kind = rng.randrange(10)
        if kind < 2:
            rows.append(f"select {rng.randrange(4)}")
        elif kind == 2:
            rows.append("off")
        elif kind < 5:
            n = rng.choice(["-", "0", "1", "9"])
            target = rng.choice([-1, 0, 120, 182, 280, 281, 300, 301])
            rows.append(f"M109 {n} {target} {target-4} {target-3} {target-2}")
        elif kind < 7:
            target = rng.choice([-1, 0, 60, 99, 130, 131])
            rows.append(f"M190 {target} {target-4} {target-3} {target-2}")
        elif kind == 7:
            rows.append(f"set chamber {rng.choice([0, 20, 90, 91])}")
        else:
            low = rng.randint(-20, 35)
            rows.append(f"range ambient {low} {low+4} {low-1} {low+5} {low+4}")
    return rows


def stock(path, rows, debug):
    sys.path.insert(0, str(path / "klippy"))
    from extras import heater_bed, heaters
    from gcode import CommandError as KlippyError, GCodeCommand
    from kinematics.extruder import PrinterExtruder

    fake_gcode = SimpleNamespace(error=CommandError, respond_raw=lambda _: None, respond_info=lambda _: None)
    active, pauses, samples = "extruder", 0, []

    def pause(at):
        nonlocal pauses
        pauses += 1
        return at

    reactor = SimpleNamespace(monotonic=lambda: 0., pause=pause)
    head = SimpleNamespace(register_lookahead_callback=lambda _: None,
                           get_extruder=lambda: extruders[active],
                           get_last_move_time=lambda: 0.)
    no_default = object()

    def lookup(name, default=no_default):
        if name in objects:
            return objects[name]
        if default is not no_default:
            return default
        raise CommandError(name)

    printer = SimpleNamespace(lookup_object=lookup, get_reactor=lambda: reactor,
                              is_shutdown=lambda: False, command_error=CommandError,
                              get_start_args=lambda: {"debugoutput": "/unused"} if debug else {})

    settings = {"extruder": 300., "extruder1": 280., "heater_bed": 130., "chamber": 90.}
    catalog = {}
    for name, maximum in settings.items():
        heater = heaters.Heater.__new__(heaters.Heater)
        heater.lock, heater.printer = Lock(), printer
        heater.target_temp, heater.min_temp, heater.max_temp = 0., 0., maximum
        controller = heaters.ControlBangBang.__new__(heaters.ControlBangBang)
        controller.max_delta = 2.
        heater.check_busy = lambda at, h=heater, c=controller: c.check_busy(
            at, samples[int(at)], h.target_temp)
        heater.get_temp = lambda at, h=heater: (samples[int(at)], h.target_temp)
        catalog[name] = heater
    extruders = {}
    for name in ["extruder", "extruder1"]:
        e = PrinterExtruder.__new__(PrinterExtruder)
        e.printer, e.heater = printer, catalog[name]
        extruders[name] = e
    bed = heater_bed.PrinterHeaterBed.__new__(heater_bed.PrinterHeaterBed)
    bed.printer, bed.heater = printer, catalog["heater_bed"]
    registry = heaters.PrinterHeaters.__new__(heaters.PrinterHeaters)
    registry.printer, registry.heaters = printer, catalog
    registry._get_temp = lambda _: ""
    objects = {"gcode": fake_gcode, "toolhead": head, "heaters": registry,
               "ambient": SimpleNamespace(get_temp=lambda at: (samples[int(at)], 0.)),
               **extruders}

    def run(row):
        nonlocal active, pauses, samples
        opcode, *parts = row.split()
        params = {}
        pauses = 0
        try:
            if opcode == "select":
                n, = parts
                name = "extruder" if n == "0" else "extruder" + n
                if name not in extruders:
                    raise CommandError(name)
                active = name
                return True
            if opcode in {"M104", "M109"}:
                id_, temp, *values = parts
                params["S"] = temp
                if id_ != "-":
                    params["T"] = id_
                handler = getattr(extruders["extruder"], f"cmd_{opcode}")
            elif opcode in {"M140", "M190"}:
                temp, *values = parts
                params["S"] = temp
                handler = getattr(bed, f"cmd_{opcode}")
            elif opcode == "set":
                name, target, *values = parts
                if name not in catalog:
                    raise CommandError(name)
                params["TARGET"] = target
                handler = catalog[name].cmd_SET_HEATER_TEMPERATURE
            elif opcode == "off":
                values, handler = [], registry.cmd_TURN_OFF_HEATERS
            else:
                name, low, high, *values = parts
                if name not in catalog and name != "ambient":
                    raise CommandError(name)  # G-code mux rejects an unregistered SENSOR.
                params["SENSOR"] = name
                if low != "-":
                    params["MINIMUM"] = low
                if high != "-":
                    params["MAXIMUM"] = high
                handler = registry.cmd_TEMPERATURE_WAIT
            samples = list(map(float, values))
            handler(GCodeCommand(fake_gcode, opcode, row, params, False))
        except (CommandError, KlippyError):
            return False
        return True

    for row in rows:
        success = run(row)
        yield ("ok" if success else "error") + f" {pauses}" + "".join(
            f" {heater.target_temp:.1f}" for heater in catalog.values())


def gcode_cases(rows):
    for row in rows:
        command, *args = row.split()
        if command == "select":
            yield row
            continue
        samples = []
        if command in {"M104","M109"}:
            tool, target, *samples = args
            line = f"{command} S{target}" + (f" T{tool}" if tool != "-" else "")
        elif command in {"M140","M190"}:
            target, *samples = args
            line = f"{command} S{target}"
        elif command == "set":
            name, target = args
            line = f"SET_HEATER_TEMPERATURE HEATER='{name}' TARGET={target}"
        elif command == "off":
            line = "TURN_OFF_HEATERS"
        else:
            name, low, high, *samples = args
            line = f'TEMPERATURE_WAIT SENSOR="{name}"'
            if low != "-":
                line += f" MINIMUM={low}"
            if high != "-":
                line += f" MAXIMUM={high}"
        yield f'{line} ; inert | {" ".join(samples)}'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT / "target/release/examples/thermal_command_replay")
    a = p.parse_args()
    rows = cases()
    for debug in [False, True]:
        expected = list(stock(a.upstream.resolve(), rows, debug))
        for text in [False, True]:
            flags = (["debug"] if debug else []) + (["--gcode"] if text else [])
            payload = list(gcode_cases(rows)) if text else rows
            got = subprocess.run([a.binary, *flags], text=True,
                                 input="\n".join(payload) + "\n", capture_output=True, check=True)
            for row, x, y in zip(payload, got.stdout.splitlines(), expected, strict=True):
                assert x == y, (debug, text, row, x, y)
    print(f"{2*len(rows)} matching heater command outcomes, target snapshots and wait poll counts "
          "for typed inputs and independently parsed G-code")



if __name__ == "__main__":
    main()
