#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify the bridge in a stock Klippy file input / debug output process."""
import argparse
import difflib
from collections import defaultdict
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from planner import PIN, ROOT

CFG = """[mcu]
serial: /dev/unused
[printer]
kinematics: corexy
max_velocity: 250
max_accel: 4000
max_z_velocity: 15
max_z_accel: 200
[stepper_x]
step_pin: gpio1
dir_pin: gpio2
endstop_pin: gpio3
microsteps: 16
rotation_distance: 40
position_endstop: 0
position_max: 300
[stepper_y]
step_pin: gpio4
dir_pin: gpio5
endstop_pin: gpio6
microsteps: 16
rotation_distance: 40
position_endstop: 0
position_max: 300
[stepper_z]
step_pin: gpio7
dir_pin: gpio8
endstop_pin: gpio9
position_endstop: 0
position_max: 300
microsteps: 16
rotation_distance: 8
[force_move]
enable_force_move: True
"""


def gcode():
    yield "SET_KINEMATIC_POSITION X=10 Y=10 Z=1"
    yield "G90"
    for j in range(1200):
        x = 10 + (j % 100) * .12
        y = 15 + (j % 46) * .2
        z = 1 + (j // 50) * .05
        yield f"G1 X{x:.5f} Y{y:.5f} Z{z:.5f} F{1800 + (j % 6) * 700}"
        if j % 80 == 79:
            yield "M400"
    yield "M400"


def motor_trace(upstream, dictionary, data):
    sys.path.insert(0, str(upstream/"klippy"))
    from msgproto import MessageParser
    parser = MessageParser()
    parser.process_identify(dictionary.read_bytes(), decompress=False)
    rows = defaultdict(list)
    offset = 0
    while offset < len(data):
        length = parser.check_packet(data[offset:offset+64])
        assert length > 0, (offset, length)
        for command in parser.dump(data[offset:offset+length])[1:]:
            split = command.split()
            if split[0] in {"reset_step_clock", "queue_step", "set_next_step_dir"}:
                rows[split[1]].append(command)
        offset += length
    return dict(rows)


def compare_motor_traces(upstream, dictionary, data):
    stock, hybrid = [motor_trace(upstream, dictionary, d) for d in data]
    if stock != hybrid:
        differing = next(k for k in stock.keys() | hybrid.keys() if stock.get(k) != hybrid.get(k))
        wanted, actual = stock.get(differing, []), hybrid.get(differing, [])
        delta = list(difflib.unified_diff(wanted, actual, n=3))
        raise AssertionError((differing, len(wanted), len(actual), delta[:40]))
    return sum(map(len, stock.values()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=lambda s: Path(s).resolve(), help=f"stock checkout at {PIN}")
    p.add_argument("dictionary", type=lambda s: Path(s).resolve())
    p.add_argument("library", type=lambda s: Path(s).resolve())
    a = p.parse_args()
    klippy = a.upstream/"klippy/klippy.py"
    with tempfile.TemporaryDirectory(prefix="rust-motion-bridge-") as d:
        root = Path(d)
        cfg, script = root/"printer.cfg", root/"input.gcode"
        cfg.write_text(CFG)
        script.write_text("\n".join(gcode())+"\n")
        results = []
        for label in ("stock", "hybrid"):
            output, log = root/f"{label}.out", root/f"{label}.log"
            cmd = [sys.executable]
            if label == "hybrid":
                cmd.extend([ROOT/"bench/hybrid_klippy_entry.py", klippy, a.library])
            else:
                cmd.append(klippy)
            cmd.extend([cfg, "-i", script, "-o", output, "-d", a.dictionary, "-l", log])
            run = subprocess.run(cmd, text=True, capture_output=True, timeout=60)
            if run.returncode:
                raise RuntimeError(run.stdout+run.stderr+log.read_text()[-5000:])
            data = output.read_bytes()
            assert len(data) > 100
            results.append(data)
            if label == "hybrid":
                row = next(l for l in run.stdout.splitlines() if l.startswith("RUST_LOOKAHEAD_STATS "))
                stats = json.loads(row.split(" ", 1)[1])
                assert stats["planned"] > 1000 and stats["fallbacks"] == 0, stats
                print(f"Hybrid real Klippy debug mode: {stats}")
        # Separate the wire batching from the motion output. Klippy's C
        # worker batches frames asynchronously even in debug-output mode.
        commands = compare_motor_traces(a.upstream, a.dictionary, results)
        print(f"{commands} decoded stock and hybrid motor commands matched by OID")


if __name__ == "__main__":
    main()
