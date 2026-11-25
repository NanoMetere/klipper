#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare the optional Rust scanner with the pinned Klippy dispatcher."""
import argparse
import logging
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

from planner import PIN, ROOT


ODD = [
    'G0 X+1 Y-2', 'G1 X.123 Y-.2', 'G1 E--1', 'G1 F.1 Z0 X-0',
    'G1 X1 X2', 'G1 Z.1 E1E-4', 'G1 E1e-4', 'G1X2Y3', 'G1 X5  ',
    '  G1 X2', 'G1 X2 ;comment', 'G1\tX2', 'G1 V2', 'G1', 'G0 X2\nG1 Y3',
    'G1 Y3\x00', 'G1 X2 ;😸', 'G1 X2;日本語\t', 'G1 X2\u00a0;注释',
    'G01 X12', ' G1 Z4\r', 'N72 G1 F30*12',
    'SET_GCODE_OFFSET X=2', 'M400', '', ';ignored',
]


def simulate(gcode, method, cmdlist, ack, raises):
    events = []

    def record(c):
        events.append((c.get_command(), c.get_commandline(),
                       tuple(c.get_command_parameters().items()),
                       c.get_raw_command_parameters()))
        if raises and c.get_command_parameters().get("X") == "77":
            raise gcode.CommandError('fixture error')
        if raises and c.get_command_parameters().get("X") == "78":
            raise ValueError('fixture unexpected')

    def put(*parts):
        events.append(parts)

    mock = SimpleNamespace(
        args_r=gcode.GCodeDispatch.args_r, gcode_handlers={"G0": record, "G1": record},
        cmd_default=record, error=gcode.CommandError, _respond_error=put,
        respond_info=put, respond_raw=put,
        printer=SimpleNamespace(send_event=put, invoke_shutdown=put),
    )
    try:
        method(mock, cmdlist, ack)
    except Exception as e:
        events.append((type(e).__name__,str(e)))
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stock", type=Path)
    parser.add_argument("library", type=Path)
    args = parser.parse_args()
    pin = subprocess.check_output(["git", "-C", args.stock, "rev-parse", "HEAD"], text=True).strip()
    if pin != PIN:
        parser.error(f"stock checkout must be {PIN}")
    subprocess.run(["git", "-C", args.stock, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path[:0] = [str(args.stock.resolve() / "klippy"), str(ROOT / "python")]
    import gcode
    from klipper_rust_dispatch import install

    stock = gcode.GCodeDispatch._process_commands
    counts = install(gcode, args.library.resolve())
    candidate = gcode.GCodeDispatch._process_commands
    logging.disable(logging.CRITICAL)
    rng = random.Random(512)
    for i in range(600):
        lines = []
        for _ in range(30):
            if rng.random() < .20:
                lines.append(rng.choice(ODD))
                continue
            keys = rng.sample("XYZEF", rng.randrange(1, 6))
            tokens = [
                (" " * rng.randint(1, 3)) + key
                + rng.choice([f"{rng.uniform(-7, 9):.4f}", "77", "78"])
                for key in keys
            ]
            lines.append(rng.choice(["G0", "G1"]) + " ".join(tokens))
        for ack in (False, True):
            for raises in (False, True):
                a = simulate(gcode, stock, lines, ack, raises)
                b = simulate(gcode, candidate, lines, ack, raises)
                assert a == b, (i, ack, raises, a, b)
    assert counts["canonical"] > 0 and counts["stock"] > 0, counts
    print("2,400 batches with 72,000 submitted lines; fallback and exception/ACK paths match:", counts)


if __name__ == "__main__":
    main()
