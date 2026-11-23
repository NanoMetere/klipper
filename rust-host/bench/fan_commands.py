#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Match the typed print-fan subset to stock G-code dispatch and handler."""
import argparse
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

from planner import PIN


def reference(upstream, lines):
    sys.path.insert(0, str(upstream / "klippy"))
    from extras.fan import PrinterFan
    from gcode import CommandError, GCodeDispatch

    seen = []
    fan = PrinterFan.__new__(PrinterFan)
    fan.fan = SimpleNamespace(set_speed_from_command=seen.append)
    dispatch = GCodeDispatch.__new__(GCodeDispatch)
    dispatch.gcode_handlers = {"M106": fan.cmd_M106, "M107": fan.cmd_M107}
    dispatch.error = CommandError
    dispatch.cmd_default = lambda _cmd: None
    dispatch._respond_error = lambda _: None
    dispatch.printer = SimpleNamespace(send_event=lambda _: None)

    for line in lines:
        seen.clear()
        try:
            dispatch._process_commands([line], need_ack=False)
        except CommandError:
            yield "rejected"
        else:
            yield float(seen[0]) if seen else "other"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path, help=f"stock at {PIN}")
    parser.add_argument("--binary", type=Path, required=True)
    args = parser.parse_args()
    upstream = args.upstream.resolve()
    actual = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    if actual != PIN:
        parser.error(f"stock checkout must be {PIN}")
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    rng = random.Random(6408)
    lines = ["M106", "M107", " m106s127.5 ; comment", "M106 S", "M106 S-.5", "M106.1 S123",
             "M104 S210", "SET_FAN_SPEED FAN=AUX SPEED=.3", "m106 s510", "M106 S+255", "M106 S.9"]
    for i in range(242):
        speed = rng.randrange(-50,7650)/10
        style = ["M106 S", "m106s", "M106\tS ", "   m106 S"][i%4]
        lines.append(f"{style}{speed:.3f} ; fixture")
    out = subprocess.check_output([args.binary.resolve()], input="\n".join(lines)+"\n", text=True)
    for source, row, want in zip(lines, out.splitlines(), reference(upstream, lines), strict=True):
        got = row if row in {"other", "rejected"} else float(row)
        assert got == want or isinstance(got,float) and isinstance(want,float) and abs(got-want)<1e-17, (source,got,want)
    print(f"{len(lines)} bounded print-fan commands matched stock handler outcomes")


if __name__ == "__main__":
    main()
