#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline fan scheduling with stock Fan and GCodeRequestQueue."""
import argparse
import math
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

from planner import PIN, ROOT


def corpora(upstream):
    sys.path.insert(0, str(upstream.resolve() / "klippy"))
    from extras.fan import Fan
    from extras.output_pin import GCodeRequestQueue
    rng = random.Random(85313)
    for seed in range(240):
        source, target = [], []
        fan = Fan.__new__(Fan)
        fan.last_fan_value = fan.last_req_value = 0.
        fan.max_power = rng.choice([.3, .7, .93, 1.])
        fan.kick_start_time = rng.choice([0., .033, .1, .5])
        fan.off_below = rng.choice([0., .1, .25])
        fan.mcu_fan = SimpleNamespace(set_pwm=lambda *args: target.append(("P", *args)))
        fan.enable_pin = SimpleNamespace(set_digital=lambda *args: target.append(("E", *args))) if seed % 2 else None
        queue = GCodeRequestQueue.__new__(GCodeRequestQueue)
        minimum = rng.choice([.000015, .001, .01, .050])
        queue.mcu = SimpleNamespace(min_schedule_time=lambda: minimum)
        queue.motion_queuing = SimpleNamespace(note_mcu_movequeue_activity=lambda *a, **kw: None)
        queue.callback = fan._apply_speed
        queue.rqueue = []
        queue.next_min_flush_time = 0.
        source.append(f"new {fan.max_power} {fan.off_below} {fan.kick_start_time} {minimum} {int(bool(fan.enable_pin))}")
        now = 0.
        values = [0., .1, .28, .5, .55, .9, 1., 2., -.1]
        for _ in range(95):
            now += rng.choice([0., .0002, .02, .15, .251])
            kind = rng.choice(["queue","queue","queue","flush","flush","async"])
            value = rng.choice(values)
            if kind == "queue":
                queue._queue_request(now, value)
            elif kind == "async":
                queue.send_async_request(value, now)
            else:
                queue._flush_notification(now, now)
            source.append(f"{kind} {now}" + ("" if kind == "flush" else f" {value}"))
            target.append(("S", fan.last_req_value, queue.next_min_flush_time, len(queue.rqueue)))
        yield source, target


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/fan_replay")
    a = p.parse_args()
    rows, expect = [], []
    for source, target in corpora(a.upstream):
        rows.extend(source)
        expect.extend(target)
    actual = subprocess.check_output([a.binary], input=chr(10).join(rows)+chr(10), text=True).splitlines()
    writes = 0
    for i, (line, want) in enumerate(zip(actual, expect, strict=True)):
        kind, *values = line.split()
        assert kind == want[0], (i, line, want)
        for x, y in zip(map(float, values), want[1:], strict=True):
            assert math.isclose(x, y, abs_tol=1e-12, rel_tol=1e-12), (i, line, want)
        writes += kind != "S"
    print(f"{len(expect)-writes} request/flush snapshots and {writes} PWM/enable events match stock")

if __name__ == "__main__":
    main()
