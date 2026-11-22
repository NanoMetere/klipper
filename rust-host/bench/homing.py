#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare the offline homing model with pinned Klippy using fake counters."""
import argparse
import math
import random
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

from planner import PIN, ROOT


class Toolhead:
    def __init__(self, pos, kin):
        self.pos, self.kin = pos, kin
        self.moves = []

    def get_position(self):
        return self.pos

    def get_kinematics(self):
        return self.kin

    def move(self, pos, speed):
        self.moves.append((pos, speed))

    def set_position(self, pos):
        self.pos = pos


class Motor:
    def __init__(self, idx, transform, step):
        self.idx, self.transform, self.step = idx, transform, step

    def calc_position_from_coord(self, p):
        return self.transform(p)[self.idx]

    def get_step_dist(self):
        return self.step

    def get_steppers(self):
        return [self]

    def get_name(self):
        return str(self.idx)


def fixtures(upstream):
    sys.path.insert(0, str(upstream / "klippy"))
    from extras.homing import Homing, HomingMove
    from kinematics.cartesian import CartKinematics
    from kinematics.corexy import CoreXYKinematics

    rng = random.Random(4109)
    for name, cls in [("cart", CartKinematics), ("core", CoreXYKinematics)]:
        def transform(coords):
            x, y, z = coords[:3]
            return [x, y, z] if name == "cart" else [x + y, x - y, z]

        for iteration in range(500):
            mask = rng.randint(1, 7)
            start = [rng.uniform(-5, 150) for _ in range(3)]
            end = [rng.uniform(-5, 150) for _ in range(3)]
            if iteration % 13 == 0:
                end = start.copy()
            elif iteration % 7 == 0:
                end = start.copy()
                end[iteration % 3] += rng.uniform(1, 50)
            dist = [rng.choice([.0125, .00625, .0025, .04]) for _ in range(3)]
            speed = rng.uniform(1., 40.)
            motors = [Motor(i, transform, d) for i, d in enumerate(dist)]
            kin = object.__new__(cls)
            kin.rails = motors
            kin.dc_module = None
            toolhead = Toolhead(start + [2.5], kin)
            hmove = HomingMove(None, [], toolhead=toolhead)
            endstop = SimpleNamespace(get_steppers=lambda: [m for m in motors
                                                            if mask & (1 << m.idx)])
            estimate = hmove._calc_endstop_rate(endstop, end + [2.5], speed)
            yield ["rate", name, *start, *end, *dist, speed, mask], [estimate]

            if end != start and hasattr(Homing, "_retract_move"):
                # Test both clamped and nonclamped retract ratios.
                length = 500. if iteration % 4 == 0 else rng.uniform(.01, 5.)
                h = object.__new__(Homing)
                h.toolhead = toolhead
                h._retract_move(SimpleNamespace(retract_dist=length, retract_speed=10.), start, end)
                yield ["retract", name, *start, *end, length], toolhead.moves[-1][0][:3] + toolhead.pos[:3]

            hmove.toolhead.pos = start + [2.5]
            hmove.printer = SimpleNamespace(get_start_args=lambda: {})
            reading_fields, counters = [], {}
            for i in range(3):
                mcu_start = rng.randint(-50000, 50000)
                trig = mcu_start + (rng.randint(-1000, 1000) if iteration % 9 else 0)
                halt = trig + (rng.randint(-10, 10) if iteration % 5 else 0)
                reading_fields.extend([int(bool(mask & (1 << i))), mcu_start, trig, halt])
                if mask & (1 << i):
                    counters[str(i)] = SimpleNamespace(start_pos=mcu_start, trig_pos=trig, halt_pos=halt,
                                                       endstop_name=str(i))
            hmove.stepper_positions = list(counters.values())
            probe, triggered, require = map(bool, [iteration & 1, iteration % 11, iteration & 2])
            motor_pos = {str(i): value for i, value in enumerate(transform(start))}
            stopped = hmove.check_no_movement() if require else None
            status = -2 if not triggered else -1 if stopped is None else int(stopped)
            if probe:
                trigger = hmove.calc_toolhead_pos(motor_pos, {i: c.trig_pos - c.start_pos
                                                              for i, c in counters.items()})
                halt = hmove.calc_toolhead_pos(motor_pos, {i: c.halt_pos - c.start_pos
                                                           for i, c in counters.items()})
            else:
                trigger = halt = start
                excess = {i: c.halt_pos - c.trig_pos for i, c in counters.items()}
                if any(excess.values()):
                    halt = hmove.calc_toolhead_pos(motor_pos, excess)
            yield ["positions", name, *start, *dist, int(probe), int(triggered), int(require),
                   *reading_fields], [status, *trigger[:3], *halt[:3]]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    items = list(fixtures(p.parse_args().upstream.resolve()))
    with tempfile.TemporaryDirectory() as directory:
        lib = Path(directory) / "libklipper_host_protocol.rlib"
        exe = Path(directory) / "homing"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/homing.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", exe], check=True)
        run = subprocess.run([exe], text=True, check=True, capture_output=True,
                             input="\n".join(" ".join(map(str, fields)) for fields, _ in items) + "\n")
    counts = {}
    for (fields, expected), line in zip(items, run.stdout.splitlines(), strict=True):
        for actual, target in zip(map(float, line.split()), expected, strict=True):
            assert math.isclose(actual, target, rel_tol=1e-12, abs_tol=1e-12), (fields, actual, target)
        counts[fields[0]] = counts.get(fields[0], 0) + 1
    print(f"Matched offline homing: {counts}")


if __name__ == "__main__":
    main()
