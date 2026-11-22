#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Differential test and offline benchmark for stock Klippy move look-ahead."""
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
    yield ("r", 150, 2000, 5, .5, 1)
    yield ("m", 1e-10, 0, 0, .125, 15, -1, 0, -1)
    yield ("m", 0, 8, 0, .125, 80, -1, 0, -1)
    yield ("f", 1)
    yield ("m", 0, 0, 0, .125, 80, -1, 0, -1)
    yield ("f", 0)
    r = random.Random(20261122)
    for profile in range(480):
        yield ("r", 25 + r.randrange(400), 120 + r.randrange(13000),
               r.choice([0, 2, 5, 9]), r.choice([0., .25, .5, .75, .95]),
               r.choice([-1, .5, 1, 3, 5]))
        pos = [0.] * 4
        last = [1., 0., 0.]
        for j in range(r.randrange(10, 96)):
            if j % 12 == 0:
                delta = last
            elif j % 12 == 1:
                delta = [-d for d in last]
            else:
                delta = r.choice([[1., 0., 0.], [-1., 0., 0.], [0., 1., 0.],
                                  [0., 0., 1.], [1., 1., 0.], [-1., 1., 0.],
                                  [1., 2., -1.], [-.8, -.2, .5], [0., 0., 0.]])
            last = delta
            length = r.choice([.001, .043, .295, .33, .5, 2., 3., 18., 30.])
            if delta == [0., 0., 0.]:
                e = r.choice([-.3, .125, .5])
            else:
                e = r.choice([0., 0., 0., .002, -.015, .021]) * length
            pos = [x + d * length for x, d in zip(pos[:3], delta)] + [pos[3] + e]
            if any(not math.isfinite(x) for x in pos):
                raise ValueError(pos)
            yield ("m", *pos, r.uniform(1., 500.),
                   r.choice([-1., -1., -1., 14., 42.]), r.uniform(50., 15000.),
                   r.choice([-1., -1., -1., 0., 3., 14.]))
            if r.randrange(5) == 0:
                yield ("f", 1)
            elif r.randrange(33) == 0:
                yield ("f", 0)
        yield ("f", 0)


def stock(cases, emit):
    from toolhead import Move, LookAheadQueue, ToolHead
    from kinematics.extruder import PrinterExtruder
    samples = 0
    time_sum = 0.
    for op, v in cases:
        if op == "r":
            velocity, accel, scv, ratio, corner = v
            extras = []
            if corner >= 0.:
                axis = SimpleNamespace(instant_corner_v=corner)
                axis.calc_junction = lambda a, b, i, axis=axis: PrinterExtruder.calc_junction(axis, a, b, i)
                extras.append(axis)
            head = SimpleNamespace(max_velocity=velocity, max_accel=accel,
                                   square_corner_velocity=scv, min_cruise_ratio=ratio,
                                   extra_axes=extras)
            ToolHead._calc_junction_deviation(head)
            queue = LookAheadQueue()
            pos = [0.] * 4
        elif op == "m":
            end = v[:4]
            move = Move(head, pos, end, v[4])
            if v[5] > 0:
                move.limit_speed(v[5], v[6])
            if v[7] >= 0.:
                move.limit_next_junction_speed(v[7])
            lazy = queue.add_move(move)
            if emit:
                print("push", int(bool(lazy)), len(queue.queue))
            pos = move.end_pos
        else:
            out = queue.flush(bool(v[0]))
            if emit:
                print("flush", len(out), len(queue.queue))
            samples += len(out)
            for move in out:
                time_sum += move.accel_t + move.cruise_t + move.decel_t
                if emit:
                    print("move", int(move.is_kinematic_move), *move.start_pos, *move.end_pos, *move.axes_r,
                          move.start_v, move.cruise_v, move.end_v, move.accel,
                          move.accel_t, move.cruise_t, move.decel_t)
    return samples, time_sum


def stock_cli():
    cases = [(fields[0], list(map(float, fields[1:]))) for fields in map(str.split, sys.stdin)]
    if len(sys.argv) == 2:
        stock(cases, True)
    else:
        stock(cases, False)
        for _ in range(int(sys.argv[2])):
            start = time.perf_counter_ns()
            count, duration = stock(cases, False)
            print(time.perf_counter_ns() - start, count, duration)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"disposable Klipper tree at {PIN}")
    p.add_argument("--rounds", type=int, default=7)
    args = p.parse_args()
    corpus = "\n".join(" ".join(map(str, row)) for row in fixture()) + "\n"
    with tempfile.TemporaryDirectory() as dir_:
        tmp = Path(dir_)
        rust, lib = tmp / "lookahead", tmp / "libklipper_host_protocol.rlib"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/lookahead.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        env = os.environ.copy()
        env["PYTHONPATH"] = str(args.upstream.resolve() / "klippy") + os.pathsep + env.get("PYTHONPATH", "")
        programs = {"stock": [sys.executable, __file__, "--stock"], "rust": [rust]}

        def run(argv):
            return subprocess.check_output(argv, env=env, text=True, input=corpus).splitlines()

        want, got = (run(program) for program in programs.values())
        counts = {}
        max_abs = 0.
        for row, (expected, actual) in enumerate(zip(want, got, strict=True)):
            a, b = expected.split(), actual.split()
            label = a[0]
            assert a[:2] == b[:2], (row, expected, actual)
            counts[label] = counts.get(label, 0) + 1
            if label != "move":
                assert a == b, (row, expected, actual)
            else:
                for x, y in zip(map(float, a[2:]), map(float, b[2:]), strict=True):
                    assert math.isclose(x, y, rel_tol=2e-12, abs_tol=2e-12), (row, expected, actual)
                    max_abs = max(max_abs, abs(x-y))
        print(f"{counts}; maximum profile field absolute difference {max_abs:g}")
        if args.rounds == 0:
            return
        timings = {k: [] for k in programs}
        reference = None
        for i in range(args.rounds):
            for kind in tuple(programs)[::(-1 if i % 2 else 1)]:
                elapsed, count, duration = run([*programs[kind], "1"])[0].split()
                assert int(count) == counts["move"]
                total = float(duration)
                if reference is None:
                    reference = total
                assert math.isclose(total, reference, rel_tol=2e-12, abs_tol=2e-12)
                timings[kind].append(int(elapsed)/int(count))
        for kind, samples in timings.items():
            print(f"{kind}: {statistics.median(samples):.1f} ns/move median, "
                  f"{min(samples):.1f}–{max(samples):.1f} ns/move range; {samples}")


if __name__ == "__main__":
    if "--stock" in sys.argv:
        stock_cli()
    else:
        main()
