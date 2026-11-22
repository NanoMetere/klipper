#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare stock HeaterCheck decisions from synthetic telemetry, no heaters."""
import argparse
import logging
import math
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

from planner import PIN, ROOT


def fixtures(upstream):
    sys.path.insert(0, str(upstream / "klippy"))
    from extras.verify_heater import HeaterCheck
    rng = random.Random(3194)
    rows, want = [], []
    trials = 320
    for trial in range(trials):
        name = "heater_bed" if trial % 3 == 0 else "extruder"
        options = {
            "hysteresis": rng.choice([2.0, 5.0, 8.0]),
            "max_error": rng.choice([0., 80., 120., 145., 400.]),
            "heating_gain": rng.choice([1.5, 2.0, 3.0]),
            "check_gain_time": rng.choice([10., 20., 60.]),
        }
        shutdown = []
        printer = SimpleNamespace(
            register_event_handler=lambda *args: None,
            get_reactor=lambda: SimpleNamespace(NEVER=1e30),
            invoke_shutdown=shutdown.append)
        cfg = SimpleNamespace(get_printer=lambda: printer,
                              get_name=lambda: f"verify_heater {name}",
                              getfloat=lambda key, default, **bounds: options[key])
        check = HeaterCheck(cfg)
        t, temp, target = rng.uniform(1, 10), 25., 0.
        check.heater = SimpleNamespace(get_temp=lambda _: (temp, target))
        rows.append("new " + " ".join(map(str, options.values())))
        for i in range(280):
            if i in (2, 80, 130, 220):
                target = (115. if name == "heater_bed" else 220.) if i % 3 else 0.
            if trial % 5 == 0:
                delta = 0.0
            elif trial % 5 == 1:
                delta = rng.uniform(-0.30, 2.0)
            elif trial % 5 == 2:
                delta = rng.uniform(0.0, 8.0) if i < 150 else -1.1
            else:
                delta = (target - temp) * .12 + rng.uniform(-1, 1)
            temp = max(20., min(255., temp + delta))
            t += rng.uniform(.9, 1.06)
            rows.append(f"sample {t} {temp} {target}")
            next_at = check.check_event(t)
            want.append([float(bool(shutdown)), float(check.approaching_target),
                         float(check.starting_approach), check.last_target, check.goal_temp,
                         check.error, check.goal_systime, next_at])
            if shutdown:
                assert len(shutdown) == 1 and "not heating" in shutdown[0]
                break
    return rows, want, trials


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/verify_heater_replay")
    a = p.parse_args()
    logging.disable(logging.CRITICAL)
    rows, expected, trials = fixtures(a.upstream.resolve())
    out = subprocess.check_output([a.binary], text=True, input="\n".join(rows)+"\n").splitlines()
    for i, (want, got) in enumerate(zip(expected, out, strict=True)):
        for x, y in zip(want, map(float, got.split()), strict=True):
            assert math.isclose(x, y, abs_tol=1e-10, rel_tol=1e-12), (i, want, got)
    print(f"{len(out)} heater monitor decisions across {trials} traces; "
          f"{sum(int(w[0]) for w in expected)} stock-equivalent faults")


if __name__ == "__main__":
    main()
