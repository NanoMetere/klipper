#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline controller duty and temperature smoothing with stock."""
import argparse
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace

from planner import PIN, ROOT


def fixture():
    r = random.Random(82348)
    for i in range(100):
        power, smooth = r.choice([.23,.71,1.]), r.choice([.1,.4,1.,2.])
        if i % 2:
            yield ("p", power, smooth, r.uniform(10,60), r.choice([0, 1.6, 6]), r.uniform(10,130))
        else:
            yield ("w", power, smooth, r.uniform(.05,4.))
        time, temp = 0., 25.
        for j in range(220):
            target = (0, 55, 220, 220, 0)[min(4, j//44)]
            time += r.choice([.02,.08,.31,2.5])
            temp += .25 * ((target or 25) - temp) + r.gauss(0.,1.1)
            yield ("u", time, temp, target, int(r.randrange(16) != 0))


def compare(stock, rows):
    controller = None
    out = []
    for op, *v in rows:
        if op != "u":
            power, smooth = v[:2]
            opts = dict(zip(("pid_Kp", "pid_Ki", "pid_Kd"), v[2:])) if op == "p" else {"max_delta":v[2]}
            heater = SimpleNamespace(get_max_power=lambda: power, get_smooth_time=lambda: smooth,
                                     set_pwm=lambda _, u: setattr(heater, "value", u))
            cfg = SimpleNamespace(getfloat=lambda k, *_, **__: opts[k])
            cls = stock.ControlPID if op == "p" else stock.ControlBangBang
            controller = cls(heater, cfg)
            smooth_temp, last_time = 0., 0.
        else:
            time, temperature, target, live = v
            controller.temperature_update(time, temperature, target)
            smooth_temp += (temperature - smooth_temp) * min((time-last_time)*(1/smooth), 1.)
            last_time = time
            out.append((heater.value if live and target > 0 else 0., smooth_temp,
                        float(controller.check_busy(time, smooth_temp, target))))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    a = p.parse_args()
    sys.path.insert(0, str(a.upstream.resolve() / "klippy"))
    from extras import heaters
    rows = list(fixture())
    want = compare(heaters, rows)
    with tempfile.TemporaryDirectory() as dir_:
        t = Path(dir_)
        rust, lib = t / "rust", t / "libklipper_host_protocol.rlib"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/thermal.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        input_ = "\n".join(" ".join(map(str, r)) for r in rows) + "\n"
        got = subprocess.check_output([rust], text=True, input=input_).splitlines()
        for i, (x, y) in enumerate(zip(want, got, strict=True)):
            assert all(math.isclose(f, g, abs_tol=2e-12, rel_tol=2e-12)
                       for f, g in zip(x, map(float,y.split()), strict=True)), (i, x, y)
        print(f"{len(want)} offline temperature updates matched stock calculations")


if __name__ == "__main__":
    main()
