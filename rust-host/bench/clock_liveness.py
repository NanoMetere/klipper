#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare pinned clock timeout policy with native C and Rust, without I/O."""
from pathlib import Path
from types import SimpleNamespace
import argparse
import hashlib
import os
import random
import statistics
import subprocess
import sys
import tempfile
import time

from planner import PIN, ROOT

class Serial:
    def __init__(self):
        self.sent = self.aborts = 0
    def raw_send(self, *args):
        self.sent += 1
    def abort_wait_acks(self):
        self.aborts += 1

def case(fileoutput, actions, clock_module, mcu_module):
    trace = []
    serial = Serial()
    clock = clock_module.ClockSync(SimpleNamespace(register_timer=lambda callback: callback))
    clock.serial = serial
    shutdowns = []
    helper = object.__new__(mcu_module.MCUConnectHelper)
    helper._serial = serial
    helper._printer = SimpleNamespace(invoke_shutdown=shutdowns.append)
    helper._is_timeout, helper._name, helper._clocksync = False, "fixture", clock
    helper._mcu = SimpleNamespace(is_fileoutput=lambda: fileoutput)
    sends = 0
    for i, op in enumerate(actions):
        due = 0
        if op == "q":
            at = i * .125
            deadline = clock._get_clock_event(at)
            sends += 1
            assert deadline == at + .9839 and serial.sent == sends
            due = round(deadline * 10000)
        elif op == "r":
            clock._handle_clock({"clock": i, "#sent_time": 0.,
                                 "#receive_time": i * .125})
        elif op == "c":
            helper.check_timeout(i * .125)
        assert serial.aborts == len(shutdowns)
        assert not shutdowns or shutdowns == ["Lost communication with MCU 'fixture'"]
        trace.append(f"{int(clock.is_active())} {int(helper._is_timeout)} {serial.aborts} {serial.sent} {due}\n")
    return trace

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stock", type=lambda v: Path(v).resolve())
    parser.add_argument("binary", type=lambda v: Path(v).resolve())
    parser.add_argument("--rounds", type=int, default=0)
    parser.add_argument("--core", type=int)
    args = parser.parse_args()
    if not 0 <= args.rounds <= 200:
        parser.error("--rounds must be 0–200")
    if args.core is not None:
        os.sched_setaffinity(0, {args.core})
    assert subprocess.check_output(["git", "-C", args.stock, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", args.stock, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(args.stock / "klippy"))
    import clocksync
    import mcu

    rng = random.Random(191)
    cases = [(mode, script) for mode in (0, 1) for script in
             (["cqqqqcqc", "cqqqqrqc", "qqqqqcqrccqqqqqqcc", "rqqqrcq" * 3] +
              ["".join(rng.choices("qrc", (12, 2, 4), k=256)) for _ in range(25)])]
    data = "".join(f"{mode} {script}\n" for mode, script in cases).encode()
    expect = "".join(row for mode, script in cases
                     for row in case(mode, script, clocksync, mcu)).encode()
    with tempfile.TemporaryDirectory(prefix="clock-liveness-policy-") as tmp:
        c, bounded = [Path(tmp) / name for name in ("plain-c", "bounded-c")]
        for binary, flags in ((c, []), (bounded, ["-DSATURATING"])):
            subprocess.run(["cc", "-O3", *flags, ROOT / "bench/clock_liveness.c",
                            "-o", binary], check=True)
        arms = [c, bounded, args.binary]
        for arm in arms:
            assert subprocess.check_output([arm], input=data, timeout=10) == expect
        samples = []
        for i in range(-6, args.rounds) if args.rounds else ():
            wall = [0, 0, 0]
            order = [(i+j) % 3 for j in range(3)]
            for j in order if i % 2 == 0 else reversed(order):
                start = time.perf_counter_ns()
                output = subprocess.check_output([arms[j]], input=data, timeout=10)
                wall[j] = time.perf_counter_ns() - start
                assert output == expect
            if i >= 0:
                samples.append(wall)
        if samples:
            medians = [round(statistics.median(v) / 1e6, 3) for v in zip(*samples)]
            deltas = [round(100 * statistics.median(r[i] / r[0] - 1 for r in samples), 2)
                      for i in (1, 2)]
            print("whole replay: baseline C, bounded C, Rust (ms):", medians)
            print("paired changes vs baseline C (%):", deltas)
    print(len(cases), len(expect.splitlines()), hashlib.sha256(expect).hexdigest(),
          "exact pinned stock, both C, Rust")


if __name__ == "__main__":
    main()
