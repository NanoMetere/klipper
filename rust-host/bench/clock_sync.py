#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Replay the same synthetic timestamps through pinned stock Klippy and Rust."""
import argparse
import collections
import importlib.util
import math
from pathlib import Path
import random
import statistics
import subprocess
import tempfile
from time import perf_counter_ns

PIN = "c9559fa04ec6b11fb1a9a1610903546319fcf310"
MASK = (1 << 32) - 1
ROOT = Path(__file__).resolve().parents[1]


class Reactor:
    def register_timer(self, _):
        return None


class Serial:
    def __init__(self):
        self.release = None

    def set_clock_est(self, hz, when, tick):
        self.release = (when, tick, hz)


def scenarios():
    for name, hz, base, drift, timestamp in [
        ("jitter", 1e6, 500000, 0.00012, 20.0),
        ("multiple-wraps", 48e6, (1 << 32) - 350_000, -0.0002, 50.0),
        ("reinitialized", 120e6, 251_000, 0.0003, 971.25),
        ("high-uptime", 16e6, (1 << 44) + 5000, -0.00002, 173700.0),
    ]:
        rng = random.Random(2026 + len(name))
        rows = []
        for i in range(1500):
            distance = (i + 1) * 0.050 if i < 8 else 0.4 + (i - 7) * 0.9839
            sent = timestamp + distance + rng.uniform(-0.0005, 0.0005)
            uplink = rng.uniform(0.00004, 0.00013)
            downlink = rng.uniform(0.00008, 0.00045)
            clock = base + int((sent - timestamp + uplink) * hz * (1 + drift))
            if i % 113 == 33:
                clock += int(0.030 * hz)  # forward outlier, but before next sample
            if i % 127 == 44:
                clock -= int(0.018 * hz)  # triggers variance reset
            if i % 101 == 17:
                sent = received = 0.0
            else:
                received = sent + uplink + downlink
            rows.append((clock & MASK, sent, received, i < 8))
        yield name, (hz, base, timestamp), rows


def init(stock, header, model=None):
    hz, ticks, sent = header
    model = model if model is not None else stock.ClockSync(Reactor())
    model.serial = Serial()
    # Equivalent to the uptime seeding in ClockSync.connect, before eight reads.
    model.mcu_freq = hz
    model.last_clock = model.clock_avg = ticks
    model.time_avg = sent
    model.clock_est = (sent, ticks, hz)
    model.prediction_variance = (0.001 * hz) ** 2
    return model


def run_stock(stock, header, rows):
    model = init(stock, header)
    for low, sent, recv, warmup in rows:
        if warmup:
            model.last_prediction_time = -9999.0
        previous = model.serial.release
        model._handle_clock({"clock": low, "#sent_time": sent, "#receive_time": recv})
        at, tick, hz = model.clock_est
        kind = "untimed" if not sent else "outlier" if model.serial.release is previous else "updated"
        expected = (model.last_clock, at, tick, hz, model.clock32_to_clock64((low - 331) & MASK),
                    tick + (at + 0.5 - at) * hz, model.estimate_clock_systime(model.last_clock))
        yield expected, kind, model.serial.release


def timing_stock(stock, header, rows, repeats):
    # Prebuild parsed samples. The stock API takes a dictionary.
    prepared = [({"clock": low, "#sent_time": sent, "#receive_time": recv}, warm)
                for low, sent, recv, warm in rows]
    begin = perf_counter_ns()
    for _ in range(repeats):
        model = init(stock, header)
        for params, warm in prepared:
            if warm:
                model.last_prediction_time = -9999.0
            model._handle_clock(params)
    return perf_counter_ns() - begin


def encode(header, rows):
    return " ".join(map(str, header)) + "\n" + "".join(
        f"{low} {sent!r} {recv!r} {int(warm)}\n" for low, sent, recv, warm in rows)


def compare(binary, stock, name, header, rows):
    run = subprocess.run([binary], input=encode(header, rows), capture_output=True, text=True, check=True)
    lines = run.stdout.splitlines()
    assert len(lines) == 2 * len(rows)
    kinds = collections.Counter()
    metrics = [0.0, 0.0, 0]
    for i, (want, kind, release) in enumerate(run_stock(stock, header, rows)):
        got = lines[i * 2].split()
        report = lines[i * 2 + 1].split()
        assert report[0] == kind, (name, i, report, kind)
        kinds[kind] += 1
        for j, value in enumerate(want):
            if j in [0, 4]:
                assert int(got[j]) == value, (name, i, j, got, want)
            else:
                error = abs(float(got[j]) - value)
                allowed = max(math.ulp(float(value)) * 4, 1e-6 if j in [2, 3, 5] else 1e-9)
                assert error <= allowed, (name, i, j, error, allowed)
                if j == 3:
                    metrics[0] = max(metrics[0], error)
                if j == 5:
                    metrics[1] = max(metrics[1], error)
        if kind == "updated":
            assert abs(float(report[1]) - release[0]) <= 1e-9
            delta = abs(int(report[2]) - release[1])
            assert delta <= 1
            metrics[2] = max(metrics[2], delta)
            assert math.isclose(float(report[3]), release[2], rel_tol=1e-12)
    print(f"{name}: {len(rows)} samples, {dict(kinds)}, "
          f"max delta {metrics[0]:.3g} Hz / {metrics[1]:.3g} ticks / {metrics[2]} release tick")


def compare_secondary(binary, stock, fixture):
    _, ha, rows_a = fixture[0]
    _, hb, rows_b = fixture[1]
    shift = hb[2] - ha[2]
    hb = (hb[0], hb[1], hb[2] - shift)
    a = init(stock, ha)
    b = init(stock, hb, stock.SecondarySync(Reactor(), a))
    expected, data = [], []
    def observe(model, row, warmup):
        low, sent, recv = row
        if warmup:
            model.last_prediction_time = -9999.0
        model._handle_clock({"clock": low, "#sent_time": sent, "#receive_time": recv})
    for i, ((x, sa, ra, _), (y, sb, rb, _)) in enumerate(zip(rows_a, rows_b)):
        if sb:
            sb -= shift
            rb -= shift
        at = max(ra, rb, ha[2] + i * 0.05) + 0.001
        observe(a, (x, sa, ra), i < 8)
        observe(b, (y, sb, rb), i < 8)
        lead = (i % 11) * 0.55
        print_target = 0.0 if i < 7 else a.estimated_print_time(at) + lead
        data.append(f"{x} {sa!r} {ra!r} {y} {sb!r} {rb!r} {at!r} {print_target!r}\n")
        if i < 7:
            continue
        if i == 7:
            b.clock_adj = (0.0, b.mcu_freq)
            offset = a.estimated_print_time(at) - b.estimated_print_time(at)
            b.clock_adj = (offset, b.mcu_freq)
            b.calibrate_clock(0.0, at)
        offset, hz = b.calibrate_clock(print_target, at)
        tick = b.print_time_to_clock(b.last_sync_time)
        expected.append((offset, hz, b.last_sync_time, tick, b.clock_to_print_time(tick)))
    headers = "\n".join(" ".join(map(str, header)) for header in [ha, hb]) + "\n"
    run = subprocess.run([binary, "--secondary"], input=headers + "".join(data),
                         capture_output=True, text=True, check=True)
    lines = run.stdout.splitlines()
    assert len(lines) == len(expected)
    max_tick = 0
    for i, (line, want) in enumerate(zip(lines, expected)):
        words = line.split()
        tick_gap = abs(int(words[3]) - want[3])
        max_tick = max(max_tick, tick_gap)
        assert tick_gap <= 1, (i, tick_gap)
        for j in [0, 1, 2, 4]:
            atol = [1e-9, 1e-6, 1e-9, 1e-7][[0, 1, 2, 4].index(j)]
            assert abs(float(words[j]) - want[j]) <= atol, (i, j, words, want)
    print(f"secondary: {len(expected)} adjustments, max horizon difference {max_tick} ticks")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path, help=f"disposable Klipper tree pinned at {PIN}")
    parser.add_argument("--bench", type=int, default=0, metavar="REPEATS")
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("stock_clocksync", args.upstream / "klippy/clocksync.py")
    stock = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(stock)
    with tempfile.TemporaryDirectory() as tmp:
        lib = Path(tmp) / "libklipper_host_protocol.rlib"
        binary = Path(tmp) / "clock-sync"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib", "--crate-name=klipper_host_protocol",
                        ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/clock_sync.rs", "--extern",
                        f"klipper_host_protocol={lib}", "-o", binary], check=True)
        fixture = list(scenarios())
        for name, header, rows in fixture:
            compare(binary, stock, name, header, rows)
        compare_secondary(binary, stock, fixture)
        if args.bench > 0:
            _, header, rows = fixture[0]
            data = {"stock": [], "rust": []}
            for n in range(8):
                for host in list(data)[::(-1 if n % 2 else 1)]:
                    if host == "stock":
                        elapsed = timing_stock(stock, header, rows, args.bench)
                    else:
                        elapsed = int(subprocess.check_output([binary, str(args.bench)],
                                      input=encode(header, rows), text=True))
                    if n:
                        data[host].append(elapsed / (args.bench * len(rows)))
            for host, values in data.items():
                print(f"{host}: {statistics.median(values):.2f} ns/sample median "
                      f"({min(values):.2f}–{max(values):.2f}, n={len(values)})")


if __name__ == "__main__":
    main()
