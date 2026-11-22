#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Measure actual Klippy flush calls including Python/Rust marshalling."""
import argparse
import importlib.util
import math
from pathlib import Path
import statistics
import subprocess
import sys
import time

from lookahead import fixture, stock
from planner import PIN, ROOT


def load_adapter(path):
    spec = importlib.util.spec_from_file_location("fixture_adapter", path)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    return adapter


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"stock checkout at {PIN}")
    p.add_argument("library", type=Path)
    p.add_argument("--rounds", type=int, default=30)
    p.add_argument("--baseline-adapter", type=Path)
    a = p.parse_args()
    if not 1 <= a.rounds <= 200:
        p.error("--rounds must be between 1 and 200")
    upstream = a.upstream.resolve()
    pin = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    if pin != PIN:
        p.error(f"stock checkout must be {PIN}")
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(upstream / "klippy"))
    import toolhead
    queue_type = toolhead.LookAheadQueue
    original = queue_type.flush
    flushes = []

    def capture(queue, lazy=False):
        flushes.append((queue.queue[:], lazy))
        return original(queue, lazy)

    queue_type.flush = capture
    try:
        stock([(row[0], row[1:]) for row in fixture()], False)
    finally:
        queue_type.flush = original

    fns, stats = {"stock": original}, {}
    versions = {"adapter": ROOT / "python/klipper_rust_lookahead.py"}
    if a.baseline_adapter:
        versions["baseline adapter"] = a.baseline_adapter.resolve()
    for label, path in versions.items():
        try:
            stats[label] = load_adapter(path).install(toolhead, a.library.resolve())
            fns[label] = queue_type.flush
        finally:
            queue_type.flush = original

    def replay(flush, verify=False):
        queues = [queue_type() for _ in flushes]
        for queue, (moves, _) in zip(queues, flushes, strict=True):
            queue.queue = moves[:]
        outputs = []
        # Rebuilding each queue and capturing result fields stay out of
        # reported CPU timing; packing, kernel, and set_junction stay in.
        started = time.process_time_ns()
        for queue, (_, lazy) in zip(queues, flushes, strict=True):
            result = flush(queue, lazy)
            if verify:
                outputs.append((len(queue.queue), [
                    (m.start_v, m.cruise_v, m.end_v, m.accel_t, m.cruise_t, m.decel_t)
                    for m in result
                ]))
        return outputs if verify else time.process_time_ns() - started

    reference = replay(original, verify=True)
    for label, fn in fns.items():
        if label == "stock":
            continue
        actual = replay(fn, verify=True)
        for i, ((left, stock_moves), (right, rust_moves)) in enumerate(zip(reference, actual, strict=True)):
            assert left == right, (label, i, left, right)
            for x, y in zip(stock_moves, rust_moves, strict=True):
                assert all(math.isclose(j, k, rel_tol=2e-12, abs_tol=2e-12)
                           for j, k in zip(x, y, strict=True)), (label, i, x, y)
    series = {label: [] for label in fns}
    for i in range(a.rounds):
        labels = list(fns)
        order = i % len(labels)
        for label in labels[order:] + labels[:order]:
            series[label].append(replay(fns[label]))
    moves = sum(len(out) for _, out in reference)
    print(f"{len(flushes)} stock queue snapshots; {moves} planned moves matched")
    for label, values in series.items():
        if label in stats:
            assert stats[label]["fallbacks"] == 0 and stats[label]["planned"] == moves * (a.rounds + 1), stats[label]
        print(f"{label}: {statistics.median(values)/1e6:.3f} ms median CPU per replay "
              f"({min(values)/1e6:.3f}–{max(values)/1e6:.3f} ms)")


if __name__ == "__main__":
    main()
