#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Pinned C branch-only comparison: redundant validation vs insertion proof."""
from pathlib import Path
import argparse
import json
import os
from statistics import median
import subprocess
import tempfile
from time import perf_counter_ns

from file_debug_lane_timing import cases, rows_with_prefix, timeline
from shared_window_capture import PIN, ROOT, ATTRS, XYZ_ATTRS, flatten_file, run_stock

PREP = """
// Laboratory only: check that adjacent packed clocks advance, allowing wrap.
static int
captures_advance(uint32_t before, uint32_t after)
{
    uint32_t delta = after - before;
    return delta && delta < 0x80000000U;
}
"""

EVERY = """
#if CAPTURE_ORDER_POLICY == 1
        uint32_t previous = sc->last_step_clock;
        for (struct qstep *q = sc->queue_pos; q < sc->queue_next; q++) {
            if (!captures_advance(previous, q->clock32))
                return ERROR_RET;
            previous = q->clock32;
        }
#endif
"""
ON_INSERT = """
#if CAPTURE_ORDER_POLICY == 2
    if (sc->next_step_clock <= sc->last_step_clock)
        return ERROR_RET;
    if (sc->queue_pos < sc->queue_next
        && !captures_advance(sc->queue_next[-1].clock32, sc->next_step_clock))
        return ERROR_RET;
#endif
"""


def build(upstream, temp):
    helper = upstream / "klippy/chelper"
    stock = helper / "stepcompress.c"
    source = stock.read_text()
    check = "        struct step_move move = compress_bisect_add(sc);"
    assert source.count(check) == 1
    source = source.replace(check, EVERY + check)
    marker = "// Convert previously scheduled steps into commands for the mcu"
    assert source.count(marker) == 1
    source = source.replace(marker, PREP + "\n" + marker)
    marker = "    if (unlikely(sc->next_step_clock >= sc->last_step_clock + CLOCK_DIFF_MAX))"
    assert source.count(marker) == 1
    source = source.replace(marker, ON_INSERT + marker)
    shadow = temp / "stepcompress_shadow.c"
    shadow.write_text(source)
    others = ("itersolve", "trapq", "kin_corexy", "kin_cartesian",
              "kin_extruder", "msgblock", "pyhelper")
    exes = []
    for i in range(3):
        exe = temp / f"capture_{i}"
        subprocess.run(["cc", "-O3", "-DWIRE", "-DCAPTURE_LANES",
                        f"-DCAPTURE_ORDER_POLICY={i}", "-pthread", "-I", helper,
                        ROOT / "bench/file_debug_capture_lanes.c",
                        ROOT / "bench/shared_windows.c", stock if i == 0 else shadow,
                        *(helper / f"{src}.c" for src in others), "-lm", "-o", exe], check=True)
        exes.append(exe)
    return exes


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path)
    p.add_argument("dictionary", type=Path)
    p.add_argument("--cores", default="4")
    p.add_argument("--rounds", type=int, default=120)
    a = p.parse_args()
    a.upstream, a.dictionary = a.upstream.resolve(), a.dictionary.resolve()
    assert 1 <= a.rounds <= 200
    os.sched_setaffinity(0, {int(c) for c in a.cores.split(",")})
    print("Effective Linux CPU affinity:", sorted(os.sched_getaffinity(0)), flush=True)
    assert subprocess.check_output(["git", "-C", a.upstream, "rev-parse", "HEAD"], text=True).strip() == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    hz = int(json.loads(a.dictionary.read_text())["config"]["CLOCK_FREQ"])
    fixture = ROOT / "bench/shared_extruder_fixture"
    with tempfile.TemporaryDirectory(prefix="c-captured-proof-") as td:
        t = Path(td)
        exes = build(a.upstream, t)
        gcode, xyz, e, out = (t / s for s in ("path.gcode", "xyz.txt", "e.txt", "lanes.out"))
        for tolerance in (0, int(hz * .000025)):
            for label, script, count in cases(fixture):
                gcode.write_text(script)
                rows, gold = run_stock(a.upstream, a.dictionary, fixture, t, gcode, tolerance == 0)
                meta, = rows_with_prefix(rows, "XYZ_BIND")
                eid, = {row["oid"] for row in rows_with_prefix(rows, "SHARED_OUTCOME")}
                assert meta["max_error"] == tolerance
                e_rows = rows_with_prefix(rows, "WINDOW")
                e_start = next(i for i, row in enumerate(e_rows) if row["op"] == "p")
                e.write_text(timeline(e_rows[e_start:], ATTRS))
                xyz.write_text(timeline(rows_with_prefix(rows, "XYZ"), XYZ_ATTRS))
                flattened = [flatten_file(gold[f"oid={oid}"]) for oid in [*meta["oids"], eid]]
                assert sum(steps for _, steps in flattened) == count
                expected = "".join("\n".join([*lane, "end", ""]) for lane, _ in flattened).encode()
                args = ["1", xyz, e, str(hz), str(tolerance),
                        *map(str, meta["d"]), *(str(int(v)) for v in meta["inverted"])]
                all_times = []
                for i in range(-6, a.rounds):
                    pair = [None] * 3
                    order = [(i + j) % 3 for j in range(3)]
                    if i % 2:
                        order.reverse()
                    for j in order:
                        start = perf_counter_ns()
                        with out.open("wb") as f:
                            subprocess.run([exes[j], *args], stdout=f, check=True)
                        pair[j] = perf_counter_ns() - start
                        assert out.read_bytes() == expected, (tolerance, label, i, j)
                    if i >= 0:
                        all_times.append(pair)
                ms = [round(median(values)/1e6,3) for values in zip(*all_times)]
                proof_vs_each = 100 * median(c/b-1 for _,b,c in all_times)
                proof_vs_stock = 100 * median(c/a-1 for a,_,c in all_times)
                print(tolerance, label, count, "ms [stock/each/proof]", ms,
                      f"proof vs each {proof_vs_each:+.2f}%, vs stock {proof_vs_stock:+.2f}%", flush=True)


if __name__ == "__main__":
    main()
