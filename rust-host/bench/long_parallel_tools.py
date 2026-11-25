#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Replay three independent extruder queues with shared immutable XYZ motion."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

from long_parallel import build, timed
from planner import PIN, ROOT
from tool_motion import cli_rows, compare_runtime, long_gcode, reference


def capture_plan(groups):
    assert len(groups) == 4
    xyz, *tools = groups
    assert len([r for r in xyz if r[0] == "s"]) == 3
    initial = [r for r in xyz if r[0] != "s"]
    lanes = [r for r in xyz if r[0] == "s"]
    for index, tool in enumerate(tools):
        assert tool[-1][:2] == ("s", "e") and all(r[0] in ("e", "g") for r in tool[:-1])
        initial.extend([("t", index), *tool[:-1]])
        lanes.append(("u", index, *tool[-1][1:]))
    return "".join(" ".join(map(str, r)) + "\n" for r in [*initial, *lanes]).encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("stock", "serial", "parallel"):
        parser.add_argument(name, type=lambda p: Path(p).resolve())
    parser.add_argument("--rounds", type=int, default=0)
    parser.add_argument("--cpu-mask")
    args = parser.parse_args()
    if not 0 <= args.rounds <= 80:
        parser.error("--rounds must be 0–80")
    if args.cpu_mask:
        os.sched_setaffinity(0, set(map(int, args.cpu_mask.split(","))))
    stock, serial, parallel = args.stock, args.serial, args.parallel
    assert subprocess.check_output(["git", "-C", stock, "rev-parse", "HEAD"], text=True).strip() == PIN
    subprocess.run(["git", "-C", stock, "diff", "--quiet", "HEAD", "--"], check=True)

    with tempfile.TemporaryDirectory(prefix="long-parallel-tools-") as td:
        work = Path(td)
        seq, native, output = (work / n for n in ("serial-c", "parallel-c", "out"))
        build(stock, seq, "machine_trace.c")
        build(stock, native, "long_parallel.c")

        def compare(label, flags, old, expected, groups):
            payload = capture_plan(groups)
            packets = ("\n".join(expected) + "\n").encode()
            gold = subprocess.check_output([serial, *flags], timeout=25)
            assert json.loads(gold) == old
            assert old["tool_mode"] and [x["tool"] for x in old["extruder_lanes"]] == [0, 2, 6]
            rust = [[parallel, "--lane-workers", str(i), *flags] for i in (1, 2, 3, 4)]
            c = [[native, str(i)] for i in (1, 2, 3, 4)]
            assert subprocess.check_output([parallel, *flags], timeout=25) == gold
            for arm in rust:
                assert subprocess.check_output(arm, timeout=25) == gold, (label, arm)
            for arm in c:
                assert subprocess.check_output(arm, input=payload, timeout=25,
                                               stderr=subprocess.DEVNULL) == packets, (label, arm)
            for count in (0, 5, 8):
                proc = subprocess.run([parallel, "--lane-workers", str(count), *flags],
                                      timeout=15, capture_output=True)
                assert proc.returncode and not proc.stdout, (label, count)
            extra_tool = flags.copy()
            temp_arg = extra_tool.index("--tool-temps") + 1
            extra_tool[temp_arg] += ",9:212"
            for invalid in ([parallel, "--lane-workers", "4", *extra_tool],
                            [parallel, "--lane-workers", "4", "--mixed-plan", *flags]):
                proc = subprocess.run(invalid, timeout=15, capture_output=True)
                assert proc.returncode and not proc.stdout
            bad = payload.replace(b"u 0 e ", b"u 3 e ", 1)
            assert bad != payload
            bad_proc = subprocess.run([native, "4"], input=bad, timeout=15, capture_output=True)
            assert bad_proc.returncode == 2 and not bad_proc.stdout
            sizes = [len(x) for x in old["motors"]] + [len(x["commands"]) for x in old["extruder_lanes"]]
            print(label, args.cpu_mask or "default", sizes, "exact all-six-lane Rust/C records with 1–4 workers", flush=True)
            if args.rounds:
                timed("Rust " + label, rust, gold, output, args.rounds)
                timed("C " + label, c, packets, output, args.rounds, payload)

        compare_runtime(stock, seq, serial, td, run_long=True, long_case=compare)

        # A configured E lane is never activated. It must remain empty and
        # must not borrow either other tool's motion or pressure state.
        config = ROOT / "examples/preview"
        script = work / "parked.gcode"
        source = [line for line in long_gcode(2500, True) if line != "T6"]
        script.write_text("\n".join(source) + "\n")
        flags = ["--long-step-preview", "--fixed-pressure-script",
                 "--tool-temps", "0:210,2:210,6:210", config, "sparse-tool.cfg",
                 script, "20000000", "10", "10", ".3", "rails.cfg", "printer.cfg"]
        baseline = json.loads(subprocess.check_output([serial, *flags], timeout=25))
        groups = []
        want, seconds = reference(stock, seq, True, source, native_plans=groups)
        assert math.isclose(baseline["planned_duration_seconds"], seconds, rel_tol=1e-12)
        assert cli_rows(baseline) == want
        assert not baseline["extruder_lanes"][2]["commands"]
        assert all(lane["commands"] for lane in baseline["extruder_lanes"][:2])
        compare("parked-t6", flags, baseline, want, groups)
    subprocess.run(["git", "-C", stock, "diff", "--quiet", "HEAD", "--"], check=True)


if __name__ == "__main__":
    main()
