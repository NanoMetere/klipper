#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline preview binaries, including startup, planning and JSON output."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import tempfile
import time

from planner import ROOT


def gcode(count, tools):
    yield "G90"
    if tools:
        yield "M83"
    for i in range(count):
        if tools and i and i % 80 == 0:
            yield f"T{[0, 2, 6][i // 80 % 3]}"
        x = 10.1 + (i % 17) * .026
        y = 10.1 + (i % 13) * .033
        z = .3 + (i // 120 % 3) * .005
        e = (" E-.01" if i % 12 == 0 else " E.010") if tools else ""
        yield f"G1 X{x:.5f} Y{y:.5f} Z{z:.5f}{e} F{600 + (i % 7) * 130}"
        if i % 80 == 79:
            yield "M400"
    yield "M400"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("baseline", type=Path)
    p.add_argument("candidate", type=Path)
    p.add_argument("--rounds", type=int, default=30)
    p.add_argument("--moves", type=int, default=1500)
    p.add_argument("--tools", action="store_true")
    p.add_argument("--candidate-workers", type=int, help="pass this worker count only to the candidate")
    a = p.parse_args()
    if not 1 <= a.rounds <= 200 or not 500 <= a.moves <= 3200:
        p.error("--rounds must be 1–200 and --moves must be 500–3,200")
    if a.candidate_workers is not None and (not a.tools or not 1 <= a.candidate_workers <= 8):
        p.error("--candidate-workers must be 1–8 and requires --tools")
    cfg = ROOT / "examples/preview"
    programs = {"baseline": a.baseline.resolve(), "candidate": a.candidate.resolve()}
    observations = {name: [] for name in programs}
    with tempfile.TemporaryDirectory(prefix="preview-cli-timing-") as d:
        script = Path(d) / "path.gcode"
        script.write_text("\n".join(gcode(a.moves, a.tools)) + "\n")
        flags = ["--tool-temps", "0:210,2:210,6:210"] if a.tools else []
        config = "sparse-tool.cfg" if a.tools else "printer.cfg"
        includes = ["printer.cfg"] if a.tools else []
        args = [*flags, cfg, config, script, "20000000",  "10", "10", ".3", "rails.cfg", *includes]
        digest = None
        for r in range(a.rounds + 1):
            for label in tuple(programs)[::(1 if r % 2 == 0 else -1)]:
                with tempfile.TemporaryFile() as out, tempfile.TemporaryFile() as err:
                    started = time.perf_counter()
                    worker_flag = (["--lane-workers", str(a.candidate_workers)]
                                   if label == "candidate" and a.candidate_workers else [])
                    with subprocess.Popen([programs[label], *worker_flag, *args], stdout=out, stderr=err) as child:
                        _, status, usage = os.wait4(child.pid, 0)
                        child.returncode = os.waitstatus_to_exitcode(status)
                    wall = time.perf_counter() - started
                    err.seek(0)
                    if child.returncode:
                        raise RuntimeError(err.read().decode("utf-8", "replace"))
                    out.seek(0)
                    data = out.read()
                    actual = hashlib.sha256(data).hexdigest()
                    if digest is None:
                        digest = actual
                        parsed = json.loads(data)
                        count = sum(map(len, parsed["motors"])) + sum(
                            len(t["commands"]) for t in parsed.get("extruder_lanes", [])
                        )
                        print(f"{a.moves} input moves; {count} records; {len(data)} bytes; SHA-256 {digest}")
                    assert actual == digest, (r, label, actual, digest)
                    if r:
                        observations[label].append((usage.ru_utime + usage.ru_stime, wall, usage.ru_maxrss))
        for label, values in observations.items():
            m = [statistics.median(col) for col in zip(*values)]
            print(f"{label}: {m[0]*1000:.3f} ms median CPU; {m[1]*1000:.3f} ms median elapsed; "
                  f"{m[2]:.0f} KiB median peak RSS (Linux)")
        for i, label in enumerate(("CPU", "elapsed", "RSS")):
            v = [new[i]/old[i] for old, new in zip(*observations.values(), strict=True)]
            print(f"{label} paired median change {(statistics.median(v)-1)*100:+.1f}% "
                  f"({sum(x<1 for x in v)}/{len(v)} candidate wins)")


if __name__ == "__main__":
    main()
