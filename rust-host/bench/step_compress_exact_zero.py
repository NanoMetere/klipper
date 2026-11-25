#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare stock Klippy debug output with an isolated C helper candidate."""
import argparse
from pathlib import Path
import json
import statistics as stats
import subprocess as sp
import sys
import tempfile
from textwrap import dedent
import time

from hybrid_process_timing import CFG, SHAPED_EXTRUSION_CFG, gcode, motor_trace, child_cpu
from planner import PIN

bench = Path(__file__).resolve().parent
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("upstream", type=lambda path: Path(path).resolve())
p.add_argument("--rounds", type=int, default=24)
p.add_argument("--mode", choices=("exact", "default"), required=True)
p.add_argument("--shadow", choices=("zero-compression", "inline-steppersync"),
               default="zero-compression")
p.add_argument("--json", type=Path)
args = p.parse_args()
upstream, rounds = args.upstream, args.rounds
if not 1 <= rounds <= 80:
    p.error("--rounds must be between 1 and 80")
tip = sp.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
if tip != PIN:
    p.error(f"expected pinned stock checkout {PIN}, got {tip}")
sp.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
source, patch_name = {
    "zero-compression": ("stepcompress.c", "step_compress_exact_zero_c.patch"),
    "inline-steppersync": ("steppersync.c", "steppersync_inline_shadow.patch"),
}[args.shadow]
entry = bench / "hybrid_klippy_entry.py"
dictionary = upstream / "out/klipper.dict"
if not dictionary.is_file():
    p.error("build the pinned Linux test dictionary at out/klipper.dict first")
host = upstream / "klippy/chelper"
sys.path.insert(0, str(upstream / "klippy"))
import chelper
with tempfile.TemporaryDirectory(prefix="klipper-shadow-timing-") as d:
    temp = Path(d)
    here = temp
    with (bench / patch_name).open("rb") as patch:
        sp.run(["patch", "--silent", "--output", here / source, host / source],
               stdin=patch, check=True)
    for arm in ("old", "new"):
        sources = [(here if arm == "new" and name == source else host) / name
                   for name in chelper.SOURCE_FILES]
        cmd = [chelper.GCC_CMD, *chelper.SSE_FLAGS.split(),
               *chelper.COMPILE_ARGS.split()[:-2], str(here / f"debug-{arm}.so"), *map(str, sources), "-I", str(host)]
        sp.run(cmd, check=True)

    wrapper = here / "debug-load.py"
    wrapper.write_text(dedent("""\
        from pathlib import Path
        import runpy
        import sys
        entry, library, stock, mode, *rest = sys.argv[1:]
        sys.path.insert(0, str(Path(entry).parent))
        sys.path.insert(0, str(Path(stock).parent))
        import chelper
        if mode == "exact":
            import stepper
            stepper.MAX_STEPCOMPRESS_ERROR = 0.0
        chelper.check_build_c_library = lambda: library
        sys.argv = [entry, "--drain-debug-output", stock, "--stock-only", *rest]
        runpy.run_path(entry, run_name="__main__")
        """))
    results = {}
    print(f"{args.shadow}, tolerance {args.mode}", flush=True)
    cfg, script, log, out = (temp / x for x in ("printer.cfg", "path.gcode", "host.log", "steps.out"))
    cfg.write_text(CFG + SHAPED_EXTRUSION_CFG)
    for profile, moves in [("layered", 12000), ("shaped-extrusion", 16000)]:
        lines = list(gcode(moves, profile, 0))
        script.write_text("\n".join(lines) + "\n")
        golden, timings = None, [[], []]
        for r in range(-1, rounds):
            for i in (r % 2, 1 - r % 2):
                arm = ("old", "new")[i]
                cpu, wall = child_cpu(), time.perf_counter()
                p = sp.run([sys.executable, wrapper, entry, here / f"debug-{arm}.so",
                            upstream / "klippy/klippy.py", args.mode,
                            cfg, "-i", script, "-o", out, "-d", dictionary, "-l", log],
                           capture_output=True, text=True, timeout=100)
                delta = (child_cpu() - cpu, time.perf_counter() - wall)
                if p.returncode != 0:
                    raise RuntimeError(p.stdout + p.stderr + (log.read_text()[-4000:] if log.exists() else ""))
                drain = next(line for line in p.stdout.splitlines() if line.startswith("FILE_OUTPUT_DRAIN "))
                assert json.loads(drain.split(" ", 1)[1]) == [str(out)]
                trace = motor_trace(upstream, dictionary, out.read_bytes())
                if golden is None:
                    golden = trace
                    assert len(trace) == 4 and all(len(v) > 1 for v in trace.values())
                assert trace == golden, (profile, r, arm)
                if r >= 0:
                    timings[i].append(delta)
            if r in (-1, rounds-1):
                print(profile, r, sum(map(len, golden.values())), "exact", flush=True)
        for col, name in enumerate(("cpu", "elapsed")):
            old, new = zip(*timings[0]), zip(*timings[1])
            old, new = list(old)[col], list(new)[col]
            paired = [b/a for a, b in zip(old, new, strict=True)]
            print(profile, name, f"{stats.median(old)*1000:.2f} -> {stats.median(new)*1000:.2f} ms; paired {(stats.median(paired)-1)*100:+.2f}%, {sum(v<1 for v in paired)}/{rounds} candidate wins", flush=True)
        results[profile] = timings
if args.json:
    args.json.write_text(json.dumps(results) + "\n")
