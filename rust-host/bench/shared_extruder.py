#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify one shared stepper's binding state against pinned, offline Klippy."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from hybrid_klippy import motor_trace
from planner import PIN, ROOT


def compare(binary, inputs, outcomes):
    commands = []
    for row in inputs:
        kind = row["kind"]
        if kind == "sync":
            commands.append(f"sync {row['queue'] or '-'} {row['flushed']!r}")
        elif kind == "move":
            commands.append(f"move {row['queue']} {row['end']!r}")
        elif kind == "activate":
            commands.append(f"activate {row['queue']}")
        elif kind == "advance":
            commands.append(f"advance {row['advance']!r}")
        else:
            raise AssertionError(row)
    # See fixture: 22.17 mm/rev, default 200 full steps, 16 microsteps.
    run = subprocess.run([binary, str(22.17 / 3200), ".04"],
                         input="\n".join(commands) + "\n", text=True,
                         capture_output=True, timeout=15, check=True)
    for line, stock in zip(run.stdout.splitlines(), outcomes, strict=True):
        action, active, bound, p, s, advance, commanded, mcu = line.split()
        assert action == stock["action"]
        assert active == stock["selected"]
        assert bound == (stock["status"]["motion_queue"] or "-")
        assert float(p) == stock["positions"]["extruder"]
        assert float(s) == stock["positions"]["extruder1"]
        assert float(advance) == stock["status"]["pressure_advance"]
        assert stock["status"]["smooth_time"] == .04
        if action == "sync":
            # Only at a flush boundary has Rust been fed a commanded position.
            assert float(commanded) == stock["commanded"], (line, stock)
            assert int(mcu) == stock["mcu_steps"], (line, stock)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=lambda s: Path(s).resolve())
    p.add_argument("--dictionary", type=lambda s: Path(s).resolve())
    a = p.parse_args()
    root = a.upstream
    actual = subprocess.check_output(["git", "-C", root, "rev-parse", "HEAD"], text=True).strip()
    assert actual == PIN, (actual, PIN)
    subprocess.run(["git", "-C", root, "diff", "--quiet", "HEAD", "--"], check=True)
    dictionary = a.dictionary or root / "out/klipper.dict"
    fixture = ROOT / "bench/shared_extruder_fixture"
    with tempfile.TemporaryDirectory(prefix="shared-extruder-offline-") as d:
        temp = Path(d)
        library = temp / "libklipper_host_protocol.rlib"
        binary = temp / "compare"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type", "rlib",
                        "--crate-name", "klipper_host_protocol", ROOT / "src/lib.rs",
                        "-o", library], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/shared_extruder.rs",
                        "--extern", f"klipper_host_protocol={library}", "-o", binary], check=True)
        source = (fixture / "path.gcode").read_text()
        for reverse in (False, True):
            script = temp / "path.gcode"
            script.write_text(source.replace("G1 E-0.37 F1200", "G1 E-3.87 F1200", 1)
                              if reverse else source)
            golden = None
            for i in range(3):
                out, log = temp / f"step{i}.out", temp / f"host{i}.log"
                run = subprocess.run([sys.executable, fixture / "stock_entry.py",
                                      ROOT / "bench/hybrid_klippy_entry.py", root / "klippy/klippy.py",
                                      fixture / "printer.cfg", "-i", script,
                                      "-o", out, "-d", dictionary, "-l", log],
                                     text=True, capture_output=True, timeout=30)
                if run.returncode:
                    raise RuntimeError(run.stdout + run.stderr + (log.read_text() if log.exists() else ""))
                rows = run.stdout.splitlines()
                drain = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("FILE_OUTPUT_DRAIN ")]
                assert drain == [[str(out)]], drain
                def decoded(tag):
                    return [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith(tag)]

                inputs, snapshots = decoded("SHARED_INPUT "), decoded("SHARED_OUTCOME ")
                compare(binary, inputs, snapshots)
                trace = motor_trace(root, dictionary, out.read_bytes())
                if golden is None:
                    golden = inputs, snapshots, trace
                assert (inputs, snapshots, trace) == golden, "stock runs disagree"
            assert [s["status"]["motion_queue"] for s in snapshots] == [
                "extruder", "extruder", "extruder1", "extruder1", "extruder", None, "extruder"]
            assert [s["selected"] for s in snapshots] == [
                "extruder", "extruder", "extruder", "extruder1", "extruder1", "extruder", "extruder"]
            assert [s["status"]["pressure_advance"] for s in snapshots] == [
                .04, .06, .06, .085, .085, .085, .085]
            assert len({s["oid"] for s in snapshots}) == 1 and len(trace) == 4
            eid = f"oid={snapshots[0]['oid']}"
            assert eid in trace and len(trace[eid]) > 20
            assert snapshots[5]["mcu_steps"] == snapshots[6]["mcu_steps"], "detached move generated steps"
            assert snapshots[5]["positions"] != snapshots[6]["positions"]
            assert snapshots[2]["selected"] != snapshots[2]["status"]["motion_queue"]
            assert (snapshots[2]["mcu_steps"] < 0) == reverse
            assert any(s["mcu_steps"] != 0 for s in snapshots)
            assert len({s["mcu_steps"] for s in snapshots}) >= 4
            label = "negative" if reverse else "positive"
            print(f"Shared physical E {eid}, {label} first rebind: {len(snapshots)} Rust/stock binding and gain snapshots, "
                  f"{len(inputs)} committed events; {sum(map(len, trace.values()))} stock-only "
                  f"timed wire records reproduced across three runs ({len(trace[eid])} E records)")


if __name__ == "__main__":
    main()
