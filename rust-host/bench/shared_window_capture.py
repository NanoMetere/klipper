#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Replay actual pinned stock debug-file XYZ and shared E timing."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from hybrid_klippy import motor_trace
from planner import PIN, ROOT
from shared_windows import build, wire_records


ATTRS = {
    "a": "q pa at ta tc td x sv cv accel",
    "p": "at gain",
    "b": "q x",
    "f": "at",
    "t": "q cutoff clear",
}


XYZ_ATTRS = {"a": "at ta tc td x y z rx ry rz sv cv acc", "b": "x y z",
             "t": "cutoff", "f": "at"}


def observe(upstream, dictionary, fixture, temp, path, zero_error):
    out, log = temp / "steps.out", temp / "klippy.log"
    run = subprocess.run(
        [sys.executable, fixture / "stock_entry.py",
         "--capture-windows" if zero_error else "--capture-default-windows",
         ROOT / "bench/hybrid_klippy_entry.py", upstream / "klippy/klippy.py",
         fixture / "printer.cfg", "-i", path, "-o", out, "-d", dictionary, "-l", log],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
    if run.returncode:
        raise RuntimeError(run.stdout + run.stderr + (log.read_text() if log.exists() else ""))
    rows = run.stdout.splitlines()
    drain = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("FILE_OUTPUT_DRAIN ")]
    assert drain == [[str(out)]], drain
    windows = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("WINDOW ")]
    assert len(windows) > 30
    q = 0
    events = []
    pressure_ready = False
    paused_trims = 0
    for row in windows:
        op = row["op"]
        if op == "p":
            assert row["smooth"] == .04
            pressure_ready = True
        if not pressure_ready:  # Startup empty queues and no active shaping.
            assert op in ("t", "f")
            continue
        if op == "b":
            q = row["q"]
        if op == "t" and q == -1:
            paused_trims += 1
        events.append(" ".join(map(str, (op, *(row[k] for k in ATTRS[op].split())))))
    assert paused_trims == 2 and Counter(e[0] for e in events) == {
        "p": 3, "f": 10, "b": 5, "a": 12, "t": 20}
    trace = motor_trace(upstream, dictionary, out.read_bytes())
    assert len(trace) == 4
    snaps = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("SHARED_OUTCOME ")]
    ids = {snap["oid"] for snap in snaps}
    assert len(ids) == 1
    xyz = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("XYZ ")]
    meta, = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("XYZ_BIND ")]
    assert len(meta["oids"]) == len(set(meta["oids"])) == 3
    for item in xyz:
        assert item["op"] in XYZ_ATTRS
    xyz_inputs = [" ".join(map(str, (row["op"], *(row[k] for k in XYZ_ATTRS[row["op"]].split()))))
                  for row in xyz]
    eid = ids.pop()
    assert eid not in meta["oids"]
    return ("\n".join(events) + "\n", trace[f"oid={eid}"],
            "\n".join(xyz_inputs) + "\n", meta, [trace[f"oid={i}"] for i in meta["oids"]])



def flatten_file(rows):
    clock = 0
    packets = []
    expanded = 0
    for line in rows:
        tag, _, *fields = line.split()
        args = {k: int(v) for k,v in (item.split("=", 1) for item in fields)}
        if tag == "set_next_step_dir":
            packets.append(f"d {args['dir']}")
        elif tag == "queue_step":
            n, delta, add = args["count"], args["interval"], args["add"]
            expanded += n
            clock += n * delta + add * n * (n - 1) // 2
            packets.append(f"s {delta} {n} {add} {clock}")
        else:
            raise AssertionError(f"unexpected clock reset in offline fixture: {line}")
    return packets, expanded


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=lambda s: Path(s).resolve())
    p.add_argument("--dictionary", required=True, type=lambda s: Path(s).resolve())
    a = p.parse_args()
    actual = subprocess.check_output(["git", "-C", a.upstream, "rev-parse", "HEAD"], text=True).strip()
    assert actual == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    hz = str(int(json.loads(a.dictionary.read_text())["config"]["CLOCK_FREQ"]))
    fixture = ROOT / "bench/shared_extruder_fixture"
    source = (fixture / "path.gcode").read_text()
    with tempfile.TemporaryDirectory(prefix="klipper-captured-window-") as d:
        temp = Path(d)
        _, c, rust = build(a.upstream, temp)
        xyz = temp / "xyz"
        lib = temp / "libklipper_host_protocol.rlib"
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/shared_xyz_capture.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", xyz], check=True)
        for zero_error in (True, False):
            for negative, e_count, e_ticks in ((False, 579 if zero_error else 113, 1250),
                                                      (True, 831 if zero_error else 118, 1755)):
                path = temp / "fixture.gcode"
                script = source.replace("G1 E-0.37 F1200", "G1 E-3.87 F1200", 1) if negative else source
                path.write_text(script)
                observed = observe(a.upstream, a.dictionary, fixture, temp, path, zero_error)
                assert observed == observe(a.upstream, a.dictionary, fixture, temp, path, zero_error)
                events, stock_file, xyz_events, meta, xyz_stock_files = observed
                tol = str(meta["max_error"])
                assert int(tol) == (0 if zero_error else int(int(hz) * 0.000025))
                c_out = wire_records(subprocess.check_output([c, hz, tol], input=events, text=True, timeout=15))
                rust_out = wire_records(subprocess.check_output([rust, "--wire", hz, tol],
                                                              input=events, text=True, timeout=15))
                want, physical_steps = flatten_file(stock_file)
                assert c_out == rust_out == want
                assert (len(want), physical_steps) == (e_count, e_ticks)
                print(f"Actual stock debug, max compress error {tol} ticks, E, negative={negative}: "
                      f"{len(want)} compressed blocks/directions, "
                      f"{physical_steps} individual ticks, exact C replay and Rust")
                xyz_args = [xyz, hz, tol, *map(str, meta["d"]), *(str(int(v)) for v in meta["inverted"])]
                xyz_rows = subprocess.check_output(xyz_args, input=xyz_events, text=True, timeout=20)
                fields = xyz_rows.split("end\n")
                assert fields[-1] == "" and len(fields) == 4
                tick_count = 0
                records = 0
                for stock_axis, candidate in zip(xyz_stock_files, fields[:-1], strict=True):
                    expected, n = flatten_file(stock_axis)
                    assert expected == wire_records(candidate)
                    tick_count += n
                    records += len(expected)
                assert (records, tick_count) == (7078 if zero_error else 99, 20488)
                print(f"Actual stock debug, max compress error {tol} ticks, XYZ, negative={negative}: "
                      f"{records} compressed blocks/directions, {tick_count} individual ticks, exact Rust")


if __name__ == "__main__":
    main()
