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


def run_stock(upstream, dictionary, fixture, temp, path, zero_error):
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
    trace = motor_trace(upstream, dictionary, out.read_bytes())
    assert len(trace) == 4
    return rows, trace


def observe(upstream, dictionary, fixture, temp, path, zero_error, capture_batches=0):
    rows, trace = run_stock(upstream, dictionary, fixture, temp, path, zero_error)
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
        if capture_batches and op == "f":
            events.append(f"w {row['cutoff']}")
    counts = Counter(e[0] for e in events)
    expected = ({"p": 3, "f": capture_batches, "b": 5, "a": 228,
                 "t": 2 * capture_batches, "w": capture_batches} if capture_batches else
                {"p": 3, "f": 10, "b": 5, "a": 12, "t": 20})
    assert paused_trims == 2 and counts == expected, (capture_batches, counts)
    snaps = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("SHARED_OUTCOME ")]
    ids = {snap["oid"] for snap in snaps}
    assert len(ids) == 1
    xyz = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("XYZ ")]
    meta, = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("XYZ_BIND ")]
    assert len(meta["oids"]) == len(set(meta["oids"])) == 3
    for item in xyz:
        assert item["op"] in XYZ_ATTRS
    xyz_inputs = []
    for row in xyz:
        xyz_inputs.append(" ".join(map(str, (row["op"], *(row[k] for k in XYZ_ATTRS[row["op"]].split())))))
        if capture_batches and row["op"] == "f":
            xyz_inputs.append(f"w {row['cutoff']}")
    eid = ids.pop()
    assert eid not in meta["oids"]
    return ("\n".join(events) + "\n", trace[f"oid={eid}"],
            "\n".join(xyz_inputs) + "\n", meta, [trace[f"oid={i}"] for i in meta["oids"]])



def normalized_ops(text):
    """Skip only duplicate/no-time flushes and their idempotent retirement."""
    last_flush = None
    skip = False
    for row in text.splitlines():
        op, *fields = row.split()
        values = tuple(map(float, fields))
        if op == "f":
            skip = values == (0.,) or values == last_flush
            last_flush = values
            if skip:
                continue
        if op == "t" and skip:
            continue
        yield op, values


def same_plan(stock, independent):
    a, b = tuple(normalized_ops(stock)), tuple(normalized_ops(independent))
    for i, (wanted, actual) in enumerate(zip(a, b, strict=True)):
        assert wanted == actual, (i, wanted, actual)
    return len(b)


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


def independent_cases(source, fixture, long_case):
    lines = long_case.splitlines(keepends=True)
    ends = []
    size = 0
    for line in lines:
        size += len(line)
        ends.append(size)
    chosen = max(i for i, end in enumerate(ends) if end <= 4093)
    assert lines[chosen].startswith("G1 ")
    for target in (4095, 4096, 4097):
        # Shift this exact command's newline, preserving all commands.
        padded = lines.copy()
        padded[chosen] = lines[chosen].rstrip("\n") + " ;" + "p" * (target - ends[chosen] - 2) + "\n"
        assert sum(map(len, padded[:chosen+1])) == target
        yield f"byte-{target}", "".join(padded), 29992
    for width in (20, 180):
        padded = "".join(line + " ;" + "c" * width + "\n" for line in long_case.splitlines())
        yield f"comment-{width}", padded, 29992

    def terminal(lines):
        return "".join(lines) + "M400\n"

    wide = terminal(f"G1 X{18+(i%2)*.61:.3f} Y{39+(1-i%2)*.29:.3f} E.006 F1020\n" for i in range(240))
    mixed = terminal(
        f"G1 X{18+(i%2)*.28:.3f} Y{39+(1-i%2)*.67:.3f} Z{1.42+(i%3)*.017:.3f}"
        f" E{.008 if i%11 else -.006} F480\n" for i in range(200)
    )
    waits = terminal(("M400\nG4 P20\n" if i in (30, 73, 181) else "") +
        f"G1 X{18+(i%2)*.24:.3f} Y{39+(1-i%2)*.14:.3f} E.002 F600\n" for i in range(216))
    yield "wide", source.replace("M400\n", wide, 1), 45412
    yield "xyz-with-retract", source.replace("M400\n", mixed, 1), 45442
    yield "absolute-then-relative", (fixture / "absolute_flow.gcode").read_text().replace(
        "M400\n", "M83\n" + wide, 1), 45377
    yield "waits-and-dwell", source.replace("M400\n", waits, 1), 29994


def file_cli_lanes(a, fixture, temp, path, tol, hz):
    config = json.loads((fixture / "offline_model.json").read_text())
    config["max_error_ticks"] = int(tol)
    model = temp / "debug-model.json"
    model.write_text(json.dumps(config))
    raw = subprocess.check_output([a.file_debug_preview, model, path], timeout=10)
    result = json.loads(raw)
    assert result["schema"] == "klipper.shared-corexy.file-debug.v1"
    assert result["lane_order"] == ["corexy_a", "corexy_b", "z", "shared_e"]
    assert result["clock_hz"] == float(hz)
    return [
        [f"d {int(row['positive'])}" if row["kind"] == "direction" else
         "s {interval} {count} {add} {last_clock}".format(**row) for row in lane]
        for lane in result["lanes"]
    ]


def check_independent(a, fixture, temp, integrated, name, script, count, zero_error, hz):
    path = temp / "independent.gcode"
    path.write_text(script)
    rows, trace = run_stock(a.upstream, a.dictionary, fixture, temp, path, zero_error)
    # Actual debug-file runs must also agree without using them as inputs.
    _, repeat = run_stock(a.upstream, a.dictionary, fixture, temp, path, zero_error)
    assert repeat == trace, (name, zero_error)
    meta, = [json.loads(s.split(" ", 1)[1]) for s in rows if s.startswith("XYZ_BIND ")]
    eids = {json.loads(s.split(" ", 1)[1])["oid"] for s in rows if s.startswith("SHARED_OUTCOME ")}
    eid, = eids
    tol = str(meta["max_error"])
    assert int(tol) == (0 if zero_error else int(int(hz) * 0.000025))
    direct = subprocess.check_output([integrated, tol, "--file-debug"], input=script,
                                     text=True, timeout=20).split("end\n")
    assert len(direct) == 5 and direct[-1] == ""
    observed_count = 0
    for oid, lane in zip([*meta["oids"], eid], direct[:-1], strict=True):
        expected, steps = flatten_file(trace[f"oid={oid}"])
        assert expected == wire_records(lane), (name, zero_error, oid)
        observed_count += steps
    assert observed_count == count, (name, zero_error, observed_count)
    if a.file_debug_preview:
        assert file_cli_lanes(a, fixture, temp, path, tol, hz) == [wire_records(l) for l in direct[:-1]]

    print(f"Independent file debug, {name}, {tol} ticks: {count} exact stock steps")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=lambda s: Path(s).resolve())
    p.add_argument("--dictionary", required=True, type=lambda s: Path(s).resolve())
    p.add_argument("--shared-preview", type=lambda s: Path(s).resolve())
    p.add_argument("--file-debug-preview", type=lambda s: Path(s).resolve())
    a = p.parse_args()
    actual = subprocess.check_output(["git", "-C", a.upstream, "rev-parse", "HEAD"], text=True).strip()
    assert actual == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    hz = str(int(json.loads(a.dictionary.read_text())["config"]["CLOCK_FREQ"]))
    fixture = ROOT / "bench/shared_extruder_fixture"
    source = (fixture / "path.gcode").read_text()
    # E and XYZ counts are checked separately even though the absolute-E
    # case traces the same XYZ coordinates as the relative-E fixtures.
    cases = (
        ("relative-positive", source, (579, 113), 1250, (7078, 99)),
        ("relative-negative", source.replace("G1 E-0.37 F1200", "G1 E-3.87 F1200", 1),
         (831, 118), 1755, (7078, 99)),
        ("absolute-flow", (fixture / "absolute_flow.gcode").read_text(),
         (522, 113), 1230, (7080, 99)),
        ("intermediate-waits", source.replace("M83\n", "M400\nM83\n", 1)
         .replace("G1 X28 Y18 E0.83 F900\n", "G1 X28 Y18 E0.83 F900\nM400\nM400\n", 1)
         .replace("G1 Z1.42 F360\n", "G1 Z1.42 F360\nM400\n", 1),
         (578, 113), 1250, (7079, 101)),
        ("bounded-dwell", source.replace("M83\n", "G4 P19\nM83\n", 1)
         .replace("G1 Z1.17 F360\n", "G1 Z1.17 F360\nG4 P37.5\n", 1)
         .replace("M400\n", "G4\nG4 P11\nM400\n", 1),
         (578, 113), 1250, (7080, 100)),
        ("runtime-limits", source.replace("G1 Z1.17 F360\n",
         "M204 S105\nG1 Z1.17 F360\n", 1)
         .replace("G1 X46 Y21 E1.24 F900\n",
         "M204 P155 T210\nG1 X46 Y21 E1.24 F900\n", 1)
         .replace("G1 X70 Y41 E0.83 F840\n",
         "SET_VELOCITY_LIMIT VELOCITY=13 ACCEL=185 SQUARE_CORNER_VELOCITY=3 MINIMUM_CRUISE_RATIO=0.3\n"
         "G1 X70 Y41 E0.83 F840\n", 1)
         .replace("G1 Z1.42 F360\n",
         "SET_VELOCITY_LIMIT VELOCITY=250 ACCEL=4000 SQUARE_CORNER_VELOCITY=5 MINIMUM_CRUISE_RATIO=0.5\n"
         "G1 Z1.42 F360\n", 1),
         (584, 130), 1242, (7612, 206)),
        ("display-metadata", source.replace("M83\n", "M73 P0 R5\nM83\n", 1)
         .replace("G1 X46 Y21 E1.24 F900\n",
         "M117 Layer 1 ; display only\nG1 X46 Y21 E1.24 F900\n", 1)
         .replace("G1 Z1.42 F360\n",
         "SET_DISPLAY_TEXT MSG=\"Shared tool\"\nG1 Z1.42 F360\n", 1)
         .replace("M400\n", "M400\nM73 P100\nSET_DISPLAY_TEXT\nM117\n", 1),
         (579, 113), 1250, (7078, 99)),
        ("saved-coordinate-state", source.replace("G1 X28 Y18 E0.83 F900\n",
         "G1 X28 Y18 E0.83 F900\nSAVE_GCODE_STATE NAME=First\nG91\n", 1)
         .replace("G1 Z1.17 F360\n", "G1 Z0.17 F455\n", 1)
         .replace("G1 X46 Y21 E1.24 F900\n",
         "RESTORE_GCODE_STATE NAME=First MOVE=0\nG1 X46 Y21 E1.24\n", 1)
         .replace("G1 X70 Y41 E0.83 F840\n",
         "SAVE_GCODE_STATE NAME=Active\nG1 X70 Y41 E0.83 F840\n", 1)
         .replace("G1 E-0.28 F1200\n",
         "RESTORE_GCODE_STATE NAME=Active\nG1 E-0.28 F1200\n", 1),
         (579, 113), 1250, (7078, 99)),
        ("restore-move", source.replace("G1 X28 Y18 E0.83 F900\n",
         "G1 X28 Y18 E0.83 F900\nSAVE_GCODE_STATE NAME=First\n", 1)
         .replace("G1 Z1.17 F360\n",
         "G1 Z1.17 F360\nRESTORE_GCODE_STATE NAME=First MOVE=1 MOVE_SPEED=9\n", 1),
         (577, 113), 1250, (7150, 114)),
        ("coordinate-offset", source.replace("G1 X28 Y18 E0.83 F900\n",
         "G1 X28 Y18 E0.83 F900\nSET_GCODE_OFFSET X=0.25 Y=-0.125 Z=0.02\n", 1)
         .replace("G1 X46 Y21 E1.24 F900\n",
         "SET_GCODE_OFFSET Z_ADJUST=0.1 MOVE=1 MOVE_SPEED=7.5\nG1 X46 Y21 E1.24 F900\n", 1)
         .replace("G1 Z1.42 F360\n",
         "SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0\nG1 Z1.42 F360\n", 1),
         (635, 112), 1250, (6830, 98)),
        ("long-short-moves", source.replace("M400\n", "".join(
         f"G1 X{18 + (i % 2)*0.24:.3f} Y{39 + (1 - i % 2)*0.14:.3f} E0.002 F600\n"
         for i in range(216)) + "M400\n", 1),
         (644, 183), 1312, ((5491, 5564, 87), (688, 1119, 25))),
    )
    long_case = cases[-1]
    padded = "".join(line + " ;" + "c" * 80 + "\n" for line in long_case[1].splitlines())
    def commands(text):
        return [line.split(";", 1)[0].rstrip() for line in text.splitlines()]

    assert commands(padded) == commands(long_case[1])
    cases += (("padded-long-short-moves", padded, *long_case[2:]),)
    with tempfile.TemporaryDirectory(prefix="klipper-captured-window-") as d:
        temp = Path(d)
        _, c, rust = build(a.upstream, temp)
        xyz = temp / "xyz"
        lib = temp / "libklipper_host_protocol.rlib"
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/shared_xyz_capture.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", xyz], check=True)
        plan = temp / "plan"
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/shared_independent_plan.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", plan], check=True)
        integrated = temp / "integrated"
        subprocess.run(["rustc", "--edition=2021", "-O", "--cfg", 'feature="file-debug-preview"',
                        ROOT / "bench/shared_integrated_render.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", integrated], check=True)
        model = json.loads((fixture / "offline_model.json").read_text())
        mt, body = temp / "model.json", temp / "body.gcode"
        for zero_error in (True, False):
            mode = 0 if zero_error else 1
            baseline_long_e = None
            for name, script, e_counts, e_ticks, xyz_counts in cases:
                path = temp / "fixture.gcode"
                e_count = e_counts[mode]
                path.write_text(script)
                batches = {"long-short-moves": 36, "padded-long-short-moves": 37}.get(name, 0)
                observed = observe(a.upstream, a.dictionary, fixture, temp, path, zero_error, batches)
                assert observed == observe(a.upstream, a.dictionary, fixture, temp, path, zero_error, batches)
                events, stock_file, xyz_events, meta, xyz_stock_files = observed
                tol = str(meta["max_error"])
                assert int(tol) == (0 if zero_error else int(int(hz) * 0.000025))
                c_out = wire_records(subprocess.check_output([c, hz, tol], input=events, text=True, timeout=15))
                rust_out = wire_records(subprocess.check_output([rust, "--wire", hz, tol],
                                                              input=events, text=True, timeout=15))
                want, physical_steps = flatten_file(stock_file)
                assert c_out == rust_out == want, (name, tol)
                assert (len(want), physical_steps) == (e_count, e_ticks)
                print(f"Actual stock debug, max compress error {tol} ticks, E, case={name}: "
                      f"{len(want)} compressed blocks/directions, "
                      f"{physical_steps} individual ticks, exact C replay and Rust")
                xyz_args = [xyz, hz, tol, *map(str, meta["d"]), *(str(int(v)) for v in meta["inverted"])]
                init, executable = script.split("\n", 1)
                assert init.split(";", 1)[0].rstrip() == "SET_KINEMATIC_POSITION X=10 Y=10 Z=1"
                if batches:
                    direct = subprocess.check_output([integrated, tol, "--file-debug"], input=script,
                                                     text=True, timeout=20)
                    direct_lanes = direct.split("end\n")
                    assert len(direct_lanes) == 5 and direct_lanes[-1] == ""
                    for stock_axis, lane in zip([*xyz_stock_files, stock_file], direct_lanes[:-1], strict=True):
                        expected, _ = flatten_file(stock_axis)
                        assert expected == wire_records(lane), (name, tol, "independent debug")
                    if a.file_debug_preview:
                        assert file_cli_lanes(a, fixture, temp, path, tol, hz) == [
                            wire_records(lane) for lane in direct_lanes[:-1]
                        ]
                    replay = subprocess.check_output(xyz_args, input=xyz_events, text=True, timeout=20)
                    fields = replay.split("end\n")
                    assert len(fields) == 4 and fields[-1] == ""
                    for want_axis, actual, n, records in zip(
                        xyz_stock_files, fields[:-1], (13651, 14861, 168), xyz_counts[mode], strict=True,
                    ):
                        expected, physical = flatten_file(want_axis)
                        assert (len(expected), physical) == (records, n)
                        assert expected == wire_records(actual), (name, tol)
                    if a.shared_preview:
                        model["max_error_ticks"] = int(tol)
                        mt.write_text(json.dumps(model))
                        body.write_text(executable)
                        guarded = subprocess.run([a.shared_preview, mt, body], capture_output=True, timeout=10)
                        assert guarded.returncode == 2 and not guarded.stdout and b"Inputs" in guarded.stderr
                    if batches == 36:
                        baseline_long_e = want
                    else:
                        assert baseline_long_e is not None and want != baseline_long_e
                    print(f"Capture replay only (no independent schedule), {name}, {tol} ticks: "
                          f"{sum(xyz_counts[mode]) + e_count} exact records, 29992 steps")
                    continue
                full = subprocess.check_output([integrated, tol], input=executable, text=True, timeout=15)
                lanes = full.split("end\n")
                assert lanes[-1] == "" and len(lanes) == 5
                if a.shared_preview:
                    model["max_error_ticks"] = int(tol)
                    mt.write_text(json.dumps(model))
                    body.write_text(executable)
                    cli = subprocess.check_output([a.shared_preview, mt, body], text=True, timeout=10)
                    assert cli == subprocess.check_output([a.shared_preview, mt, body],
                                                          text=True, timeout=10)
                    preview = json.loads(cli)
                    assert preview["schema"] == "klipper.shared-corexy.offline.v1"
                    assert preview["lane_order"] == ["corexy_a", "corexy_b", "z", "shared_e"]
                    assert preview["clock_hz"] == float(hz)
                    for c_lane, expected_lane in zip(preview["lanes"], lanes[:-1], strict=True):
                        got = [f"d {int(r['positive'])}" if r["kind"] == "direction" else
                               "s {interval} {count} {add} {last_clock}".format(**r) for r in c_lane]
                        assert got == wire_records(expected_lane), (name, tol)
                for i, stock_axis in enumerate([*xyz_stock_files, stock_file]):
                    expected, _ = flatten_file(stock_axis)
                    assert expected == wire_records(lanes[i]), (name, tol, i)
                independently_planned_e = subprocess.check_output([plan], input=executable, text=True,
                                                                  timeout=10)
                independently_planned_xyz = subprocess.check_output([plan, "--xyz"], input=executable,
                                                                    text=True, timeout=10)
                assert same_plan(events, independently_planned_e) == 38
                expect_xyz_rows = 25 if name in ("restore-move", "coordinate-offset") else 24
                assert same_plan(xyz_events, independently_planned_xyz) == expect_xyz_rows
                plan_e = subprocess.check_output([rust, "--wire", hz, tol], input=independently_planned_e,
                                                 text=True, timeout=15)
                assert wire_records(plan_e) == want
                captured_xyz = subprocess.check_output(xyz_args, input=xyz_events, text=True, timeout=20)
                xyz_rows = subprocess.check_output(xyz_args, input=independently_planned_xyz,
                                                   text=True, timeout=20)
                assert captured_xyz == xyz_rows
                fields = xyz_rows.split("end\n")
                assert fields[-1] == "" and len(fields) == 4
                tick_count = 0
                records = 0
                for stock_axis, candidate in zip(xyz_stock_files, fields[:-1], strict=True):
                    expected, n = flatten_file(stock_axis)
                    assert expected == wire_records(candidate)
                    tick_count += n
                    records += len(expected)
                expect_xyz_ticks = {"restore-move": 20624, "coordinate-offset": 20548}.get(name, 20488)
                assert (records, tick_count) == (xyz_counts[mode], expect_xyz_ticks)
                print(f"Actual stock debug, max compress error {tol} ticks, XYZ, case={name}: "
                      f"{records} compressed blocks/directions, {tick_count} individual ticks, exact independently planned Rust")
            for name, script, count in independent_cases(source, fixture, long_case[1]):
                check_independent(a, fixture, temp, integrated, name, script, count, zero_error, hz)
        if a.shared_preview:
            def rejected(path, why):
                r = subprocess.run([a.shared_preview, mt, path], capture_output=True, timeout=10)
                assert r.returncode == 2 and not r.stdout and why in r.stderr, r.stderr

            model["serial"] = "unsupported"
            mt.write_text(json.dumps(model))
            rejected(body, b"unknown field")
            del model["serial"]
            mt.write_text(json.dumps(model))
            body.write_text("G28\nM400\n")
            rejected(body, b"Offline preview")
            body.write_text("G1 X11\nG4 P-1\nM400\n")
            rejected(body, b"Offline preview")
            body.write_text("G1 X11\nSET_VELOCITY_LIMIT ACCEL=0\nM400\n")
            rejected(body, b"Offline preview")
            body.write_text("G1 X11\nM73 Pnan\nM400\n")
            rejected(body, b"Offline preview")
            body.write_text("G1 X11\nRESTORE_GCODE_STATE NAME=Missing MOVE=1\nM400\n")
            rejected(body, b"Offline preview")
            body.write_text("G1 X11\nSET_GCODE_OFFSET X=1000 MOVE=1\nM400\n")
            rejected(body, b"Offline preview")
            symlink = temp / "symlink.gcode"
            symlink.symlink_to(body)
            rejected(symlink, b"non-symlink")
            body.write_bytes(b" " * 65_537)
            rejected(body, b"exceeds 65536 bytes")
            print("Standalone shared preview matched 20 stock cases; 4 longer captured cases replayed exactly")
        if a.file_debug_preview:
            help_text = subprocess.check_output([a.file_debug_preview, "--help"], text=True, timeout=5)
            assert help_text.startswith("Usage: klipper-file-debug-preview ")
            model_file = temp / "debug-model.json"
            good = temp / "debug-valid.gcode"
            good.write_text(source)
            bad = temp / "debug-invalid.gcode"
            symlink = temp / "debug-symlink.gcode"
            symlink.symlink_to(good)
            rejected = [symlink]
            for data in (b"M400\n", source.encode()+b"\x00", source.rstrip("\n").encode(),
                         b";"+b" "*65536):
                bad.write_bytes(data)
                result = subprocess.run([a.file_debug_preview, model_file, bad], capture_output=True, timeout=5)
                assert result.returncode == 2 and not result.stdout, result.stderr
            for path in rejected:
                result = subprocess.run([a.file_debug_preview, model_file, path], capture_output=True, timeout=5)
                assert result.returncode == 2 and b"non-symlink" in result.stderr and not result.stdout
            print("Standalone opt-in file-debug CLI matched 22 stock cases; invalid files rejected")




if __name__ == "__main__":
    main()
