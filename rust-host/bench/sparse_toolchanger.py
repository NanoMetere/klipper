# SPDX-License-Identifier: GPL-3.0-only
"""Sparse physical tools on distinct offline MCU outputs, without docking."""
import json
from pathlib import Path
import re
import subprocess
import sys

from hybrid_klippy import compare_motor_traces, motor_trace

TOOLS = ((0, "extruder"), (2, "extruder1"), (6, "extruder2"))
CHECK = """[gcode_macro _SPARSE_ASSERT]
gcode:
    {% set n = params.T|int %}
    {% set saved_e = params.E|float %}
    {% set e = printer.gcode_move %}
    {% if printer.toolchanger.tool_numbers != [0,2,6]
       or printer.toolchanger.tool_number != n
       or printer.toolhead.extruder != params.NAME|lower
       or (e.position.e - saved_e)|abs > 0.000001
       or e.gcode_position.e|abs > 0.000001 %}
        { action_raise_error('Sparse tool identity or E rebasing mismatch: %s %s' % (printer.toolhead.extruder, e.position.e)) }
    {% endif %}
    { action_respond_info('SPARSE_T%d_%s_%.6f' % (n, printer.toolhead.extruder, e.position.e)) }
[gcode_macro _SPARSE_ASSERT_HEATERS]
gcode:
    {% if printer.toolchanger.tool_number != 0 or printer.extruder.target != 205
          or printer.extruder1.target != 217 or printer.extruder2.target != 228 %}
        { action_raise_error('Sparse heater routing mismatch') }
    {% endif %}
    { action_respond_info('SPARSE_HEATERS_OK') }
"""


def run_sparse(checkout, dictionary, previous, out, install_tree, bridge=None):
    installed = install_tree(out / "config", simulated=True)
    folder = installed / "toolchanger"
    for n, name in TOOLS[1:]:
        path = folder / f"10-T{n}.cfg"
        data, count = re.subn(rf"\bextruder{n}\b", name, path.read_text())
        assert count == 4, (n, count)
        path.write_text(data)
    selection = "\n".join(f"[include {folder / file}]" for file in
                          ["00-toolchanger.cfg", *(f"10-T{n}.cfg" for n, _ in TOOLS)])
    source = (previous / "printer.cfg").read_text()
    original = f"[include {previous / 'config/toolchanger/tools_2.cfg'}]"
    assert source.count(original) == 1
    source = source.replace(original, selection).replace(str(previous), str(out))
    cfg, gcode = out / "printer.cfg", out / "sparse.gcode"
    cfg.write_text(source + CHECK)
    entries = ((0, "extruder", 0., .4637), (2, "extruder1", 0., .23185),
               (6, "extruder2", 0., .69555), (2, "extruder1", .23185, -.115925))
    names = []
    script = ["_INIT_CHECK_TOOLCHANGER", "SET_KINEMATIC_POSITION X=20 Y=20 Z=10", "M83",
              "INITIALIZE_TOOLCHANGER T=0", "M104 S205", "M104 T2 S217", "M104 T6 S228",
              "_SPARSE_ASSERT_HEATERS"]
    for j, (n, name, e, delta) in enumerate(entries):
        script.extend([f"INITIALIZE_TOOLCHANGER T={n}", f"_SPARSE_ASSERT T={n} NAME={name} E={e}",
                       f"G1 X{21+j} Y{20+j%2} E{delta} F300", "M400"])
        names.append(f"SPARSE_T{n}_{name}_{e:.6f}")
    gcode.write_text("\n".join(script) + "\n")
    klippy = checkout / "klippy/klippy.py"
    entry = Path(__file__).resolve().with_name("hybrid_klippy_entry.py")

    def run(label, lib=None):
        wire = out / f"{label}.bin"
        program = [entry, klippy, lib] if lib else [klippy]
        args = [cfg, "-i", gcode, "-o", wire, "-d", dictionary]
        for n, _ in TOOLS:
            args += ["-d", f"tool{n}={dictionary}"]
        proc = subprocess.run([sys.executable, *program, *args], cwd=checkout,
                              text=True, capture_output=True, timeout=60)
        return proc.returncode, proc.stdout + proc.stderr, wire

    def successful(label, lib=None):
        rc, log, wire = run(label, lib)
        clean = not re.search(r"(?m)(^!!|Traceback|Unknown command|Internal error)", log)
        observations = re.findall(r"(?m)^(?:INFO:root:)?(SPARSE_T[^\n]+)$", log)
        if rc or not clean or observations != names or log.count("SPARSE_HEATERS_OK") != 1:
            raise AssertionError((label, rc, observations, log[-6500:]))
        if lib:
            stats = next(line for line in log.splitlines() if line.startswith("RUST_LOOKAHEAD_STATS "))
            stats = json.loads(stats.split(" ", 1)[1])
            assert stats["planned"] >= 4 and stats["fallbacks"] == 0, stats
        return [wire.read_bytes(), *((out / f"{wire.name}-tool{n}").read_bytes() for n, _ in TOOLS)]

    expected = [(640, 0), (320, 160), (960, 0)]
    stock = successful("stock")
    for (n, _), data, target in zip(TOOLS, stock[1:], expected, strict=True):
        rows = motor_trace(checkout, dictionary, data)
        assert len(rows) == 1, (n, rows.keys())
        (trace,) = rows.values()
        direction, counts = None, [0, 0]
        for line in trace:
            if line.startswith("set_next_step_dir"):
                direction = int(line.split("dir=", 1)[1])
            elif line.startswith("queue_step"):
                counts[1 - direction] += int(re.search(r"\bcount=(\d+)", line)[1])
        assert tuple(counts) == target, (n, target, counts)
    hybrid_count = 0
    if bridge:
        hybrid = successful("hybrid", bridge)
        hybrid_count = sum(compare_motor_traces(checkout, dictionary, pair)
                           for pair in zip(stock, hybrid, strict=True))
    # Break the first numbered extruder and all references to it together.
    # Detection now fails because the stock loader does not skip the gap.
    path = folder / "10-T2.cfg"
    path.write_text(re.sub(r"\bextruder1\b", "extruder3", path.read_text()))
    rc, log, _ = run("missing-extruder1")
    if "SPARSE_T" in log or "extruder3" not in log or not re.search(
        r"(?i)(not valid|unknown|not configured|not found|unable to|config error)", log
    ):
        raise AssertionError((rc, log[-6500:]))
    print(f"Sparse T0/T2/T6/T2: {expected} forward/reverse steps on isolated MCU outputs; "
          f"sparse heater targets verified; missing [extruder1] refused; "
          f"{hybrid_count} stock/hybrid wire records matched")
