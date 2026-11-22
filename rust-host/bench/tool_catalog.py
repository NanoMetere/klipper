#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Check static tool catalog inheritance against actual plugin config reads."""
import argparse
import importlib
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

from planner import ROOT
from toolchange_order import TOOLCHANGER


def vectors(stock, plugin):
    sys.path.insert(0, str(stock.resolve()/"klippy"))
    import extras
    extras.__path__.insert(0, str(plugin.resolve()/"klipper/extras"))
    from configfile import ConfigFileReader, ConfigWrapper
    tool = importlib.import_module("extras.tool")
    rng = random.Random(1155)
    axes = lambda word: sum(1 << "xyz".index(c) for c in set(word.lower()))
    created = 0
    for i in range(220):
        used = rng.choice(["x", "XY", "xz", "xyz"])
        restore = rng.choice(["", "Z", "XY", "xYz"])
        require, verify, detect = bool(i % 2), bool(i % 3), bool(i % 4)
        metadata = ["[toolchanger]", f"uses_axis: {used}",
                    f"require_tool_present: {str(require)}", f"verify_tool_pickup: {str(verify)}",
                    f"t_command_restore_axis: {restore}"]
        for axis in "xyz":
            if rng.random() < .7:
                metadata.append(f"gcode_{axis}_offset: {rng.uniform(-2, 2):.4f}")
        used_numbers = rng.sample(range(40), rng.randint(1, 7))
        for number in used_numbers:
            metadata.extend([f"[tool Tool_{number}]", f"tool_number: {number}"])
            if detect:
                metadata.append(f"detection_pin: inert_test_{number}")
            if rng.random() < .75:
                metadata.append(f"t_command_restore_axis: {rng.choice(['', 'X', 'YZ', 'xyz'])}")
            for axis in "xyz":
                if rng.random() < .5:
                    metadata.append(f"gcode_{axis}_offset: {rng.uniform(-3, 3):.4f}")
        metadata.append("[tool unassigned]")
        text = "\n".join(metadata) + "\n"
        raw = ConfigFileReader().build_fileconfig_with_includes(text, "/unused/printer.cfg")
        tc = SimpleNamespace(params={}, assign_tool=lambda *args: None)
        handlers = {
            "gcode_macro": SimpleNamespace(load_template=lambda *args: None),
            "toolchanger": tc,
            "buttons": SimpleNamespace(register_buttons=lambda *args: None),
        }
        gcode = SimpleNamespace(register_command=lambda *args, **kwargs: None,
                                register_mux_command=lambda *args, **kwargs: None)
        printer = SimpleNamespace(load_object=lambda _, name: handlers[name],
                                  lookup_object=lambda name: gcode,
                                  register_event_handler=lambda *args: None)
        tc.config = ConfigWrapper(printer, raw, {}, "toolchanger")
        wrappers = [tool.Tool(ConfigWrapper(printer, raw, {}, s))
                    for s in raw.sections() if s.startswith("tool ")]
        expected = [f"P {axes(used)} {int(not require)} {int(verify and detect)}"]
        for slot in sorted(wrappers, key=lambda a: a.tool_number):
            if slot.tool_number < 0:
                continue
            offsets = " ".join(f"{value:.17f}" for value in slot.get_offset())
            expected.append(f"T {slot.tool_number} {axes(slot.t_command_restore_axis)} "
                            f"{int(slot.detect_state == 'mounted')} {offsets} "
                            f"{slot.name.replace(' ', '_')}")
        created += len(expected)-1
        yield text, expected, created


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("legacy_klipper", type=Path)
    p.add_argument("toolchanger", type=Path, help=TOOLCHANGER)
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/tool_catalog_replay")
    a = p.parse_args()
    revision = subprocess.check_output(["git", "-C", a.toolchanger, "rev-parse", "HEAD"], text=True).strip()
    if revision != TOOLCHANGER:
        p.error("wrong pinned toolchanger")
    inputs, expected = [], []
    for text, outcomes, count in vectors(a.legacy_klipper, a.toolchanger):
        inputs.append(text)
        expected.extend(outcomes)
    actual = subprocess.check_output([a.binary], input="\f".join(inputs), text=True).splitlines()
    for i, (one, other) in enumerate(zip(expected, actual, strict=True)):
        # Rust and Python print signed zero slightly differently after the stock fallback.
        assert one.replace("-0.00000000000000000", "0.00000000000000000") ==             other.replace("-0.00000000000000000", "0.00000000000000000"), (i, one, other)
    print(f"{len(inputs)} configured catalogs and {count} tool metadata records match stock")
if __name__ == "__main__":
    main()
