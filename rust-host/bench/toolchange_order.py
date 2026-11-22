#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run actual toolchanger selection control flow against inert callbacks."""
import argparse
import importlib
import itertools
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from planner import ROOT

TOOLCHANGER = "94756dfde9b729fd69f9b8780067821c5c99a528"


class GcodeError(Exception):
    pass


def fixture(module, before, target, detect, verify):
    seen = []
    tools = {i: SimpleNamespace(
        tool_number=i, name=f"tool{i}", before_change_gcode=f"before:{i}",
        after_change_gcode=f"after:{i}", dropoff_gcode=f"drop:{i}", pickup_gcode=f"pickup:{i}"
    ) for i in (0, 2, 5)}
    c = module.Toolchanger.__new__(module.Toolchanger)
    c.status = module.STATUS_READY
    c.active_tool = tools.get(before)
    c.has_detection = detect
    c.verify_tool_pickup = verify
    c.next_change_id = 1
    c.error_message = ""
    c.last_change_gcode_position = [0.] * 4
    c.default_before_change_gcode = "before:-"
    c.printer = SimpleNamespace(
        lookup_object=lambda name: SimpleNamespace(wait_moves=lambda: None),
        get_reactor=lambda: SimpleNamespace(monotonic=lambda: 0., pause=lambda limit: None),
    )
    c.tool_missing_helper = SimpleNamespace(
        activate=lambda tool: seen.append(f"watch:{tool.tool_number}"),
        deactivate=lambda: seen.append("suspend"))
    c.ensure_homed = lambda gcmd: None
    c._save_state = lambda axis, tool: seen.append("save")
    c._set_toolchange_transform = lambda: seen.append("dock")
    c._position_with_tool_offset = lambda state, tool: state
    c._position_to_xyz = lambda state, axes: [state["XYZ".index(axis.upper())] for axis in axes]
    c.run_gcode = lambda label, content, context: seen.append(content)
    def configure(tool):
        c.active_tool = tool
        seen.append(f"configure:{'-' if tool is None else tool.tool_number}")
    c._configure_toolhead_for_tool = configure
    c.validate_detected_tool = lambda tool, **kwargs: seen.append(f"verify:{tool.tool_number}")
    c._restore_state_and_transform = lambda tool: seen.append("restore")
    gcmd = SimpleNamespace(error=GcodeError, respond_info=lambda message: None)
    c.select_tool(gcmd, tools.get(target), "XYZ")
    assert c.status == module.STATUS_READY
    seen.append(f"active:{'-' if c.active_tool is None else c.active_tool.tool_number}")
    return seen


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help="stock legacy Klippy checkout compatible with the plugin")
    p.add_argument("toolchanger", type=Path, help=f"checkout of viesturz/klipper-toolchanger at {TOOLCHANGER}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/toolchange_replay")
    a = p.parse_args()
    plugin = a.toolchanger.resolve()
    got = subprocess.check_output(["git", "-C", plugin, "rev-parse", "HEAD"], text=True).strip()
    if got != TOOLCHANGER:
        p.error(f"toolchanger must be pinned at {TOOLCHANGER}")
    sys.path.insert(0, str(a.upstream.resolve()/"klippy"))
    import extras
    extras.__path__.insert(0, str(plugin/"klipper/extras"))
    module = importlib.import_module("extras.toolchanger")
    records, wants = [], []
    value = lambda i: "-" if i is None else str(i)
    for (before, target, detect, verify) in itertools.product([None, 0, 2, 5], [None, 0, 2, 5],
                                                             [False, True], [False, True]):
        records.append(f"{value(before)} {value(target)} {int(detect and verify)}")
        wants.append(fixture(module, before, target, detect, verify))
    out = subprocess.check_output([a.binary], text=True, input="\n".join(records)+"\n").splitlines()
    for i, (want, row) in enumerate(zip(wants, out, strict=True)):
        assert want == row.split(), (i, records[i], want, row)
    print(f"{len(out)} selection/no-op cases and {sum(len(w) for w in wants)} ordered observations match")


if __name__ == "__main__":
    main()
