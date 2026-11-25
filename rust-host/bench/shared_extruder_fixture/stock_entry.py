# SPDX-License-Identifier: GPL-3.0-only
"""Observe committed planner inputs and binding outcomes in stock debug mode."""
import json
from pathlib import Path
import runpy
import sys

entry, stock, *args = sys.argv[1:]
sys.path[:0] = [str(Path(stock).parent), str(Path(entry).parent)]
from kinematics.extruder import ExtruderStepper, PrinterExtruder


def emit(tag, **row):
    print(tag + " " + json.dumps(row), flush=True)


def snapshot(value, action):
    toolhead = value.printer.lookup_object("toolhead")
    now = value.printer.get_reactor().monotonic()
    emit("SHARED_OUTCOME", action=action, oid=value.stepper.get_oid(),
         status=value.get_status(now), selected=toolhead.get_extruder().get_name(),
         commanded=value.stepper.get_commanded_position(),
         mcu_steps=value.stepper.get_mcu_position(),
         positions={name: value.printer.lookup_object(name).last_position
                    for name in ("extruder", "extruder1")})


original_sync = ExtruderStepper.sync_to_extruder


def capture_sync(value, name):
    if value.name != "shared_extruder":
        return original_sync(value, name)
    stepper = value.stepper
    set_position = stepper.set_position
    calls = []

    def before_rebase(coord):
        calls.append((stepper.get_commanded_position(), stepper.get_mcu_position()))
        set_position(coord)

    # Stock flushes inside sync_to_extruder before set_position.
    stepper.set_position = before_rebase
    try:
        original_sync(value, name)
    finally:
        del stepper.set_position
    assert len(calls) == bool(name)
    # Detach changes trapq only, not commanded position.
    if calls:
        flushed, old_mcu_steps = calls[0]
        assert stepper.get_mcu_position() == old_mcu_steps
    else:
        flushed = stepper.get_commanded_position()
    emit("SHARED_INPUT", kind="sync", queue=name or None, flushed=flushed)
    snapshot(value, "sync")


original_pa = ExtruderStepper.cmd_SET_PRESSURE_ADVANCE


def capture_pa(value, command):
    original_pa(value, command)
    if value.name == "shared_extruder":
        emit("SHARED_INPUT", kind="advance", advance=command.get_float("ADVANCE"))
        snapshot(value, "advance")


original_commit = PrinterExtruder.process_move


def capture_commit(value, print_time, move, axis):
    # Use the planner's accepted end, not the derived last_position status.
    original_commit(value, print_time, move, axis)
    emit("SHARED_INPUT", kind="move", queue=value.get_name(), end=move.end_pos[axis])


original_activation = PrinterExtruder.cmd_ACTIVATE_EXTRUDER


def capture_activation(value, command):
    original_activation(value, command)
    emit("SHARED_INPUT", kind="activate", queue=value.get_name())


ExtruderStepper.sync_to_extruder = capture_sync
ExtruderStepper.cmd_SET_PRESSURE_ADVANCE = capture_pa
PrinterExtruder.process_move = capture_commit
PrinterExtruder.cmd_ACTIVATE_EXTRUDER = capture_activation
sys.argv = [entry, "--drain-debug-output", stock, "--stock-only", *args]
runpy.run_path(entry, run_name="__main__")
