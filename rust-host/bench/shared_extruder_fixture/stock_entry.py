# SPDX-License-Identifier: GPL-3.0-only
"""Observe committed planner inputs and binding outcomes in stock debug mode."""
import json
from pathlib import Path
import runpy
import sys

capture_windows = sys.argv[1] in ("--capture-windows", "--capture-default-windows")
zero_error = sys.argv[1] == "--capture-windows"
entry, stock, *args = sys.argv[2 if capture_windows else 1:]
sys.path[:0] = [str(Path(stock).parent), str(Path(entry).parent)]
from kinematics.extruder import ExtruderStepper, PrinterExtruder
from extras.motion_queuing import PrinterMotionQueuing
from toolhead import ToolHead
import stepper as stepper_module
if zero_error:
    # Exact debug-output diagnostic only: this is a new file-output process.
    stepper_module.MAX_STEPCOMPRESS_ERROR = 0.0


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
    if capture_windows:
        # The logical end is an input, not the commanded result of the rebase.
        target = value.printer.lookup_object(name).last_position if name else 0.0
        emit("WINDOW", op="b", q={None: -1, "extruder": 0, "extruder1": 1}[name or None],
             x=target)
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
    if capture_windows:
        ratio = move.axes_r[axis]
        emit("WINDOW", op="a", q={"extruder":0,"extruder1":1}[value.get_name()],
             pa=int(bool(ratio > 0. and (move.axes_d[0] or move.axes_d[1]))),
             at=print_time, ta=move.accel_t, tc=move.cruise_t, td=move.decel_t,
             x=move.start_pos[axis], sv=move.start_v*ratio, cv=move.cruise_v*ratio,
             accel=move.accel*ratio)


original_activation = PrinterExtruder.cmd_ACTIVATE_EXTRUDER


def capture_activation(value, command):
    original_activation(value, command)
    emit("SHARED_INPUT", kind="activate", queue=value.get_name())



old_mq_init = PrinterMotionQueuing.__init__
def setup_mq(self, config):
    old_mq_init(self, config)
    generate = self.steppersyncmgr_gen_steps
    def capture(*args):
        emit("WINDOW", op="f", cutoff=args[1], at=args[2], history=args[3])
        emit("XYZ", op="f", at=args[2], cutoff=args[1])
        return generate(*args)
    self.steppersyncmgr_gen_steps = capture
    trim = self.trapq_finalize_moves
    def capture_trim(trapq, cutoff, clear):
        th = self.printer.lookup_object("toolhead", None)
        if th is not None and trapq == th.get_trapq():
            emit("XYZ", op="t", cutoff=cutoff)
        for i, name in enumerate(("extruder", "extruder1")):
            obj = self.printer.lookup_object(name, None)
            if obj is not None and trapq == obj.get_trapq():
                emit("WINDOW", op="t", q=i, cutoff=cutoff, clear=clear)
        return trim(trapq, cutoff, clear)
    self.trapq_finalize_moves = capture_trim

old_apply_gain = ExtruderStepper._set_pressure_advance
def capture_gain(self, gain, smooth):
    if self.name != "shared_extruder":
        return old_apply_gain(self, gain, smooth)
    th = self.printer.lookup_object("toolhead")
    old_register = th.register_lookahead_callback
    registered = []
    def registration(callback):
        registered.append(True)
        def apply(at):
            emit("WINDOW", op="p", at=at, gain=gain, smooth=smooth)
            callback(at)
        old_register(apply)
    th.register_lookahead_callback = registration
    try:
        old_apply_gain(self, gain, smooth)
    finally:
        del th.register_lookahead_callback
    if not registered:
        emit("WINDOW", op="p", at=0., gain=gain, smooth=smooth)

old_toolhead = ToolHead.__init__
def setup_toolhead(self, config):
    old_toolhead(self, config)
    append = self.trapq_append
    def capture_append(q, at, ta, tc, td, x, y, z, rx, ry, rz, sv, cv, acc):
        emit("XYZ", op="a", at=at, ta=ta, tc=tc, td=td,
             x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, sv=sv, cv=cv, acc=acc)
        return append(q, at, ta, tc, td, x, y, z, rx, ry, rz, sv, cv, acc)
    self.trapq_append = capture_append

old_set_position = ToolHead.set_position
def capture_start(self, newpos, homing_axes=""):
    old_set_position(self, newpos, homing_axes)
    motors = self.get_kinematics().get_steppers()
    emit("XYZ_BIND", oids=[m.get_oid() for m in motors],
         d=[m.get_step_dist() for m in motors],
         inverted=[m.get_dir_inverted()[0] for m in motors],
         max_error=motors[0].get_mcu().seconds_to_clock(stepper_module.MAX_STEPCOMPRESS_ERROR))
    emit("XYZ", op="b", x=newpos[0], y=newpos[1], z=newpos[2])

if capture_windows:
    PrinterMotionQueuing.__init__ = setup_mq
    ToolHead.__init__ = setup_toolhead
    ToolHead.set_position = capture_start
    ExtruderStepper._set_pressure_advance = capture_gain
ExtruderStepper.sync_to_extruder = capture_sync
ExtruderStepper.cmd_SET_PRESSURE_ADVANCE = capture_pa
PrinterExtruder.process_move = capture_commit
PrinterExtruder.cmd_ACTIVATE_EXTRUDER = capture_activation
sys.argv = [entry, "--drain-debug-output", stock, "--stock-only", *args]
runpy.run_path(entry, run_name="__main__")
