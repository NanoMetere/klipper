# SPDX-License-Identifier: GPL-3.0-only
"""Opt-in Klippy adapter for the Rust junction solver; Python retains I/O."""
import struct
import ctypes
import threading

_FIELDS = ("max_start_v2", "max_cruise_v2", "delta_v2",
           "max_mcr_start_v2", "mcr_delta_v2")


def install(toolhead, library):
    """Install for this process and return per-call instrumentation.

    This is a development/virtual fixture opt-in, not a deployment path.
    Do not use this adapter on an unverified stock revision or a printer.
    """
    old = toolhead.LookAheadQueue.flush
    if getattr(old, "_klipper_rust_bridge", False):
        raise ValueError("Rust look-ahead already installed")
    if not set(_FIELDS) <= set(old.__code__.co_names):
        raise ValueError("Unrecognized stock LookAheadQueue.flush implementation")
    native = ctypes.CDLL(str(library))
    plan = native.klipper_rs_lookahead_plan_v1
    plan.restype = ctypes.c_int
    base = ctypes.POINTER(ctypes.c_double)
    plan.argtypes = (base, base, ctypes.c_size_t, ctypes.c_uint8)
    workspace = threading.local()
    row, triple = struct.Struct("=5d"), struct.Struct("=3d")
    pack = row.pack_into
    input_stride, output_stride = row.size, triple.size
    stats = {"calls": 0, "fallbacks": 0, "planned": 0}

    def flush(self, lazy=False):
        queue = self.queue
        count = len(queue)
        if not 1 <= count <= 4096:
            return old(self, lazy)
        if getattr(workspace, "busy", False):
            return old(self, lazy)
        workspace.busy = True
        try:
            buffers = getattr(workspace, "buffers", None)
            if buffers is None:
                inputs = (ctypes.c_double * (5 * 4096))()
                output = (ctypes.c_double * (3 * 4096))()
                buffers = workspace.buffers = (inputs, output, memoryview(output).cast("B"))
            inputs, output, output_bytes = buffers
            stats["calls"] += 1
            try:
                for i, m in enumerate(queue):
                    pack(inputs, i * input_stride, m.max_start_v2, m.max_cruise_v2,
                         m.delta_v2, m.max_mcr_start_v2, m.mcr_delta_v2)
                total = plan(inputs, output, count, bool(lazy))
            except (AttributeError, TypeError, ValueError, OverflowError, struct.error):
                total = -1
            if not 0 <= total <= count:
                stats["fallbacks"] += 1
                return old(self, lazy)
            self.junction_flush = toolhead.LOOKAHEAD_FLUSH_TIME
            result = queue[:total]
            for move, j in zip(result, triple.iter_unpack(output_bytes[:total * output_stride]),
                               strict=True):
                move.set_junction(*j)
            del queue[:total]
            stats["planned"] += total
            return result
        finally:
            workspace.busy = False

    flush._klipper_rust_bridge = True
    toolhead.LookAheadQueue.flush = flush
    return stats
