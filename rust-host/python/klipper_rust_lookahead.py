# SPDX-License-Identifier: GPL-3.0-only
"""Opt-in Klippy adapter for the Rust junction solver; Python retains I/O."""
import array
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
    stats = {"calls": 0, "fallbacks": 0, "planned": 0}

    def flush(self, lazy=False):
        queue = self.queue
        count = len(queue)
        if not 1 <= count <= 4096:
            return old(self, lazy)
        # CPython converts the flat list to doubles in one array operation.
        bits = array.array("d", [
            f for m in queue for f in (m.max_start_v2, m.max_cruise_v2, m.delta_v2,
                                      m.max_mcr_start_v2, m.mcr_delta_v2)
        ])
        inputs = (ctypes.c_double * (5 * count)).from_buffer(bits)
        output = getattr(workspace, "output", None)
        if output is None:
            output = workspace.output = (ctypes.c_double * (3 * 4096))()
        try:
            total = plan(inputs, output, count, int(bool(lazy)))
        except (TypeError, ValueError, OverflowError):
            total = -1
        stats["calls"] += 1
        if not 0 <= total <= count:
            stats["fallbacks"] += 1
            return old(self, lazy)
        self.junction_flush = toolhead.LOOKAHEAD_FLUSH_TIME
        for i in range(total):
            j = i * 3
            queue[i].set_junction(output[j], output[j+1], output[j+2])
        result = queue[:total]
        del queue[:total]
        stats["planned"] += total
        return result

    flush._klipper_rust_bridge = True
    toolhead.LookAheadQueue.flush = flush
    return stats
