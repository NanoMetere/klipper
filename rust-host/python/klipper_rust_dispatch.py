# SPDX-License-Identifier: GPL-3.0-only
"""Optional canonical G0/G1 scanner for the pinned offline Klippy fixture.

The command dispatch and error path is adapted from klippy/gcode.py
(Copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>).
"""
import ctypes
import hashlib
import inspect
import logging
import struct

_FIELDS = ("", "X", "Y", "Z", "E", "F")
_TAG = ("", "G0", "G1")
_DIGITS = ("", "0", "1")
_ROW = struct.Struct("16B")
_STOCK_DISPATCH_SHA = "b9f11eabb33a4fef594c639e50607708aae5b4abdb5b9af395e92882d5f8ba5a"


def install(gcode, library):
    """Install only for the explicitly opted-in file-output harness."""
    cls, factory = gcode.GCodeDispatch, gcode.GCodeCommand
    stock = cls._process_commands
    regex = cls.args_r
    if (hashlib.sha256(inspect.getsource(stock).encode()).hexdigest() != _STOCK_DISPATCH_SHA
            or regex.pattern != "([A-Z_]+|[A-Z*])"):
        raise ValueError("unrecognized stock G-code dispatcher")
    native = ctypes.CDLL(str(library))
    scan = native.klipper_rs_move_scan_v1
    scan.argtypes = (ctypes.c_char_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t)
    scan.restype = ctypes.c_int
    totals = {"canonical": 0, "stock": 0}

    def process(self, commands, need_ack=True):
        if (self.args_r is not regex or type(commands) not in (list, tuple)
                or not 8 <= len(commands) <= 4096):
            if type(commands) in (list, tuple):
                totals["stock"] += len(commands)
            return stock(self, commands, need_ack)
        try:
            source = "\n".join(commands).encode("ascii")
        except (TypeError, UnicodeError):
            totals["stock"] += len(commands)
            return stock(self, commands, need_ack)
        size = len(commands) * _ROW.size
        out = (ctypes.c_ubyte * size)()
        if scan(source, len(source), out, size):
            totals["stock"] += len(commands)
            return stock(self, commands, need_ack)
        count = 0
        for line, row in zip(commands, _ROW.iter_unpack(out), strict=True):
            code = row[0]
            if not code:
                stock(self, [line], need_ack)
                continue
            count += 1
            cmd = _TAG[code]
            params = {"G": _DIGITS[code]}
            for i in (1, 4, 7, 10, 13):
                field = row[i]
                if not field:
                    break
                params[_FIELDS[field]] = line[row[i + 1]:row[i + 2]]
            gcmd = factory(self, cmd, line, params, need_ack)
            # Preserve the verified stock handler and ACK/error policy.
            handler = self.gcode_handlers.get(cmd, self.cmd_default)
            try:
                handler(gcmd)
            except self.error as e:
                self._respond_error(str(e))
                self.printer.send_event("gcode:command_error")
                if not need_ack:
                    raise
            except:
                msg = 'Internal error on command:"%s"' % (cmd,)
                logging.exception(msg)
                self.printer.invoke_shutdown(msg)
                self._respond_error(msg)
                if not need_ack:
                    raise
            gcmd.ack()
        totals["canonical"] += count
        totals["stock"] += len(commands) - count

    cls._process_commands = process
    return totals
