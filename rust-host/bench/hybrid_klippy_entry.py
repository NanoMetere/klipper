#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""A disposable debug-output-only Klippy runner with the Rust seam turned on."""
import json
import os
from pathlib import Path
import runpy
import stat
import sys
import time

from planner import ROOT


def install_debug_drain():
    """Wait for the asynchronous C file writer before stock disconnect."""
    import serialhdl

    original = serialhdl.SerialReader.disconnect
    failures = []
    seen = []

    def disconnect(serial):
        handle = serial.serial_dev
        if serial.serialqueue is not None and handle is not None and not handle.closed:
            # Offline -o files only. Never wait on or modify a device.
            if stat.S_ISREG(os.fstat(handle.fileno()).st_mode):
                deadline = time.monotonic() + 2.0
                try:
                    while True:
                        counters = dict(token.split("=",1) for token in serial.stats(0.).split())
                        if int(counters["ready_bytes"]) == int(counters["upcoming_bytes"]) == 0:
                            seen.append(str(handle.name))
                            break
                        if time.monotonic() >= deadline:
                            raise RuntimeError("timed out draining file-output queue")
                        time.sleep(0.001)
                except (RuntimeError, KeyError, ValueError) as error:
                    failures.append(str(error))
        return original(serial)

    serialhdl.SerialReader.disconnect = disconnect
    return failures, seen


def main():
    prefix = sys.argv[1:]
    drain = prefix[:1] == ["--drain-debug-output"]
    if drain:
        prefix = prefix[1:]
    dispatch = prefix[:1] == ["--rust-dispatch"]
    if dispatch:
        prefix = prefix[1:]
    if len(prefix) < 3:
        raise SystemExit("usage: hybrid_klippy_entry.py [--drain-debug-output] [--rust-dispatch] KLIPPY.py LIBRARY [debug arguments]")
    klippy = Path(prefix[0]).resolve()
    library = None if prefix[1] == "--stock-only" else Path(prefix[1]).resolve()
    if dispatch and library is None:
        raise SystemExit("--rust-dispatch requires the bridge library")
    args = prefix[2:]
    # Only explicit debug-output flags. Flag values may not be other flags.
    if not args or args[0].startswith("-") or len(args[1:]) % 2:
        raise SystemExit("expected CONFIG followed by -i/-o/-d/-l pairs")
    seen = set()
    for key, value in zip(args[1::2], args[2::2], strict=True):
        if key not in {"-i", "-o", "-d", "-l"} or value.startswith("-"):
            raise SystemExit("invalid debug flag or value")
        if key in seen and key != "-d":
            raise SystemExit(f"duplicate debug flag {key}")
        seen.add(key)
    if not {"-i", "-o", "-d"} <= seen:
        raise SystemExit("bridge fixture requires -i, -o and -d")
    sys.path.insert(0, str(klippy.parent))
    sys.path.insert(0, str(ROOT / "python"))
    import toolhead
    draining = install_debug_drain() if drain else None
    stats = None
    dispatch_stats = None
    if library is not None:
        from klipper_rust_lookahead import install
        stats = install(toolhead, library)
    if dispatch:
        import gcode
        from klipper_rust_dispatch import install
        dispatch_stats = install(gcode, library)
    sys.argv = [str(klippy), *args]
    try:
        runpy.run_path(str(klippy), run_name="__main__")
    finally:
        if draining is not None:
            failures, seen = draining
            if failures or not seen:
                raise RuntimeError(f"debug output did not drain: {failures}")
            print("FILE_OUTPUT_DRAIN", json.dumps(sorted(seen)), flush=True)
        if stats is not None:
            print("RUST_LOOKAHEAD_STATS", json.dumps(stats, sort_keys=True), flush=True)
        if dispatch_stats is not None:
            print("RUST_DISPATCH_STATS", json.dumps(dispatch_stats, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
