#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""A disposable debug-output-only Klippy runner with the Rust seam turned on."""
import json
from pathlib import Path
import runpy
import sys

from planner import ROOT


def main():
    if len(sys.argv) < 4:
        raise SystemExit("usage: hybrid_klippy_entry.py KLIPPY.py LIBRARY [Klippy debug arguments]")
    klippy = Path(sys.argv[1]).resolve()
    library = None if sys.argv[2] == "--stock-only" else Path(sys.argv[2]).resolve()
    args = sys.argv[3:]
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
    stats = None
    if library is not None:
        from klipper_rust_lookahead import install
        stats = install(toolhead, library)
    sys.argv = [str(klippy), *args]
    try:
        runpy.run_path(str(klippy), run_name="__main__")
    finally:
        if stats is not None:
            print("RUST_LOOKAHEAD_STATS", json.dumps(stats, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
