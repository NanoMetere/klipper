#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Exercise the integrated read-only software demonstrator."""
import json
from pathlib import Path
import subprocess
import sys


def main():
    exe, dictionary = map(Path, sys.argv[1:])
    raw = json.loads(dictionary.read_text())
    actual = json.loads(subprocess.check_output([exe, dictionary], text=True))
    assert actual["mode"] == "simulation" and actual["state"] == "complete", actual
    assert actual["preflight_boundary"] == 10 < actual["source_bytes"], actual
    assert actual["clock_hz"] == raw["config"]["CLOCK_FREQ"], actual
    assert all(i > 0 for i in actual["lanes"]) and len(actual["lanes"]) == 3, actual
    assert sum(actual["lanes"]) == actual["issued"] == actual["total"], actual
    print(f"{actual['issued']} firmware-dictionary commands in three inert lanes")


if __name__ == "__main__":
    main()
