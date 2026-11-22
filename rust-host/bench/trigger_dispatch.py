#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare stock C trigger propagation using only in-memory serial callbacks."""
import argparse
from pathlib import Path
import os
import random
import subprocess
import tempfile

from planner import PIN, ROOT


def vectors():
    rng = random.Random(27341)
    rows = []
    for case in range(240):
        rows.append("P")
        count = 1 if case % 8 == 0 else rng.randint(2, 9)
        base = 10. + case / 5.
        peers = []
        for peer in range(count):
            oid = 0 if peer % 3 else 3 + peer
            hz = rng.choice([1e6, 12e6, 72e6, 180e6])
            start = (1 << 32) + rng.randrange(-1_000_000, 1_000_000)
            timeout = int(hz * (.25 if count == 1 else .025))
            extend = int(timeout * .24 + .5)
            peers.append((oid, hz, start))
            rows.append(f"M {oid} {start} {start+timeout} {timeout} {extend} {hz} {base} {start}")
        for i in range(60):
            peer = rng.randrange(count)
            oid, hz, start = peers[peer]
            at = .002 * i
            when = base + at + .0005
            clock = (start + int((at - rng.uniform(0, .0003)) * hz)) & 0xffffffff
            if i % 11 == 9:
                oid ^= 1  # An unrelated object on the same node.
            reason = (4 if case % 2 else 1) if i == 43 else 0
            rows.append(f"S {peer} {oid} {int(not reason)} {reason} {clock} {when} {hz} {base} {start}")
    return "\n".join(rows) + "\n"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/trigger_dispatch_replay")
    a = p.parse_args()
    path = a.upstream.resolve() / "klippy/chelper"
    script = vectors()
    with tempfile.TemporaryDirectory() as directory:
        binary = Path(directory) / "reference"
        subprocess.run([os.environ.get("CC", "cc"), "-O3", "-I", path, ROOT/"bench/trigger_dispatch.c",
                        path/"trdispatch.c", path/"msgblock.c", path/"pyhelper.c", "-pthread", "-lm",
                        "-o", binary], check=True)
        def run(exe):
            result = subprocess.run([exe], input=script, text=True, capture_output=True, check=True)
            return [line.split() for line in result.stdout.splitlines()]
        expected = run(binary)
        actual = run(a.binary)
    reports = (row for row in script.splitlines() if row.startswith("S "))
    for want, got, report in zip(expected, actual, reports, strict=True):
        assert want == got, (report, want, got)
    actions = [a for row in actual for a in row]
    print(f"{len(actual)} reports; {sum(a.startswith('E') for a in actions)} exact expiry extensions; "
          f"{sum(a.startswith('T') for a in actions)} exact trigger sends")


if __name__ == "__main__":
    main()
