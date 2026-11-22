#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare unchanged Klippy C scheduling and offline Rust planning."""
import argparse
import collections
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PIN = "c9559fa04ec6b11fb1a9a1610903546319fcf310"
BG = 0x7FFFFFFF00000000
BASE = 200_000_000
RATE = 50_000_000


def fixture():
    rng = random.Random(20261122)
    rows = ["d 0 0 0 0 0 1 1 0 0 0 0 0",
            "p 0 0 0 6 91",
            "d 0 0 0 0 0 1 1 0 1 64 12 139",
            "d 0 0 0 0 0 1 1 0 0 0 0 0",
            "d 0 0 0 0 0 1 1 0 0 0 0 0"]
    for phase in range(500):
        offset = phase * 2
        for _ in range(rng.randint(3, 30)):
            release = BASE + offset * RATE + rng.randint(-300_000, 18_000_000)
            priority = release + rng.randrange(12_000_000)
            if rng.randrange(11) == 0:
                priority = BG
            if phase % 29 == 0 and rng.randrange(5) == 0:
                # Distant deadline requires the upstream 31-bit horizon clamp.
                priority += (3 << 29) if priority != BG else 0
            rows.append(f"p {rng.randrange(8)} {release} {priority} "
                        f"{rng.randrange(1, 60)} {rng.randrange(256)}")
        for i in range(45):
            delta = min(i * 0.02, 0.75)
            now = 100 + offset + delta
            idle = now + rng.uniform(0, 0.002) if rng.randrange(5) == 0 else now - 0.002
            wire, baud = rng.choice([(0, 0), (1, 250000), (2, 1000000)])
            flight = rng.randrange(13) if i < 20 else 0
            pending = rng.randrange(65) if rng.randrange(5) == 0 else 0
            window = rng.choice([0, 128, 192, 512])
            outstanding = rng.randrange(40 * flight + 1)
            prior = rng.randrange(64) if flight and rng.randrange(4) == 0 else 0
            rows.append(f"d {wire} {baud} {RATE} 100 {BASE} {now!r} {idle!r} "
                        f"{pending} {flight} {outstanding} {prior} {window}")
    return "\n".join(rows) + "\n"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"disposable Klipper tree at {PIN}")
    args = p.parse_args()
    helper = args.upstream.resolve() / "klippy/chelper"
    with tempfile.TemporaryDirectory() as root:
        temp = Path(root)
        c, rust = temp / "stock", temp / "rust"
        lib = temp / "libklipper_host_protocol.rlib"
        subprocess.run([
            os.environ.get("CC", "cc"), "-O3", "-ffunction-sections", "-fdata-sections",
            "-Wl,--gc-sections", "-pthread", "-I", helper, ROOT / "bench/planner.c",
            helper / "msgblock.c", helper / "pollreactor.c", helper / "pyhelper.c",
            "-o", c,
        ], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib", "--crate-name=klipper_host_protocol",
                        ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/planner.rs", "--extern",
                        f"klipper_host_protocol={lib}", "-o", rust], check=True)
        corpus = fixture()
        want, got = [subprocess.check_output([binary], input=corpus, text=True).splitlines()
                     for binary in (c, rust)]
        steps = [line for line in corpus.splitlines() if line.startswith("d ")]
        assert len(want) == len(got) == len(steps)
        types = collections.Counter()
        max_wake = 0.0
        for i, (a, b) in enumerate(zip(want, got)):
            x, y = a.split(), b.split()
            assert x[0] == y[0], (i, steps[i], a, b)
            types[x[0]] += 1
            if x[0] == "send":
                assert x[1] == y[1], (i, steps[i], a, b)
            elif len(x) == 2:
                delta = abs(float(x[1]) - float(y[1]))
                assert delta <= max(1e-10, 4 * math.ulp(float(x[1]))), (i, steps[i], a, b)
                max_wake = max(max_wake, delta)
        print(f"{len(steps)} scheduling decisions match: {dict(types)}; maximum wake delta {max_wake:g}s")


if __name__ == "__main__":
    main()
