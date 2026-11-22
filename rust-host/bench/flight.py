#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare the Rust flight ledger with pinned stock C receive/retry routines."""
import argparse
import os
from pathlib import Path
import random
import subprocess
import tempfile

from planner import PIN, ROOT


def fixture():
    yield from ("s 10 17 0", "s 17 18 0", "s 20 19 0", "a 0 1", "a 0 0",
                "a 0 0", "a 0 0", "t", "a 1 0")
    randomizer = random.Random(20261122)
    for _ in range(30_000):
        op = randomizer.randrange(100)
        if op < 60:
            yield (f"s {randomizer.randrange(1, 60)} {randomizer.randrange(256)} "
                   f"{randomizer.choice([0, 79, 128, 256, 512])}")
        elif op < 93:
            yield f"a {randomizer.randrange(16)} {randomizer.randrange(2)}"
        else:
            yield "t"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"disposable Klipper tree at {PIN}")
    helper = p.parse_args().upstream.resolve() / "klippy/chelper"
    with tempfile.TemporaryDirectory() as root:
        temp = Path(root)
        c, rust = temp / "stock", temp / "rust"
        lib = temp / "libklipper_host_protocol.rlib"
        subprocess.run([
            os.environ.get("CC", "cc"), "-O3", "-ffunction-sections", "-fdata-sections",
            "-Wl,--gc-sections", "-pthread", "-I", helper, ROOT / "bench/flight.c",
            helper / "msgblock.c", helper / "pollreactor.c", helper / "pyhelper.c",
            "-o", c,
        ], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib],
                       check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/flight.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        corpus = list(fixture())
        want, got = [subprocess.check_output([binary], input="\n".join(corpus) + "\n",
                                             text=True).splitlines() for binary in (c, rust)]
        for i, (a, b) in enumerate(zip(want, got, strict=True)):
            assert a == b, (i, a, b)
        retries = sum(row.startswith("7e") for row in want)
        print(f"{len(corpus)} flight events, {len(want)} outcomes, {retries} exact retransmissions")


if __name__ == "__main__":
    main()
