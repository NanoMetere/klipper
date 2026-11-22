#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify Classic CAN framing and six isolated receive streams without any CAN interface."""
import argparse
from collections import Counter, deque
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile

from planner import PIN, ROOT


def fixture(upstream):
    sys.path.insert(0, str(upstream / "klippy"))
    from msgproto import crc16_ccitt

    r = random.Random(76912)
    for i in range(640):
        count = r.choice([0, 1, 7, 8, 9, 63, 64, r.randrange(770), 769])
        yield ("t", r.randrange(4, 256), count, *r.randbytes(count))
    lanes = [deque() for _ in range(6)]
    for n, lane in enumerate(lanes):
        stream = bytearray()
        for k in range(240):
            if k % 9 == n:
                stream.extend(b"\xff" * (k % 31 + 1) + b"\x7e")
            size = r.randrange(60)
            base = [size+5, 0x10|(k&15), *r.randbytes(size)]
            stream.extend(bytes(base + crc16_ccitt(base) + [0x7e]))
        while stream:
            length = r.randint(1, 8)
            lane.append(bytes(stream[:length]))
            del stream[:length]
    while active := [i for i, q in enumerate(lanes) if q]:
        which = r.choice(active)
        if r.randrange(9) == 0:
            bad = r.choice([0, 264, 270, 277, 0x3f0, 0x3f1, 0x80000109, 0x4000010b])
            data = r.randbytes(r.randrange(9))
            yield ("r", bad, len(data), *data)
        data = lanes[which].popleft()
        yield ("r", (which + 4)*2+257, len(data), *data)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    args = p.parse_args()
    h = args.upstream.resolve() / "klippy/chelper"
    data = "\n".join(" ".join(map(str,r)) for r in fixture(args.upstream.resolve()))+"\n"
    with tempfile.TemporaryDirectory() as d:
        t = Path(d)
        c, rust, lib = t/"c", t/"rust", t/"libklipper_host_protocol.rlib"
        subprocess.run([os.environ.get("CC","cc"), "-O3", "-ffunction-sections", "-fdata-sections",
                        "-Wl,--gc-sections", "-pthread", "-I", h, ROOT/"bench/can.c", h/"msgblock.c",
                        h/"pyhelper.c", h/"pollreactor.c", "-o", c], check=True)
        subprocess.run(["rustc","--edition=2021","-O","--crate-type=rlib","--crate-name=klipper_host_protocol",
                        ROOT/"src/lib.rs","-o",lib],check=True)
        subprocess.run(["rustc","--edition=2021","-O",ROOT/"bench/can.rs","--extern",
                        f"klipper_host_protocol={lib}","-o",rust],check=True)
        a,b = [subprocess.check_output([exe],input=data,text=True,timeout=20).splitlines() for exe in [c,rust]]
        for i,(x,y) in enumerate(zip(a,b,strict=True)):
            assert x==y,(i,x,y)
        print(f"Exact stock outcomes: {dict(Counter(r.split()[0] for r in a))}")


if __name__=="__main__":
    main()
