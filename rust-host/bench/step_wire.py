#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare simulated motion payloads and framing with the stock dictionary encoder."""
import argparse
from pathlib import Path
import random
import subprocess
import sys

from planner import PIN


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("dictionary", type=Path)
    p.add_argument("binary", type=Path)
    a = p.parse_args()
    sys.path.insert(0, str(a.upstream.resolve() / "klippy"))
    import msgproto
    mp = msgproto.MessageParser()
    mp.process_identify(a.dictionary.read_bytes(), decompress=False)
    r = random.Random(24458)
    rows, want = [], []
    for i in range(800):
        oid = r.randrange(256)
        first = r.randrange(1 << 32) + (2**33 if i % 2 else 0)
        required = first
        trace = []
        rows.append(f"m {oid} {first}")
        positive = i % 2
        for j in range(r.randrange(2, 20)):
            if j == 0 or r.randrange(4) == 0:
                positive ^= 1
                rows.append(f"d {positive}")
                cmd = f"set_next_step_dir oid={oid} dir={positive}"
                trace.append((0, required, cmd))
            interval = r.choice([1, 95, 96, 12288, 19999, 2**20])
            count = r.choice([1, 2, 7, 95, 96, 1023])
            add = r.choice([-32768, -13, -1, 0, 1, 200, 32767])
            if interval + (count - 1)*add <= 0 or interval*count + add*count*(count-1)//2 >= (3 << 28):
                interval, count, add = (10, 33, 7)
            last = required + interval*count + add*count*(count-1)//2
            rows.append(f"s {interval} {count} {add} {last}")
            cmd = f"queue_step oid={oid} interval={interval} count={count} add={add}"
            trace.append((required, required, cmd))
            required = last
        sequence = len(rows)
        rows.append("e")
        for j, (earliest, deadline, spec) in enumerate(trace):
            payload = list(mp.create_command(spec))
            raw = mp.encode_msgblock(sequence + j, payload)
            frame = [v for entry in raw for v in (entry if isinstance(entry, list) else [entry])]
            assert mp.check_packet(frame) == len(frame)
            want.append(f"{earliest} {deadline} {bytes(payload).hex()} {bytes(frame).hex()}")
    got = subprocess.check_output([a.binary.resolve(), a.dictionary.resolve()],
                                  text=True, input="\n".join(rows)+"\n").splitlines()
    for i, (x, y) in enumerate(zip(want, got, strict=True)):
        assert x == y, (i, x, y)
    print(f"{len(got)} exact simulated step commands and frames (800 lanes)")


if __name__ == "__main__":
    main()
