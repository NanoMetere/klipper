#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare raw and compressed shared E windows with unchanged stock C."""
import argparse
from collections import Counter
from pathlib import Path
import random
import subprocess
import tempfile

from planner import PIN, ROOT

def corpus():
    yield (ROOT / "bench/shared_extruder_fixture/window_seed.txt").read_text()
    r = random.Random(986102)
    for case in range(70):
        cmds = ["p 0 0.04", "b 0 0"]
        prior = 0
        pos = [0., 0.]
        for phase in range(8):
            at = 2. + phase * .5
            q = [0, 1, 0, -1, 1, 0, 1, 0][phase]
            actual = phase % 2 if q == -1 else q
            if phase + case > 0 and phase % 2 == 0:
                cmds.append(f"p {at - .125} {r.choice([.0375, .06, .0825, .1])}")
            if prior != q:
                cmds.append(f"b {q} {pos[actual]}")
                prior = q
            for j in range(2):
                duration = r.choice([.03125, .0625, .09375])
                speed = r.choice([-9., -5., -3., 4., 7., 8., 11.])
                acc = speed / .03125
                t = at + j * .1875
                pa = int(speed > 0. and r.choice([True,True,False]))
                cmds.append(f"a {actual} {pa} {t} .03125 {duration} .03125 {pos[actual]} 0 {speed} {acc}")
                pos[actual] += speed * (duration + .03125)
            for t in (at-.019, at+.009, at+.095, at+.173, at+.201, at+.272, at+.387):
                cmds.append(f"f {t}")
        yield "\n".join(cmds) + "\n"


def parsed(text):
    return [(s[0], *map(float, s[1:])) for line in text.splitlines() if (s := line.split())]


def compare(stock, rust):
    total = Counter()
    for case, script in enumerate(corpus()):
        traces = [parsed(subprocess.check_output(binary, input=script,
                                                 text=True, timeout=3))
                  for binary in (stock, rust)]
        for i, (expected, actual) in enumerate(zip(*traces, strict=True)):
            assert expected == actual, (case, i, expected, actual)
        total.update(row[0] for row in traces[0])
    return dict(total)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=lambda path: Path(path).resolve())
    a = parser.parse_args()
    assert subprocess.check_output(["git", "-C", a.upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    helper = a.upstream / "klippy/chelper"
    c = ROOT / "bench/shared_windows.c"
    with tempfile.TemporaryDirectory(prefix="klipper-shared-windows-") as temp:
        d = Path(temp)
        stock_raw, stock_wire, rust = (d / name for name in ("stock-raw", "stock-wire", "rust"))
        lib = d / "libklipper_host_protocol.rlib"
        common = ["cc", "-O3", "-I", helper, c,
                  *(helper / f"{src}.c" for src in ("itersolve", "trapq", "kin_extruder"))]
        subprocess.run([*common, "-o", stock_raw], check=True)
        subprocess.run([*common, "-DWIRE",
                        *(helper / f"{src}.c" for src in ("stepcompress", "msgblock", "pyhelper")),
                        "-lm", "-o", stock_wire], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type", "rlib",
                        "--crate-name", "klipper_host_protocol", ROOT / "src/lib.rs",
                        "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/shared_windows.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", rust], check=True)
        raw = compare([stock_raw], [rust])
        wire = compare([stock_wire], [rust, "--wire"])
        assert raw == {"bind": 565, "step": 133188, "commit": 269845, "flush": 3936}
        assert wire == {"bind": 565, "flush": 3936, "d": 985, "s": 66668}
        print(f"71 offline 2-queue scenarios, exact switch, raw and compressed E records: "
              f"{raw}; {wire}")


if __name__ == "__main__":
    main()
