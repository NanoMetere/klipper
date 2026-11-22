#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run the same real Python Klippain plugin smoke at two pinned Klipper versions."""
import argparse
from contextlib import redirect_stdout
from io import StringIO
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile

KLIPPAIN = "77ea66b12d79aa08e919c24446525811e93a7c8b"
TOOLCHANGER = "94756dfde9b729fd69f9b8780067821c5c99a528"
LED_EFFECT = "266f1049c7172c2fba0da4a52314dcfc0c3bb56f"
OLD = "644cda5ecaa39d0dcf797624c19d5425cb8121ec"
PORT = "c9559fa04ec6b11fb1a9a1610903546319fcf310"
KNOWN = ("ProbeEndstopWrapper.__init__() missing 2 required positional arguments: "
         "'probe_offsets' and 'param_helper'")
CASES = [("stealthchanger", False), ("tapchanger", False), ("stealthchanger", True)]
MODULES = ["tool.py", "toolchanger.py", "tool_probe.py", "tool_probe_endstop.py"]


def pin(repo, expected):
    got = subprocess.check_output(["git", "-C", repo, "rev-parse", "HEAD"], text=True).strip()
    if got != expected:
        raise SystemExit(f"Unexpected pin for {repo.name}: {got}; expected {expected}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for flag in ("klippain", "toolchanger", "led-effect", "old", "old-dictionary", "port", "port-dictionary"):
        p.add_argument(f"--{flag}", required=True, type=lambda s: Path(s).resolve())
    p.add_argument("--results", type=Path, help="empty output directory for generated stock logs")
    a = p.parse_args()
    for worktree, commit in [(a.klippain, KLIPPAIN), (a.toolchanger, TOOLCHANGER),
                             (a.led_effect, LED_EFFECT), (a.old, OLD), (a.port, PORT)]:
        pin(worktree, commit)
        subprocess.run(["git", "-C", worktree, "diff", "--quiet", "HEAD", "--"], check=True)
    for stock in (a.old, a.port):
        for mod in MODULES:
            installed = (stock / "klippy/extras" / mod).read_bytes()
            expected = (a.toolchanger / "klipper/extras" / mod).read_bytes()
            if installed != expected:
                raise SystemExit(f"{stock.name}: installed {mod} does not match pinned checkout")
        if (stock / "klippy/extras/led_effect.py").read_bytes() != (a.led_effect / "src/led_effect.py").read_bytes():
            raise SystemExit(f"{stock.name}: LED Effect module does not match pinned checkout")
    # Load only the known fixture script from the verified pinned worktree.
    sys.path.insert(0, str(a.klippain / "tests"))
    from run_toolchanger_klippy_smoke import run_case

    # Keep generated configs, debug command outputs and logs isolated.
    with tempfile.TemporaryDirectory(prefix="klippain-parity-") as temp:
        dest = a.results if a.results is not None else Path(temp)
        if dest.exists() and any(dest.iterdir()):
            raise SystemExit("Results destination must be empty")
        dest.mkdir(parents=True, exist_ok=True)
        unexpected = 0
        for name, stock, dictionary in [("legacy", a.old, a.old_dictionary),
                                        ("port-pin", a.port, a.port_dictionary)]:
            print(f"{name} dictionary SHA256: {hashlib.sha256(dictionary.read_bytes()).hexdigest()}")
            for mechanism, shared in CASES:
                label = "shared-led" if shared else mechanism
                out = dest / name / label
                out.mkdir(parents=True)
                try:
                    # The harness invokes Klippy with -i, -o and an offline JSON dictionary.
                    output = StringIO()
                    with redirect_stdout(output):
                        run_case(stock, dictionary, mechanism, 2, 300, out, shared_leds=shared)
                except RuntimeError:
                    log = out / "klippy.log"
                    known = log.exists() and KNOWN in log.read_text()
                    if name == "port-pin" and known:
                        print(f"{name} {label}: expected stock probe constructor incompatibility")
                    else:
                        print(f"{name} {label}: unexpected smoke failure")
                        print(output.getvalue())
                        unexpected += 1
                else:
                    print(output.getvalue(), end="")
                    if name != "legacy":
                        print(f"{name} {label}: unexpectedly passed; re-baseline the matrix")
                        unexpected += 1
        if unexpected:
            raise SystemExit(f"{unexpected} unexpected test outcome(s)")


if __name__ == "__main__":
    main()
