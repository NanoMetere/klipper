#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run the Rust host's pinned correctness matrix without real hardware."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

from planner import PIN, ROOT

OFFLINE = (
    "check_stream.sh", "clock_sync.py", "planner.py", "flight.py", "retry_timing.py", "can.py",
    "config.py", "lookahead.py", "gcode_preview.py", "gcode_arcs.py", "bed_mesh.py", "trajectory.py", "step_times.py",
    "step_compress.py", "step_clock.py", "machine_trace.py", "pressure_advance.py",
    "tool_transform.py", "thermal.py",
)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--upstream", type=Path, required=True, help=f"git checkout at {PIN}")
    p.add_argument("--dictionary", type=Path, help="JSON from matching Linux MCU build")
    p.add_argument("--native-pty", action="store_true", help="run only against a newly created Linux MCU PTY")
    a = p.parse_args()
    upstream = a.upstream.resolve()
    got = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    if got != PIN:
        p.error(f"upstream must be pinned at {PIN}, got {got}")
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    dictionary = (a.dictionary or upstream / "out/klipper.dict").resolve()
    target = Path(os.environ.get("CARGO_TARGET_DIR", ROOT / "target"))
    if not target.is_absolute():
        target = ROOT / target
    examples = target / "release/examples"
    phases = [
        ("rust_debug", ["cargo", "test", "--locked", "--all-targets", "-q"]),
        ("rust_release", ["cargo", "test", "--locked", "--release", "--all-targets", "-q"]),
        ("rust_nodefault", ["cargo", "test", "--locked", "--no-default-features", "--all-targets", "-q"]),
    ]
    for name in OFFLINE:
        runtime = "sh" if name.endswith(".sh") else sys.executable
        extra = ["--rounds", "0"] if name in ("lookahead.py", "gcode_preview.py") else []
        phases.append((name, [runtime, ROOT/"bench"/name, upstream, *extra]))
    phases.extend([
        ("build_fixture_binaries", ["cargo", "build", "--locked", "--release", "--examples"]),
        ("webhooks", [sys.executable, ROOT/"bench/webhooks.py", upstream, examples/"webhook_replay"]),
    ])
    if dictionary.is_file():
        phases.append(("step_wire", [sys.executable, ROOT/"bench/step_wire.py", upstream,
                                     dictionary, examples/"step_wire_replay"]))
    else:
        print("SKIP step_wire (no matching Linux firmware dictionary supplied)", flush=True)
    if a.native_pty:
        if not sys.platform.startswith("linux") or not (upstream/"out/klipper.elf").is_file():
            p.error("--native-pty requires Linux and a native MCU built in the pinned checkout")
        phases.extend([
            ("native_clock", [sys.executable, ROOT/"bench/native_mcu.py", upstream, "--binary",
                              examples/"native_mcu", "--live-clock"]),
            ("native_pipeline", [sys.executable, ROOT/"bench/pipeline_pty.py", upstream,
                                 examples/"native_mcu", "--waves", "8", "--rounds", "2"]),
        ])
    failures = []
    for name, command in phases:
        start = time.monotonic()
        try:
            result = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=180)
            ok, output = result.returncode == 0, result.stdout
        except (OSError, subprocess.TimeoutExpired) as e:
            ok, output = False, str(e)
        print(f"{'PASS' if ok else 'FAIL'} {name} ({time.monotonic()-start:.2f}s)", flush=True)
        if not ok:
            print(output[-8000:], flush=True)
            failures.append(name)
    print(f"{len(phases)-len(failures)}/{len(phases)} phases passed")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
