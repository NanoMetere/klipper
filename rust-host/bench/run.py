# SPDX-License-Identifier: GPL-3.0-only
"""Run from any directory: python3 bench/run.py [upstream checkout path]."""
import importlib.util
import os
import pathlib
import statistics
import subprocess
import sys
import tempfile
import time

crate = pathlib.Path(__file__).resolve().parents[1]
upstream = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else crate.parent
spec = importlib.util.spec_from_file_location("msgproto", upstream / "klippy/msgproto.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
parser = module.MessageParser()

frames = []
for seq, payload in enumerate(([], [1, 0, 40], [0, 0, 3, 0x78, 0x9C, 0x7E], list(range(59)))):
    parts = parser.encode_msgblock(seq, payload)
    frame = bytes(parts[:-2] + parts[-2] + [parts[-1]])
    assert parser.check_packet(frame) == len(frame)
    frames.append(frame)

with tempfile.TemporaryDirectory() as temp:
    temp = pathlib.Path(temp)
    corpus = temp / "frames.bin"
    corpus.write_bytes(b"".join(frame.ljust(64, b"\x00") for frame in frames))
    chelper = upstream / "klippy/chelper"
    subprocess.run([
        os.environ.get("CC", "cc"), "-O3", "-ffunction-sections", "-fdata-sections",
        "-Wl,--gc-sections", "-I", str(chelper), str(crate / "bench/frames.c"),
        str(chelper / "msgblock.c"), "-o", str(temp / "c-frames"),
    ], check=True)
    subprocess.run([
        "rustc", "--edition=2021", "-O", "--crate-type=rlib",
        "--crate-name=klipper_host_protocol", str(crate / "src/lib.rs"),
        "-o", str(temp / "libklipper_host_protocol.rlib"),
    ], check=True)
    subprocess.run([
        "rustc", "--edition=2021", "-O", str(crate / "bench/frames.rs"),
        "--extern", f"klipper_host_protocol={temp / 'libklipper_host_protocol.rlib'}",
        "-o", str(temp / "rust-frames"),
    ], check=True)

    def python_measure():
        iterations = 100_000
        total = 0
        start = time.perf_counter_ns()
        for i in range(iterations):
            total += parser.check_packet(frames[i & 3])
        elapsed = time.perf_counter_ns() - start
        assert total == iterations // 4 * sum(map(len, frames))
        return elapsed / iterations

    samples = {"Python": [], "C": [], "Rust": []}
    for i in range(8):
        measurements = {
            "C": float(subprocess.check_output([temp / "c-frames", corpus])),
            "Rust": float(subprocess.check_output([temp / "rust-frames", corpus])),
            "Python": python_measure(),
        }
        if i:
            for name, value in measurements.items():
                samples[name].append(value)
    for name, values in samples.items():
        print(f"{name}: {statistics.median(values):.2f} ns/frame "
              f"(min/max {min(values):.2f}/{max(values):.2f}; n={len(values)})")
