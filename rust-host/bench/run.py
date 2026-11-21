# SPDX-License-Identifier: GPL-3.0-only
"""Compare valid-frame receive checks on the same machine."""
import argparse
import importlib.util
import os
import pathlib
import statistics
import subprocess
import tempfile

crate = pathlib.Path(__file__).resolve().parents[1]
args = argparse.ArgumentParser(description=__doc__)
args.add_argument("upstream", nargs="?", type=pathlib.Path, default=crate.parent)
args.add_argument("--baseline", type=pathlib.Path, help="previous Rust lib.rs")
options = args.parse_args()
upstream = options.upstream.resolve()
spec = importlib.util.spec_from_file_location("msgproto", upstream / "klippy/msgproto.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
parser = module.MessageParser()

def make_frame(seq, size):
    payload = [(seq * 29 + i * 13) & 0xFF for i in range(size - 5)]
    parts = parser.encode_msgblock(seq, payload)
    frame = bytes(parts[:-2] + parts[-2] + [parts[-1]])
    assert parser.check_packet(frame) == size
    return frame.ljust(64, b"\x00")

def compile_rust(temp, label, source):
    lib = temp / f"lib{label}.rlib"
    binary = temp / label
    subprocess.run([
        "rustc", "--edition=2021", "-O", "--crate-type=rlib",
        "--crate-name=klipper_host_protocol", str(source), "-o", str(lib),
    ], check=True)
    subprocess.run([
        "rustc", "--edition=2021", "-O", str(crate / "bench/frames.rs"),
        "--extern", f"klipper_host_protocol={lib}", "-o", str(binary),
    ], check=True)
    return binary

profiles = {
    "short": (5, 6, 8, 11),
    "mixed": (5, 8, 11, 64),
    "long": (32, 48, 63, 64),
}
with tempfile.TemporaryDirectory() as directory:
    temp = pathlib.Path(directory)
    chelper = upstream / "klippy/chelper"
    c_binary = temp / "c-frames"
    subprocess.run([
        os.environ.get("CC", "cc"), "-O3", "-ffunction-sections", "-fdata-sections",
        "-Wl,--gc-sections", "-I", str(chelper), str(crate / "bench/frames.c"),
        str(chelper / "msgblock.c"), "-o", str(c_binary),
    ], check=True)
    binaries = [("Klippy C", c_binary)]
    if options.baseline:
        binaries.append(("Rust baseline", compile_rust(temp, "baseline", options.baseline)))
    binaries.append(("Rust current", compile_rust(temp, "current", crate / "src/lib.rs")))
    for profile, sizes in profiles.items():
        corpus = temp / "frames.bin"
        corpus.write_bytes(b"".join(make_frame(seq, size) for seq, size in enumerate(sizes)))
        samples = {label: [] for label, _ in binaries}
        for turn in range(10):
            start = turn % len(binaries)
            for label, binary in binaries[start:] + binaries[:start]:
                result = float(subprocess.check_output([binary, corpus]))
                if turn:
                    samples[label].append(result)
        print(f"{profile} {sizes}")
        for label, values in samples.items():
            print(f"  {label}: {statistics.median(values):.2f} ns/frame "
                  f"(min/max {min(values):.2f}/{max(values):.2f}; n={len(values)})")
