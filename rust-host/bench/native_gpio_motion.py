#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run real stock Linux MCU scheduling with compile-time GPIO stubs only."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

from mcu_config import fresh
from planner import PIN, ROOT


def build(upstream, directory):
    head = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    if head != PIN:
        raise RuntimeError(f"fixture requires pinned stock {PIN}, got {head}")
    source = upstream / ".config"
    if "CONFIG_MACH_LINUX=y" not in source.read_text().splitlines():
        raise RuntimeError("a Linux MCU configuration is required")
    # Use the pinned tree, not the checkout's local files or a live installation.
    archive = subprocess.Popen(["git", "-C", upstream, "archive", head],
                               stdout=subprocess.PIPE)
    assert archive.stdout is not None
    try:
        with tarfile.open(fileobj=archive.stdout, mode="r|") as tree:
            tree.extractall(directory, filter="data")
    finally:
        archive.stdout.close()
        if archive.wait():
            raise RuntimeError("git archive failed")
    shutil.copyfile(source, directory / ".config")
    shutil.copyfile(ROOT/"bench/linux_gpio_fixture.c", directory/"src/linux/gpio.c")
    shutil.copyfile(ROOT/"bench/linux_timer_fixture.c", directory/"src/linux/timer.c")
    # Test-only accessor; the stock scheduling logic is unchanged.
    with (directory/"src/sched.c").open("a") as sched:
        sched.write("\nuint32_t rust_fixture_next_timer(void) { return SchedStatus.timer_list->waketime; }\n")
    for cmd in (["make", "olddefconfig"], ["make", "-j2"]):
        result = subprocess.run(cmd, cwd=directory, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        if result.returncode:
            raise RuntimeError(result.stdout)
    compiled = directory / "out/klipper.dict"
    config = json.loads(compiled.read_text())["config"]
    if any(config.get(key) != "rust-host-test-v1" for key in ("GPIO_SIM_ONLY", "TIMER_SIM_ONLY")):
        raise RuntimeError("refusing to run an unmarked software fixture")
    return directory / "out/klipper.elf"


def run(binary, firmware, mode="--gpio-motion"):
    with fresh(firmware) as port:
        return subprocess.run([binary, mode, "1"], stdin=port, stdout=port,
                              stderr=subprocess.PIPE, text=True, timeout=12)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN} with a Linux .config and build")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/native_mcu")
    p.add_argument("--rounds", type=int, default=1)
    a = p.parse_args()
    if not 1 <= a.rounds <= 10:
        p.error("--rounds must be between 1 and 10")
    upstream, binary = a.upstream.resolve(), a.binary.resolve()
    control = run(binary, upstream/"out/klipper.elf")
    assert control.returncode and "NotFixture" in control.stderr, control.stderr
    with tempfile.TemporaryDirectory(prefix="klipper-gpio-stub-") as folder:
        mcu = build(upstream, Path(folder))
        cases = {"--gpio-motion": [720, -120, 112], "--gpio-extrusion": [720, -120, 112, 100]}
        for mode, expected in cases.items():
            for _ in range(a.rounds):
                outcome = run(binary, mcu, mode)
                assert outcome.returncode == 0, (mode, outcome.stderr)
                tag, crc, payloads, *positions = outcome.stderr.split()
                assert tag == "GPIO_FIXTURE" and int(crc) != 0
                assert int(payloads) > 0 and [int(p) for p in positions] == expected, outcome.stderr
            print(f"{mode}: {a.rounds} isolated run(s), {payloads} commands; C firmware and "
                  f"Rust agree on motor counts {expected}")
    print("Plain Linux MCU refused before configuration/motion")


if __name__ == "__main__":
    main()
