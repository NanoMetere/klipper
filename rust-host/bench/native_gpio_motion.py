#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run real stock Linux MCU scheduling with compile-time GPIO stubs only."""
import argparse
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tarfile
import tempfile

from mcu_config import drop_write_ack, fresh
from native_gpio_dual import check_dual, check_mixed_tool_proofs
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
    p.add_argument("--dual-rounds", type=int, default=1)
    p.add_argument("--mixed-rounds", type=int, default=1)
    a = p.parse_args()
    if not all(1 <= r <= 99 for r in (a.rounds,a.dual_rounds,a.mixed_rounds)):
        p.error("round counts must be between 1 and 99")
    upstream, binary = a.upstream.resolve(), a.binary.resolve()
    control = run(binary, upstream/"out/klipper.elf")
    assert control.returncode and "NotFixture" in control.stderr, control.stderr
    for flag in ("--gpio-sparse", "--gpio-dock", "--gpio-mixed", "--gpio-mixed-shutdown",
                 "--gpio-tool-mixed", "--gpio-tool-mixed-shutdown"):
        check = run(binary, upstream/"out/klipper.elf", flag)
        assert check.returncode and "NotFixture" in check.stderr, check.stderr
    with tempfile.TemporaryDirectory(prefix="klipper-gpio-stub-") as folder:
        mcu = build(upstream, Path(folder))
        expected_positions = {
            "--gpio-motion": [720, -120, 112],
            "--gpio-extrusion": [720, -120, 112, 100],
            "--gpio-batched": [720, -120, 112, 100],
            "--gpio-multitool": [3920, -80, 40, 150, 175],
            "--gpio-sparse": [4080, 400, 40, 100, 155, 50],
            "--gpio-dock": [20, 20, 0, -25, 50],
        }
        modes = list(expected_positions)
        series = {mode: [] for mode in modes}
        summaries = {}
        for round_number in range(a.rounds):
            split = round_number % len(modes)
            for mode in modes[split:] + modes[:split]:
                outcome = run(binary, mcu, mode)
                assert outcome.returncode == 0, (mode, outcome.stderr)
                lines = outcome.stderr.splitlines()
                assert len(lines) == 2, lines
                tag, crc, payloads, *positions = lines[0].split()
                enqueue_tag, frames, elapsed = lines[1].split()
                expected = expected_positions[mode]
                assert tag == "GPIO_FIXTURE" and int(crc) != 0
                assert int(payloads) > 0 and [int(p) for p in positions] == expected, outcome.stderr
                assert enqueue_tag == "ENQUEUE" and 0 < int(frames) <= int(payloads) and int(elapsed) > 0
                summary = (int(payloads), int(frames), expected, int(crc))
                assert summaries.setdefault(mode, summary) == summary
                series[mode].append(int(elapsed))
        plain, packed = (summaries[mode] for mode in modes[1:3])
        assert plain[0] == packed[0] and packed[1] < plain[1] and packed[2:] == plain[2:], summaries
        for mode, times in series.items():
            payloads, frames, expected, _ = summaries[mode]
            print(f"{mode}: {a.rounds} isolated run(s), {payloads} commands in {frames} frames; "
                  f"matched {expected}; median enqueue {statistics.median(times)*1e-6:.3f} ms")
        sys.path.insert(0, str(upstream/"klippy"))
        from msgproto import MessageParser
        parser = MessageParser()
        parser.process_identify((mcu.parent/"klipper.dict").read_bytes(), decompress=False)
        candidates = [bytes(parser.messages_by_name[name].msgid_bytes) for name in ("queue_step", "set_next_step_dir")]
        details = drop_write_ack(mcu, binary, "--gpio-batched",
                                 lambda p: len(p) > 20 and any(p.startswith(t) for t in candidates))
        assert "motion frame" in details and "GPIO_FIXTURE" not in details, details
        print("Dropped a batched motion ACK: no extra host frame or virtual clock advance")
        check_dual(upstream, mcu, binary, a.dual_rounds)
        check_mixed_tool_proofs(upstream, mcu, binary, a.mixed_rounds)
        for mode, expected in (
            ("--gpio-mixed", "THERMAL_FIXTURE 13 13 560 -80 0 75"),
            ("--gpio-mixed-shutdown", "THERMAL_FIXTURE_FAULT 13 0 240 240 0 25"),
            ("--gpio-tool-mixed", "TOOL_THERMAL 13 13 13 640 0 0 35 50"),
            ("--gpio-tool-mixed-shutdown", "TOOL_THERMAL_FAULT 13 0 0 240 240 0 25 0"),
        ):
            state = run(binary, mcu, mode)
            assert state.returncode == 0 and state.stderr.strip() == expected, (mode, state.stderr)
        print("Inert mixed heater runs: 13+13 motion commands; injected shutdown held phase two at zero")
        print("Inert T2/T5/T2: 13+13+13 commands; failed second wait held both later phases at zero")

    print("Plain Linux MCU refused before configuration/motion")


if __name__ == "__main__":
    main()
