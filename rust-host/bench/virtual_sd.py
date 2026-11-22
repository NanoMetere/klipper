#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Run stock virtual SD with inert in-memory text and fake G-code callbacks."""
import argparse
import io
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace

from planner import PIN, ROOT


class GcodeError(Exception):
    pass


def stock(impl, content, start, pause, rewind, fail):
    records = []
    sd = impl.__new__(impl)
    sd.current_file = io.TextIOWrapper(io.BytesIO(content), newline="")
    sd.file_position = 0
    sd.file_size = len(content)
    sd.must_pause_work = False
    sd.cmd_from_sd = False
    sd.work_timer = object()
    sd.next_file_position = 0
    sd.on_error_gcode = SimpleNamespace(render=lambda: "inert on-error callback")
    sd.reactor = SimpleNamespace(
        NOW=0, NEVER=1e20, unregister_timer=lambda _: None, pause=lambda _: None)
    step = 0
    def dispatch(line):
        nonlocal step
        if line == "inert on-error callback":
            return
        assert sd.is_cmd_from_sd()
        step += 1
        records.append(f"{sd.file_position} {sd.get_file_position()} {line.encode().hex()}")
        if step == fail:
            raise GcodeError("synthetic dispatch error")
        if step == rewind:
            sd.set_file_position(start)
        if step == pause:
            sd.must_pause_work = True

    sd.gcode = SimpleNamespace(
        get_mutex=lambda: SimpleNamespace(test=lambda: False), error=GcodeError,
        respond_raw=lambda _: None, run_script=dispatch)
    sd.print_stats = SimpleNamespace(
        note_start=lambda: None,
        note_pause=lambda: records.append(f"PAUSE {sd.file_position}"),
        note_error=lambda _: records.append(f"Faulted {sd.file_position}"),
        note_complete=lambda: records.append(f"Completed {sd.file_position}"))
    sd.file_position = start
    sd.work_handler(0)
    if sd.must_pause_work:
        sd.work_timer = object()
        sd.must_pause_work = False
        sd.work_handler(0)
    if sd.current_file:
        sd.current_file.close()
    return records


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/virtual_sd_replay")
    a = p.parse_args()
    sys.path.insert(0, str(a.upstream.resolve() / "klippy"))
    from extras.virtual_sdcard import VirtualSD

    rng = random.Random(3997)
    count = 0
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "input.gcode"
        for case in range(64):
            words = ["G90", "; 한글 목록", "M400", "G1 X3 Y2", "; µ温度", ""]
            data = [rng.choice(words) + rng.choice(["\r\n", "\n", "\n"]) for _ in range(330)]
            data += ["G91" if case % 3 else "G91\n"]
            raw = "".join(data).encode()
            start = len("".join(data[:case % 20]).encode())
            pause = 32 if case % 3 == 0 else 0
            rewind = 48 if case % 5 == 0 else 0
            fail = 181 if case % 7 == 0 else 0
            path.write_bytes(raw)
            got = subprocess.check_output([a.binary, path, str(start), str(pause), str(rewind), str(fail)],
                                          text=True).splitlines()
            want = stock(VirtualSD, raw, start, pause, rewind, fail)
            for index, (x, y) in enumerate(zip(want, got, strict=True)):
                assert x == y, (case, index, x, y)
            count += len(got)
    print(f"{count} exact line/byte checkpoints and terminal or pause observations across 64 files")


if __name__ == "__main__":
    main()
