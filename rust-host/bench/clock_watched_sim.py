#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare stock liveness signals to guard placement in an inert job."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from clock_liveness import case
from planner import PIN, ROOT


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("upstream", "dictionary", "replay"):
        p.add_argument(name, type=lambda path: Path(path).resolve())
    a = p.parse_args()
    assert subprocess.check_output(["git", "-C", a.upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(a.upstream / "klippy"))
    import clocksync
    import mcu

    names = ("running", "paused", "ready", "file", "reply-before-five", "finished")
    actions = [(int(n == "file"), "qqqqrcqccrcc" if n == "reply-before-five"
                else "qqqqcqccrcc") for n in names]
    expected = [case(file, line, clocksync, mcu) for file, line in actions]
    data = "".join(f"{file} {line}\n" for file, line in actions).encode()
    with tempfile.TemporaryDirectory(prefix="offline-watched-job-") as tmp:
        c = Path(tmp) / "clock-check-c"
        for flags in ([], ["-DSATURATING"]):
            subprocess.run(["cc", "-O3", *flags, ROOT / "bench/clock_liveness.c", "-o", c],
                           check=True)
            assert subprocess.check_output([c], input=data, timeout=10) == "".join(
                row for cluster in expected for row in cluster).encode()
    result = json.loads(subprocess.check_output([a.replay, a.dictionary], timeout=10))
    assert result["mode"] == "simulation"
    rows = result["cases"]
    assert [item["case"] for item in rows] == list(names)
    for item, policy in zip(rows, expected, strict=True):
        name = item["case"]
        report = int(policy[-1].split()[2])
        prior = 1 if name == "running" else (item["total"] if name == "finished" else 0)
        assert item["total"] == 138 and item["lost_reports"] == report
        assert item["issued_before_check"] == prior and item["late_output"] == 0
        faulted = bool(report and name != "finished")
        assert item["state"] == ("faulted" if faulted else "complete")
        assert item["issued"] == (prior if faulted else item["total"])
        print(name, "policy report", report, "issued", item["issued"], "late", item["late_output"])
    print("6 offline guard cases; stock/C/Rust liveness, unchanged motion prefixes")


if __name__ == "__main__":
    main()
