#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare two release builds of the offline status polling fixture."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import statistics
import subprocess
import time


def run(program, capture):
    return subprocess.run([program, *(["--emit"] if capture else [])],
                          capture_output=True, check=True, timeout=40)


def children_cpu():
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    return usage.ru_utime + usage.ru_stime


def compare(old, new, rounds):
    reference = run(old, capture=True)
    candidate = run(new, capture=True)
    if reference.stdout != candidate.stdout or reference.stderr != candidate.stderr:
        raise ValueError("polling wire output differs")
    if not reference.stdout or not reference.stdout.endswith(b"\x03"):
        raise ValueError("expected a nonempty ETX-delimited stream")
    digest = hashlib.sha256(reference.stdout).hexdigest()
    fixture = reference.stderr.decode().strip()
    print(fixture)
    print(f"Verified byte-identical wire SHA-256 {digest}")
    samples = {"baseline": [], "candidate": []}
    programs = {"baseline": old, "candidate": new}
    for i in range(rounds + 1):
        for label in tuple(programs)[::1 if i % 2 == 0 else -1]:
            cpu, wall = children_cpu(), time.perf_counter()
            record = run(programs[label], capture=False)
            elapsed = (children_cpu() - cpu, time.perf_counter() - wall)
            if record.stdout or record.stderr != reference.stderr:
                raise ValueError(f"{label} produced different frame counts in pair {i}")
            if i:
                samples[label].append(elapsed)
    for kind, column in (("CPU", 0), ("elapsed", 1)):
        baseline = [x[column] for x in samples["baseline"]]
        candidate = [x[column] for x in samples["candidate"]]
        changes = [b / a - 1 for a, b in zip(baseline, candidate, strict=True)]
        print(f"{kind}: {statistics.median(baseline)*1000:.3f} → "
              f"{statistics.median(candidate)*1000:.3f} ms; "
              f"paired median {statistics.median(changes)*100:+.2f}% "
              f"({sum(x < 0 for x in changes)}/{rounds} candidate wins)")
    return {"fixture": fixture, "wire_sha256": digest,
            "fields": ["process_cpu_seconds", "elapsed_seconds"], "samples": samples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=lambda x: Path(x).resolve())
    parser.add_argument("candidate", type=lambda x: Path(x).resolve())
    parser.add_argument("--rounds", type=int, default=30)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    if not 1 <= args.rounds <= 100:
        parser.error("--rounds must be between 1 and 100")
    report = compare(args.baseline, args.candidate, args.rounds)
    if args.json:
        args.json.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
