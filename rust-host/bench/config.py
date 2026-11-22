#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Validate in-memory config resolution against pinned Klipper ConfigFileReader."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

from planner import PIN, ROOT

FILES = {
    "printer.cfg": ("[DEFAULT]\nPIN=PB1\n"
                    "[printer]\nx:1\n[include extras/[abc]?.cfg]\n"
                    "[include empty/*.cfg]\n[printer]\nx:7\n"
                    "setting=https://example.com;path ; comment\n"
                    "[include extras/.hidden.cfg]\n[include other/?/board.cfg]\n"),
    "extras/a1.cfg": "[printer]\ny:6\nx:4\n[include ../macros.cfg]",
    "extras/c1.cfg": "[printer]\nx:5\ny:3\n[gcode_macro C]\ngcode: G91",
    "extras/b1.cfg": "[printer]\nx:2\n[include other.cfg]",
    "extras/.hidden.cfg": "[gcode_macro HIDDEN]\ngcode: G21",
    "extras/.ignored.cfg": "[unused]\nsetting: hidden",
    "extras/other.cfg": "[printer]\ny:11\n",
    "other/a/board.cfg": "[mcu]\nserial: /dev/example",
    "macros.cfg": ("[gcode_macro PARK]\ngcode:  \n"
                   "    {% set x = 2 %}\n\n"
                   "    G1 X{x} ; trailing comment\n"
                   "    # stripped by Klipper before configparser\n"
                   "    G1 Y10\n"),
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    args = p.parse_args()
    upstream = args.upstream.resolve()
    sys.path.insert(0, str(upstream / "klippy"))
    from configfile import ConfigFileReader
    reader = ConfigFileReader()

    def string(s):
        return s.encode().hex()

    def expect(root, entries):
        for key in entries:
            path = root / key
            config = reader.build_fileconfig_with_includes(reader.read_config_file(path), path)
            yield f"ENTRY {string(key)}"
            for name in config.sections():
                yield f"SECTION {string(name)}"
                for option, value in sorted(config.items(name)):
                    yield f"{string(option)}={string(value)}"

    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        src = d / "fixture"
        for key, data in FILES.items():
            path = src / key
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(data)
        exe, lib = d / "rust", d / "libklipper_host_protocol.rlib"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/config.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", exe], check=True)
        total_sections = total_options = 0
        groups = ((src, ["printer.cfg"]),
                  (upstream / "config", sorted(str(p.relative_to(upstream / "config"))
                   for p in (upstream / "config").rglob("*.cfg") if p.is_file() and not p.is_symlink())))
        for root, entries in groups:
            want = list(expect(root, entries))
            got = subprocess.check_output([exe, root], text=True, input="\n".join(entries) + "\n").splitlines()
            for i, (a, b) in enumerate(zip(want, got, strict=True)):
                assert a == b, (i, a, b)
            total_sections += sum(row.startswith("SECTION ") for row in want)
            total_options += sum("=" in row for row in want)
        print(f"{sum(len(entries) for _, entries in groups)} inputs, {total_sections} sections, {total_options} exact options")


if __name__ == "__main__":
    main()
