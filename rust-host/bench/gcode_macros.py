#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Check limited Rust macro expansion against actual Klippy dispatch and Jinja."""
import argparse
import logging
import random
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from planner import PIN, ROOT

CONFIG = """
[gcode_macro PARK]
gcode:
  G90
  SAVE_GCODE_STATE NAME=park
  MOVE_STAGE X={params.X|default(12)} Y={params.Y|default(4)}
  G4 P{params.DELAY|default(15)}
  RESTORE_GCODE_STATE NAME=park
[gcode_macro MOVE_STAGE]
gcode:
  G1 X{params.X} Y{params.Y} F1800
[gcode_macro CHECKPOINT]
gcode:
  M400
[gcode_macro CYCLE_A]
gcode:
  CYCLE_B
[gcode_macro CYCLE_B]
gcode:
  CYCLE_A
[gcode_macro CONDITIONAL]
gcode:
  {% if params.X %}
  G1 X{params.X}
  {% endif %}
[gcode_macro G28]
rename_existing: G28.0
gcode:
  G28.0
"""


class Printer:
    def get_start_args(self):
        return {}

    def get_reactor(self):
        return SimpleNamespace(mutex=lambda: None)

    def register_event_handler(self, *args):
        pass

    def send_event(self, event):
        self.events.append(event)

    def lookup_object(self, name):
        return {"gcode": self.dispatch, "gcode_macro": self.engine}[name]

    def load_object(self, config, name):
        return self.lookup_object(name)


def guide(upstream, config_path):
    sys.path.insert(0, str(upstream / "klippy"))
    from configfile import ConfigFileReader, ConfigWrapper
    from extras.gcode_macro import GCodeMacro, PrinterGCodeMacro
    from gcode import GCodeDispatch

    config = ConfigFileReader().build_fileconfig_with_includes(CONFIG, config_path)
    printer = Printer.__new__(Printer)
    printer.dispatch = GCodeDispatch(printer)
    printer.engine = PrinterGCodeMacro(SimpleNamespace(get_printer=lambda: printer))
    printer.events = []
    seen = []
    for cmd in ("G90", "G1", "M400", "G4", "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE"):
        printer.dispatch.register_command(cmd, lambda c: seen.append(c.get_commandline()))
    for section in config.sections():
        GCodeMacro(ConfigWrapper(printer, config, {}, section))
    printer.dispatch._handle_ready()
    return printer.dispatch, seen


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN} with Jinja2 available")
    a = p.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        d = Path(directory)
        config = d / "printer.cfg"
        config.write_text(CONFIG)
        stock, seen = guide(a.upstream.resolve(), config)
        rng = random.Random(87421)
        lines = []
        for i in range(600):
            if i % 8 == 0:
                lines.append("CHECKPOINT")
            elif i % 3:
                lines.append(f"park x={rng.uniform(5, 20):.3f} Y={rng.uniform(1, 10):.3f} "
                             f"Delay={rng.randint(0, 300)} # comment")
            else:
                lines.append("PARK")
        script = "\n".join(lines) + "\n"
        stock.run_script_from_command(script)
        lib, exe = d / "libklipper_host_protocol.rlib", d / "rust"
        subprocess.run(["rustc", "--edition=2021", "-O", "--crate-type=rlib",
                        "--crate-name=klipper_host_protocol", ROOT / "src/lib.rs", "-o", lib], check=True)
        subprocess.run(["rustc", "--edition=2021", "-O", ROOT / "bench/gcode_macros.rs",
                        "--extern", f"klipper_host_protocol={lib}", "-o", exe], check=True)

        def run(text):
            return subprocess.run([exe, config], input=text, capture_output=True, text=True)

        rust = run(script)
        rust.check_returncode()
        expanded = [line.strip() for line in rust.stdout.splitlines() if line.strip()]
        assert expanded == seen, next((pair for pair in zip(expanded, seen) if pair[0] != pair[1]), None)
        try:
            stock.run_script_from_command("CYCLE_A")
        except stock.error as e:
            assert "recursively" in str(e)
        else:
            raise AssertionError("stock accepted recursive fixture")
        blocked = [
            ("CYCLE_A", "Recursion 1 CYCLE_A"),
            ("CHECKPOINT\nCONDITIONAL X=1", "UnsupportedTemplate 2 CONDITIONAL"),
            ("G28", "UnsupportedTemplate 1 G28"),
            ("MOVE_STAGE", "Arguments 1 MOVE_STAGE"),
        ]
        for unsafe, expected in blocked:
            result = run(unsafe)
            assert (result.returncode, result.stderr.strip(), result.stdout) == (2, expected, "")
    print(f"{len(lines)} macro calls; {len(expanded)} exact dispatched commands; "
          f"{len(blocked)} bounded Rust failures")


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.ERROR)
    main()
