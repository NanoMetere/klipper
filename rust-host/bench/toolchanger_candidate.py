#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify the exact reviewed upstream toolchanger probe patch in a disposable Klippy."""
import argparse
import importlib
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
from types import SimpleNamespace

from toolchanger_baseline import CASES, KLIPPAIN, LED_EFFECT, MODULES, PORT, check_motion_bridge, pin

CANDIDATE = "0bded502b27dae5b9261789e03b0c8a0b84f48fb"
BASE = "94756dfde9b729fd69f9b8780067821c5c99a528"


class FixtureError(Exception):
    pass


def route(checkout):
    """Exercise actual router/averager objects with inert hardware sessions."""
    sys.path.insert(0, str(checkout/"klippy"))
    mod = importlib.import_module("extras.tool_probe_endstop")
    probe = importlib.import_module("extras.probe")
    manual = importlib.import_module("extras.manual_probe")
    pos = [40., 65., 4.]
    tools = SimpleNamespace(get_position=lambda: list(pos),
                            get_status=lambda _: {"homed_axes":"xyz"},
                            manual_move=lambda coords, speed: pos.__setitem__(slice(None), coords))
    counters, results = [0, 0], []
    printer = SimpleNamespace(
        command_error=FixtureError, get_reactor=lambda: SimpleNamespace(monotonic=lambda: 0.0),
        register_event_handler=lambda *a: None,
        lookup_object=lambda name: tools if name == "toolhead" else SimpleNamespace(respond_info=lambda x: None),
        send_event=lambda *a: None,
    )
    router = mod.ToolProbeEndstop.__new__(mod.ToolProbeEndstop)
    router.active_probe = None
    router.active_tool_number = -1
    router.probe = mod.ProbeRouter(printer)
    router.mcu_probe = mod.EndstopRouter(printer)
    router.probe_offsets = router.param_helper = router.probe
    cfg = SimpleNamespace(get_printer=lambda: printer)
    router.probe_session = probe.SampleAveragingHelper(cfg, router.param_helper,
                                                       router.mcu_probe.start_probe_session)
    params = {"samples": 2, "samples_tolerance": .02, "samples_tolerance_retries": 0,
              "sample_retract_dist": 1., "lift_speed": 4., "samples_result":"average"}
    offsets = [(0.4, -0.7, 1.1), (-0.5, 0.8, 2.2)]
    commands = SimpleNamespace(get_command=lambda: "PROBE", error=FixtureError)
    choices = {}
    for i, offset in enumerate(offsets):
        def start_session(gcmd, i=i, offset=offset):
            counters[i] += 1
            pending = []
            def run_probe(_):
                pending.append(manual.create_probe_result([40.,65.,4.0], offset))
            def pull():
                result = pending[:]
                pending.clear()
                return result
            return SimpleNamespace(run_probe=run_probe, pull_probed_results=pull,
                                   end_probe_session=lambda: pending.clear())
        hw = SimpleNamespace(start_probe_session=start_session,
                             query_endstop=lambda _, i=i: i == 1)
        choices[i] = SimpleNamespace(tool_number=i, mcu_probe=hw,
                      probe_offsets=SimpleNamespace(get_offsets=lambda *args, p=offset: p),
                      param_helper=SimpleNamespace(get_probe_params=lambda *args: params))
    for selection in [0, 1, 0, 1]:
        router.set_active_probe(choices[selection])
        assert router.get_offsets() == offsets[selection]
        assert router.mcu_probe.query_endstop(0.) == (selection == 1)
        session = router.start_probe_session(commands)
        session.run_probe(commands)
        info, = session.pull_probed_results()
        session.end_probe_session()
        want = manual.create_probe_result([40.,65.,4.], offsets[selection])
        assert all(math.isclose(x,y,abs_tol=1e-12) for x,y in zip(info,want,strict=True))
        results.append(info.bed_z)
    assert counters == [2, 2] and all(math.isclose(x, y, abs_tol=1e-12)
                                      for x, y in zip(results, [2.9, 1.8] * 2, strict=True)), results
    router.set_active_probe(None)
    for fn in (lambda: router.start_probe_session(commands),
               lambda: router.mcu_probe.query_endstop(0.),
               lambda: router.get_probe_params(commands)):
        try:
            fn()
        except FixtureError:
            pass
        else:
            raise RuntimeError("inactive tool probe accepted an operation")
    print("T0/T1/T0/T1: independent offsets and two samples each, inactive paths rejected")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for flag in ("port", "dictionary", "klippain", "candidate", "led-effect"):
        p.add_argument(f"--{flag}", type=lambda s: Path(s).resolve(), required=True)
    p.add_argument("--bridge", type=lambda s: Path(s).resolve(),
                   help="optional Rust look-ahead cdylib for a matched debug-motion check")
    a = p.parse_args()
    for worktree, commit in [(a.klippain, KLIPPAIN), (a.candidate, CANDIDATE),
                             (a.led_effect, LED_EFFECT), (a.port, PORT)]:
        pin(worktree, commit)
        subprocess.run(["git","-C",worktree,"diff","--quiet","HEAD","--"],check=True)
    changed = subprocess.check_output(["git","-C",a.candidate,"diff","--name-only",BASE,CANDIDATE],
                                      text=True).splitlines()
    assert changed == ["klipper/extras/tool_probe.py","klipper/extras/tool_probe_endstop.py"]
    sys.path.insert(0, str(a.klippain/"tests"))
    from run_toolchanger_klippy_smoke import run_case
    with tempfile.TemporaryDirectory(prefix="klippain-pr174-smoke-") as directory:
        root = Path(directory)
        checkout = root/"klipper"
        checkout.mkdir()
        with subprocess.Popen(["git","-C",a.port,"archive",PORT],stdout=subprocess.PIPE) as archived:
            assert archived.stdout is not None
            with tarfile.open(fileobj=archived.stdout,mode="r|") as tar:
                tar.extractall(checkout,filter="data")
            archived.stdout.close()
            if archived.wait():
                raise RuntimeError("archive failed")
        for mod in MODULES:
            shutil.copyfile(a.candidate/"klipper/extras"/mod, checkout/"klippy/extras"/mod)
        shutil.copyfile(a.led_effect/"src/led_effect.py",checkout/"klippy/extras/led_effect.py")
        for mechanism, shared in CASES:
            dest = root/(mechanism+("-shared" if shared else ""))
            dest.mkdir()
            run_case(checkout, a.dictionary, mechanism, 2, 300, dest, shared_leds=shared)
            if a.bridge and mechanism == "stealthchanger" and not shared:
                n = check_motion_bridge(checkout, a.dictionary, dest, a.bridge)
                print(f"Modern plugin candidate: {n} equal motor commands with stock/Rust look-ahead")
        route(checkout)


if __name__ == "__main__":
    main()
