#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare trigger scheduling and pin-free simulated MCU terminal reasons."""
import argparse
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import patch

from mcu_config import fresh, from_stock
from planner import PIN, ROOT


def timing(upstream, executable):
    from mcu import MCU_trsync, TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT
    rng = random.Random(5903)
    cases, want = [], []
    for i in range(3600):
        hz = rng.choice([1e6, 12e6, 16e6, 72e6, 84e6, 133e6, 180e6, 400e6])
        start = rng.randint(1, 16_000_000_000)
        peers = 1 if i % 5 == 0 else rng.randint(2, 40)
        peer = rng.randrange(peers)
        ts = MCU_trsync.__new__(MCU_trsync)
        ts._oid = 0
        ts._trdispatch_mcu = object()
        ts._steppers = []
        ts._mcu = SimpleNamespace(
            print_time_to_clock=lambda unused: start,
            seconds_to_clock=lambda t: int(t * hz),
            register_serial_response=lambda *args: None,
        )
        arm, expiry, setup = [], [], []
        ts._trsync_start_cmd = SimpleNamespace(send=lambda fields, **kwargs: arm.extend(fields))
        ts._trsync_set_timeout_cmd = SimpleNamespace(send=lambda fields, **kwargs: expiry.extend(fields))
        with patch("chelper.get_ffi", return_value=(None, SimpleNamespace(
            trdispatch_mcu_setup=lambda _, *args: setup.extend(args)
        ))):
            ts.start(0., peer / peers, None,
                     TRSYNC_SINGLE_MCU_TIMEOUT if peers == 1 else TRSYNC_TIMEOUT)
        assert arm[-1] == MCU_trsync.REASON_COMMS_TIMEOUT and expiry[-1] == setup[1]
        cases.append(f"{start} {hz} {peer} {peers}")
        want.append(f"{arm[1]} {arm[2]} {expiry[-1]} {setup[-1]}")
    out = subprocess.check_output([executable], text=True, input="\n".join(cases)+"\n")
    for i, (actual, expected) in enumerate(zip(out.splitlines(), want, strict=True)):
        assert actual == expected, (cases[i], actual, expected)
    print(f"{len(cases)} exact stock timing vectors")


def native_stock(port, cmds):
    from reactor import SelectReactor
    from serialhdl import SerialReader, SerialRetryCommand
    reactor = SelectReactor()
    serial = SerialReader(reactor)
    result = []

    def work(_):
        try:
            serial.connect_pipe(port.name)
            for cmd in cmds:
                serial.send(cmd)
            configured = serial.send_with_response("get_config", "config")
            assert configured["is_config"] == 1 and not configured["is_shutdown"]
            arm = "trsync_start oid=0 report_clock=0 report_ticks=0 expire_reason=4"
            stop = "trsync_trigger oid=0 reason=2"
            serial.send(arm)
            stop_bytes = serial.get_msgparser().create_command(stop)
            def read_stop():
                handler = SerialRetryCommand(serial, "trsync_state", oid=0)
                return handler.get_response([stop_bytes], serial.default_cmd_queue)
            host = read_stop()
            assert (host["oid"], host["can_trigger"], host["clock"]) == (0, 0, 0)
            serial.send(arm)
            now = serial.send_with_response("get_clock", "clock")["clock"]
            hz = serial.get_msgparser().get_constant_float("CLOCK_FREQ")
            deadline = (now + int(hz * .15)) & 0xffffffff
            unsolicited = []
            serial.register_response(unsolicited.append, "trsync_state", oid=0)
            serial.send(f"trsync_set_timeout oid=0 clock={deadline}")
            reactor.pause(reactor.monotonic() + .35)
            serial.register_response(None, "trsync_state", oid=0)
            assert any(s["trigger_reason"] == 4 for s in unsolicited), unsolicited
            expired = read_stop()
            assert (expired["oid"], expired["can_trigger"], expired["clock"]) == (0, 0, 0)
            result.append((host["trigger_reason"], expired["trigger_reason"]))
        except Exception as e:
            result.append(e)
        finally:
            serial.disconnect()
            reactor.end()

    reactor.register_callback(work)
    reactor.run()
    assert len(result) == 1
    if isinstance(result[0], Exception):
        raise result[0]
    return result[0]


def native(upstream, executable):
    mcu = upstream / "out/klipper.elf"
    cmds, _ = from_stock(1)
    with fresh(mcu) as port:
        expected = native_stock(port, cmds)
    with fresh(mcu) as port:
        run = subprocess.run([executable, "--trigger-fixture", "1"], stdin=port, stdout=port,
                             stderr=subprocess.PIPE, text=True, timeout=10)
    assert run.returncode == 0, run.stderr
    terminal = [line.split() for line in run.stderr.splitlines() if line.startswith("TRIGGER")]
    assert [r[1] for r in terminal] == ["host", "timeout"], terminal
    assert tuple(int(r[2]) for r in terminal) == expected == (2, 4), terminal
    print("Rust and stock native PTY: host-trigger reason 2, expiry reason 4")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/trigger_sync_replay")
    p.add_argument("--native", action="store_true", help="do not read physical sensors; use Linux MCU PTY")
    a = p.parse_args()
    u = a.upstream.resolve()
    sys.path.insert(0, str(u/"klippy"))
    if a.native:
        native(u, a.binary)
    else:
        timing(u, a.binary)


if __name__ == "__main__":
    main()
