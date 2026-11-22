#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare pin-free config startup on dedicated Linux MCU test PTYs."""
import argparse
from contextlib import contextmanager
import os
import pty
import select
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import tty
import zlib
from types import SimpleNamespace

from planner import PIN, ROOT


@contextmanager
def fresh(mcu):
    with tempfile.TemporaryDirectory(prefix="klipper-config-fixture-") as directory:
        endpoint = Path(directory) / "mcu"
        with subprocess.Popen([mcu, "-I", endpoint], stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE) as firmware:
            try:
                until = time.monotonic() + 3
                while not endpoint.exists():
                    if firmware.poll() is not None:
                        raise RuntimeError(firmware.stderr.read().decode())
                    if time.monotonic() >= until:
                        raise RuntimeError("fixture PTY did not start")
                    time.sleep(.002)
                with endpoint.open("r+b", buffering=0) as port:
                    tty.setraw(port.fileno())
                    yield port
            finally:
                firmware.terminate()
                try:
                    firmware.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    firmware.kill()
                    firmware.wait(timeout=1)


def from_stock(count):
    from mcu import MCUConfigHelper
    h = MCUConfigHelper.__new__(MCUConfigHelper)
    h._config_callbacks = []
    h._oid_count = count
    h._config_cmds = [f"config_trsync oid={i}" for i in range(count)]
    h._restart_cmds = h._init_cmds = []
    h._printer = SimpleNamespace(lookup_object=lambda key:
                                 SimpleNamespace(get_pin_resolver=lambda name:
                                                 SimpleNamespace(update_command=lambda value: value)))
    h._name = "isolated-test-peer"
    h._finalize_config()
    expected = zlib.crc32("\n".join(h._config_cmds[:-1]).encode())
    assert h._config_crc == expected
    return h._config_cmds, expected


def run_stock(port, cmds):
    from reactor import SelectReactor
    from serialhdl import SerialReader
    reactor = SelectReactor()
    transport = SerialReader(reactor)
    result = []

    def work(_):
        try:
            transport.connect_pipe(port.name)
            initial = transport.send_with_response("get_config", "config")
            assert (initial["is_config"], initial["crc"], initial["is_shutdown"]) == (0, 0, 0)
            for cmd in cmds:
                transport.send(cmd)
            configured = transport.send_with_response("get_config", "config")
            assert configured["is_config"] == 1 and not configured["is_shutdown"]
            result.append(configured)
        except Exception as exc:
            result.append(exc)
        finally:
            transport.disconnect()
            reactor.end()

    reactor.register_callback(work)
    reactor.run()
    assert len(result) == 1
    if isinstance(result[0], Exception):
        raise result[0]
    return result[0]


def drop_write_ack(mcu, binary, mode, target):
    """Drop the first matched write ACK and log any later host frames."""
    with fresh(mcu) as firmware:
        front, client = pty.openpty()
        tty.setraw(client)
        stop = threading.Event()
        observed, errors = [], []
        rx = {front: bytearray(), firmware.fileno(): bytearray()}

        def relay():
            drop = None
            try:
                while not stop.is_set():
                    ready, _, _ = select.select(list(rx), [], [], .05)
                    for fd in ready:
                        chunk = os.read(fd, 512)
                        if not chunk:
                            return
                        data = rx[fd]
                        data.extend(chunk)
                        while data:
                            length = data[0]
                            # Only consumed data from a software fixture.
                            if length == 0x7e:
                                del data[0]
                                continue
                            if not 5 <= length <= 64:
                                raise ValueError("invalid fixture frame")
                            if length > len(data):
                                break
                            frame = bytes(data[:length])
                            del data[:length]
                            if fd == front:
                                if drop is not None:
                                    observed.append(("after", frame[2:-3]))
                                elif target(frame[2:-3]):
                                    observed.append("write")
                                    drop = 0x10 | ((frame[1] + 1) & 15)
                            if fd != front and frame[1] == drop and length == 5:
                                observed.append("dropped")
                                continue
                            dst = firmware.fileno() if fd == front else front
                            while frame:
                                frame = frame[os.write(dst, frame):]
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=relay)
        worker.start()
        try:
            run = subprocess.run([binary, mode, "1"], stdin=client, stdout=client,
                                 stderr=subprocess.PIPE, text=True, timeout=8)
        finally:
            stop.set()
            worker.join(timeout=1)
            os.close(front)
            os.close(client)
        assert not worker.is_alive() and not errors, errors
        assert run.returncode and "unknown config write outcome" in run.stderr, run.stderr
        assert observed == ["write", "dropped"], observed
        return run.stderr


def drop_first_write_ack(mcu, binary, allocate_payload):
    """Drop exactly the ACK following the first pin-free config write."""
    details = drop_write_ack(mcu, binary, "--config-fixture", lambda p: p == allocate_payload)
    assert "CONFIG " not in details and "REUSE " not in details, details
    print("Dropped a config write ACK: one write only, no finalize or retry")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("--payload-binary", type=Path)
    p.add_argument("--dictionary", type=Path)
    p.add_argument("--binary", type=Path, default=ROOT/"target/release/examples/native_mcu")
    a = p.parse_args()
    upstream = a.upstream.resolve()
    sys.path.insert(0, str(upstream / "klippy"))
    from msgproto import MessageParser
    mcu = upstream / "out/klipper.elf"
    dictionary = a.dictionary or upstream / "out/klipper.dict"
    parser = MessageParser()
    parser.process_identify(dictionary.read_bytes(), decompress=False)
    if a.payload_binary:
        rows = subprocess.check_output([a.payload_binary, dictionary], text=True).splitlines()
        total = 0
        for count, row in zip(range(256), rows, strict=True):
            found_count, crc, *encoded = row.split()
            commands, expected_crc = from_stock(count)
            assert (int(found_count), int(crc)) == (count, expected_crc)
            want = [bytes(parser.create_command(cmd)).hex() for cmd in commands]
            assert encoded == want, (count, encoded[-1], want[-1])
            total += len(want)
        print(f"{total} byte-identical starter payloads and 256 stock CRCs")
        return
    for count in (1, 3, 7, 16):
        cmds, expected = from_stock(count)
        with fresh(mcu) as port:
            reference = run_stock(port, cmds)
        with fresh(mcu) as port:
            got = subprocess.run([a.binary, "--config-fixture", str(count)], stdin=port, stdout=port,
                                 stderr=subprocess.PIPE, text=True, timeout=12)
        assert got.returncode == 0, got.stderr
        assert reference["crc"] == expected, reference
        move_count = reference["move_count"]
        slots = move_count - 3
        assert got.stderr.splitlines() == [
            f"CONFIG NeedsConfiguration {expected} {move_count} {slots} {len(cmds)}",
            f"REUSE {expected} {slots} 0",
        ], got.stderr
        print(f"{count} pin-free objects: CRC {expected} and {slots} usable slots match stock; "
              "reuse sends no writes; mismatch refused")
    allocate = bytes(parser.create_command("allocate_oids count=1"))
    drop_first_write_ack(mcu, a.binary, allocate)


if __name__ == "__main__":
    main()
