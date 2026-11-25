#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare matched waves of read-only identify requests on a disposable Linux MCU."""
import argparse
import base64
from contextlib import contextmanager
import json
import os
from pathlib import Path
import pty
import re
import select
import statistics
import subprocess
import sys
import tempfile
import threading
import time
import tty

CHUNK = 16


def stock(endpoint, count, width):
    from reactor import SelectReactor
    from serialhdl import SerialReader

    reactor = SelectReactor()
    serial = SerialReader(reactor)
    result = {}
    failure = []

    def retransmitted_bytes():
        match = re.search(r"\bbytes_retransmit=(\d+)\b", serial.stats(reactor.monotonic()))
        assert match is not None
        return int(match[1])

    def execute(_):
        try:
            serial.connect_pipe(endpoint)
            before = retransmitted_bytes()
            wave = None
            reference = None
            durations = []
            commands = [serial.get_msgparser().create_command(f"identify offset={i * CHUNK} count={CHUNK}")
                        for i in range(width)]

            def receive(params):
                offset, data = params["offset"], bytes(params["data"])
                if len(data) != CHUNK or offset % CHUNK or not 0 <= offset < width * CHUNK:
                    return
                if reference is not None and data != reference[offset:offset + CHUNK]:
                    wave[1].append(f"changed reply at {offset}")
                if offset in wave[0]:
                    wave[1].append(f"duplicate reply at {offset}")
                wave[0][offset] = data
                if len(wave[0]) == width:
                    reactor.async_complete(wave[2], True)

            serial.register_response(receive, "identify_response")
            for _ in range(count // width):
                wave = ({}, [], reactor.completion())
                start = time.perf_counter_ns()
                for cmd in commands:
                    serial.raw_send(cmd, 0, 0, serial.default_cmd_queue)
                if not wave[2].wait(reactor.monotonic() + 1.0):
                    raise RuntimeError(f"stock pipeline timed out: {len(wave[0])}/{width}")
                durations.append(time.perf_counter_ns() - start)
                if wave[1]:
                    raise RuntimeError(wave[1])
                blob = b"".join(wave[0][i * CHUNK] for i in range(width))
                if reference is not None and blob != reference:
                    raise ValueError("stock reply contents changed between waves")
                reference = blob
            result.update(wave_ns=durations, reference=base64.b64encode(reference).decode(),
                          retransmitted_bytes=retransmitted_bytes() - before)
        except Exception as exc:
            failure.append(exc)
        finally:
            serial.disconnect()
            reactor.end()

    reactor.register_callback(execute)
    reactor.run()
    if failure:
        raise failure[0]
    print(json.dumps(result))


@contextmanager
def mcu_endpoint(binary):
    with tempfile.TemporaryDirectory(prefix="klipper-pipeline-") as tmp:
        endpoint = Path(tmp) / "mcu"
        with subprocess.Popen([binary, "-I", endpoint], stderr=subprocess.PIPE,
                              stdout=subprocess.DEVNULL) as firmware:
            try:
                deadline = time.monotonic() + 3
                while not endpoint.exists():
                    if firmware.poll() is not None:
                        raise RuntimeError(firmware.stderr.read().decode())
                    if time.monotonic() > deadline:
                        raise TimeoutError("disposable MCU failed to create a PTY")
                    time.sleep(0.002)
                with endpoint.open("r+b", buffering=0) as fd:
                    tty.setraw(fd.fileno())
                yield endpoint
            finally:
                firmware.terminate()
                try:
                    firmware.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    firmware.kill()
                    firmware.wait()


@contextmanager
def fault_relay(mcu, fault):
    master, client = pty.openpty()
    tty.setraw(client)
    stop = threading.Event()
    altered = []
    failure = []

    def write_all(dst, data):
        while data:
            data = data[os.write(dst, data):]

    def relay():
        buffer = bytearray()
        outbound = fault in ("drop-frame", "crc-frame")
        scan = master if outbound else mcu
        try:
            while not stop.is_set():
                ready, _, _ = select.select([master, mcu], [], [], 0.010)
                for src in ready:
                    data = os.read(src, 4096)
                    dst = mcu if src == master else master
                    if src == scan and not altered:
                        buffer.extend(data)
                        data = bytearray()
                        while buffer and len(buffer) >= buffer[0]:
                            length = buffer[0]
                            if not 5 <= length <= 64:
                                raise ValueError(f"unexpected frame length {length}")
                            frame = bytes(buffer[:length])
                            del buffer[:length]
                            payload = frame[2:-3]
                            match = (len(payload) > 2 and payload[0] == 1 and payload[-1] == CHUNK
                                     if outbound
                                     else len(payload) > CHUNK and payload[0] == 0
                                     and payload[-CHUNK - 1] == CHUNK)
                            if match:
                                altered.append(frame)
                                if fault == "crc-frame":
                                    # Do not change the length, sequence, payload, or sync.
                                    # Send the first two bytes separately, like a partial read.
                                    broken = bytearray(frame)
                                    bit = 1 if broken[-3] ^ 1 != 0x7e else 2
                                    broken[-3] ^= bit
                                    write_all(dst, data)
                                    data.clear()
                                    write_all(dst, broken[:2])
                                    time.sleep(0.001)
                                    write_all(dst, broken[2:])
                                data.extend(buffer)
                                buffer.clear()
                                break
                            data.extend(frame)
                    write_all(dst, data)
        except (OSError, ValueError) as exc:
            failure.append(exc)

    thread = threading.Thread(target=relay)
    thread.start()
    try:
        yield client
    finally:
        stop.set()
        thread.join(timeout=1)
        os.close(master)
        os.close(client)
        if thread.is_alive() or failure or len(altered) != 1:
            raise RuntimeError((thread.is_alive(), failure, len(altered)))


def rust_result(run):
    tag, peak, frames, retries, discarded, invalid, body = run.stderr.strip().split(" ", 6)
    assert tag == "PIPE", run.stderr
    prefix, wave = body.split(" ", 1)
    return dict(peak=int(peak), frames=int(frames), retries=int(retries),
                discarded=int(discarded), invalid=int(invalid), wave_ns=json.loads(wave),
                reference=base64.b64encode(bytes.fromhex(prefix)).decode())


def fresh(args, kind, count, fault=None):
    with mcu_endpoint(args.upstream / "out/klipper.elf") as endpoint:
        if kind == "stock":
            env = os.environ.copy()
            env["PYTHONPATH"] = str(args.upstream / "klippy") + os.pathsep + env.get("PYTHONPATH", "")

            def run_stock(path):
                out = subprocess.check_output([sys.executable, __file__, "--stock", str(path),
                                               str(count), str(args.width)], env=env,
                                              text=True, timeout=30)
                return json.loads(out)

            if fault is None:
                return run_stock(endpoint)
            with endpoint.open("r+b", buffering=0) as stream:
                with fault_relay(stream.fileno(), fault) as client:
                    return run_stock(os.ttyname(client))

        argv = [args.binary, "--pipeline", str(count), str(args.width)]

        def run(fileno):
            return subprocess.run(argv, stdin=fileno, stdout=fileno, stderr=subprocess.PIPE,
                                  text=True, timeout=30)

        with endpoint.open("r+b", buffering=0) as stream:
            if fault is None:
                reply = run(stream.fileno())
            else:
                with fault_relay(stream.fileno(), fault) as client:
                    reply = run(client)
        if fault == "drop-reply":
            assert reply.returncode and "pipeline deadline: 1 replies missing, 0 frames pending" in reply.stderr, reply.stderr
            return None
        if reply.returncode:
            raise RuntimeError(reply.stderr)
        return rust_result(reply)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=lambda p: Path(p).resolve())
    p.add_argument("binary", type=lambda p: Path(p).resolve())
    p.add_argument("--width", type=int, default=128)
    p.add_argument("--waves", type=int, default=64)
    p.add_argument("--rounds", type=int, default=8)
    p.add_argument("--json", type=Path)
    args = p.parse_args()
    assert 32 <= args.width <= 128 and 1 <= args.waves <= 250 and 1 <= args.rounds <= 50
    groups = {kind: [] for kind in ("rust", "stock")}
    golden = fresh(args, "stock", args.width)["reference"]
    first = fresh(args, "rust", args.width)
    assert first["reference"] == golden and first["peak"] > 1, first
    retry = fresh(args, "rust", args.width, "drop-frame")
    assert retry["reference"] == golden and 1 <= retry["retries"] <= 3, retry
    # Crossing 4-bit sequence wrap exposes dropped or duplicate replies.
    fault_count = 32 * args.width
    crc_stock = fresh(args, "stock", fault_count, "crc-frame")
    crc_rust = fresh(args, "rust", fault_count, "crc-frame")
    assert crc_stock["reference"] == crc_rust["reference"] == golden
    assert len(crc_stock["wave_ns"]) == len(crc_rust["wave_ns"]) == 32
    assert crc_stock["retransmitted_bytes"] >= 5, crc_stock
    assert 1 <= crc_rust["retries"] <= 3, crc_rust
    fresh(args, "rust", args.width, "drop-reply")
    for i in range(args.rounds):
        for kind in tuple(groups)[::(-1 if i % 2 else 1)]:
            result = fresh(args, kind, args.width * args.waves)
            assert result.pop("reference") == golden
            assert len(result["wave_ns"]) == args.waves
            if kind == "rust":
                assert result["retries"] == result["discarded"] == result["invalid"] == 0, result
            groups[kind].append(result)
    for kind, results in groups.items():
        times = sorted(t for result in results for t in result["wave_ns"])
        middle, p95 = statistics.median(times), times[(len(times) * 95 + 99) // 100 - 1]
        print(f"{kind}: {len(times)} waves x {args.width} responses; "
              f"median {middle / 1000:.3f} us/wave, p95 {p95 / 1000:.3f} us/wave")
    print(f"Rust max flight {first['peak']} frames in {first['frames']} frames/wave; "
          f"drop-frame recovered with {retry['retries']} retry writes; "
          "drop-reply failed closed at deadline")
    print(f"CRC-corrupted fragmented outbound frame: stock and Rust recovered "
          f"{fault_count} exact replies each; stock replayed "
          f"{crc_stock['retransmitted_bytes']} bytes, Rust retried {crc_rust['retries']} time(s)")
    if args.json:
        args.json.write_text(json.dumps(dict(width=args.width, clean=groups, dropped_frame=retry)) + "\n")


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "--stock":
        stock(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
    else:
        main()
