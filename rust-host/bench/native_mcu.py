#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare read-only identify/clock/uptime with pinned stock Klippy on fresh Linux MCU PTYs."""
import argparse
import json
import os
import pty
from pathlib import Path
import select
import statistics
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty

PIN = "c9559fa04ec6b11fb1a9a1610903546319fcf310"


def stock(endpoint, count):
    from reactor import SelectReactor
    from serialhdl import SerialReader

    reactor = SelectReactor()
    transport = SerialReader(reactor)
    result = None
    failure = None

    def run(_):
        nonlocal result, failure
        try:
            begin = time.perf_counter_ns()
            transport.connect_pipe(endpoint)
            init_ns = time.perf_counter_ns() - begin
            samples = []
            last_uptime = 0
            for i in range(count):
                uptime = i % 8 == 0
                response = "uptime" if uptime else "clock"
                begin = time.perf_counter_ns()
                reply = transport.send_with_response(f"get_{response}", response)
                samples.append(time.perf_counter_ns() - begin)
                if uptime:
                    ticks = (reply["high"] << 32) | reply["clock"]
                    if ticks < last_uptime:
                        raise RuntimeError("uptime went backward")
                    last_uptime = ticks
                else:
                    assert 0 <= reply["clock"] < (1 << 32)
            result = [init_ns, samples]
        except Exception as exc:
            failure = exc
        finally:
            transport.disconnect()
            reactor.end()

    reactor.register_callback(run)
    reactor.run()
    if failure:
        raise failure
    print(json.dumps(result))


CORRUPTION = b"\xff" * 80 + b"\x7e"


def rust_run(binary, count, stream, inject):
    if not inject:
        return subprocess.run([binary, str(count)], stdin=stream, stdout=stream,
                              stderr=subprocess.PIPE, timeout=30, text=True, check=False)
    host, client = pty.openpty()
    tty.setraw(client, termios.TCSANOW)
    stop = threading.Event()
    errors = []

    def relay():
        corrupt = True
        try:
            while not stop.is_set():
                ready, _, _ = select.select([host, stream], [], [], 0.05)
                for src in ready:
                    data = os.read(src, 1024)
                    if src == stream:
                        dst = host
                        if corrupt and data:
                            data = CORRUPTION + data
                            corrupt = False
                    else:
                        dst = stream
                    while data:
                        data = data[os.write(dst, data):]
        except OSError as exc:
            errors.append(exc)

    thread = threading.Thread(target=relay, daemon=True)
    thread.start()
    try:
        run = subprocess.run([binary, str(count)], stdin=client, stdout=client,
                             stderr=subprocess.PIPE, timeout=30, text=True, check=False)
    finally:
        stop.set()
        thread.join(timeout=1)
        os.close(host)
        os.close(client)
    if errors:
        raise errors[0]
    if thread.is_alive():
        raise RuntimeError("virtual relay did not stop")
    return run


def fresh(mcu, binary, upstream, count, kind, inject=False):
    # No operating printer, GPIO outputs, realtime scheduling, or watchdog.
    with tempfile.TemporaryDirectory(prefix="klipper-rust-pty-") as tmp:
        endpoint = Path(tmp) / "mcu"
        with subprocess.Popen([mcu, "-I", endpoint], stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE) as fw:
            try:
                until = time.monotonic() + 3
                while not endpoint.exists():
                    if fw.poll() is not None:
                        raise RuntimeError(f"firmware exited: {fw.stderr.read().decode()}")
                    if time.monotonic() >= until:
                        raise TimeoutError("firmware PTY creation")
                    time.sleep(0.002)
                with endpoint.open("r+b", buffering=0) as endpoint_setup:
                    tty.setraw(endpoint_setup.fileno(), termios.TCSANOW)
                if kind == "rust":
                    with endpoint.open("r+b", buffering=0) as stream:
                        run = rust_run(binary, count, stream.fileno(), inject)
                    if run.returncode:
                        raise RuntimeError(run.stderr)
                    init, discarded, samples = run.stderr.strip().split(" ", 2)
                    expected_discarded = len(CORRUPTION) if inject else 0
                    if int(discarded) != expected_discarded:
                        raise RuntimeError(f"expected {expected_discarded} discarded bytes, got {discarded}")
                    return [int(init), json.loads(samples)]
                env = os.environ.copy()
                env["PYTHONPATH"] = str(upstream / "klippy") + os.pathsep + env.get("PYTHONPATH", "")
                run = subprocess.run([sys.executable, __file__, "--stock", str(endpoint), str(count)],
                                     env=env, capture_output=True, timeout=30, text=True, check=False)
                if run.returncode:
                    raise RuntimeError(run.stderr)
                return json.loads(run.stdout)
            finally:
                fw.terminate()
                try:
                    fw.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    fw.kill()
                    fw.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path, help=f"disposable Klipper tree at {PIN}")
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--rounds", type=int, default=6)
    parser.add_argument("--json", type=Path, help="save all raw measurements")
    args = parser.parse_args()
    if not 1 <= args.samples <= 100_000 or args.rounds < 1:
        parser.error("positive sample and round counts required")
    upstream = args.upstream.resolve()
    config = upstream / ".config"
    if "CONFIG_MACH_LINUX=y" not in config.read_text().splitlines():
        parser.error("build the upstream native Linux MCU first")
    mcu = upstream / "out" / "klipper.elf"
    data = {"rust": [], "stock": []}
    # Prime the C helper build and both implementations before collected rounds.
    for kind in data:
        fresh(mcu, args.binary.resolve(), upstream, 8, kind)
    fresh(mcu, args.binary.resolve(), upstream, 32, "rust", inject=True)
    print(f"recovery: discarded {len(CORRUPTION)} injected bytes and completed 32 exchanges")
    for index in range(args.rounds):
        for kind in list(data)[::(-1 if index % 2 else 1)]:
            data[kind].append(fresh(mcu, args.binary.resolve(), upstream, args.samples, kind))
    for kind, runs in data.items():
        samples = sorted(ns for _, group in runs for ns in group)
        assert len(samples) == args.rounds * args.samples
        rank = (len(samples) * 95 + 99) // 100 - 1
        print(f"{kind}: {len(samples)} exchanges, "
              f"identify median {statistics.median(x[0] for x in runs) / 1e6:.3f} ms, "
              f"RTT median {statistics.median(samples) / 1e3:.3f} us, "
              f"p95 {samples[rank] / 1e3:.3f} us")
    if args.json:
        args.json.write_text(json.dumps(data) + "\n")


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--stock":
        stock(sys.argv[2], int(sys.argv[3]))
    else:
        main()
