#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare read-only identify/clock/uptime with pinned stock Klippy on fresh Linux MCU PTYs."""
import argparse
import importlib.util
import json
import math
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


def rust_run(binary, count, stream, fault, trace=False, fw=None, ids=None):
    argv = [binary] + (["--clock-trace"] if trace else []) + [str(count)]
    if fault is None:
        return subprocess.run(argv, stdin=stream, stdout=stream,
                              stderr=subprocess.PIPE, timeout=30, text=True, check=False)
    host, client = pty.openpty()
    tty.setraw(client, termios.TCSANOW)
    stop = threading.Event()
    errors = []
    host_closed = threading.Event()

    def relay():
        injected = False
        cut = bytearray()
        matching = 0
        try:
            while not stop.is_set():
                ready, _, _ = select.select([host, stream], [], [], 0.05)
                for src in ready:
                    data = os.read(src, 1024)
                    if src == stream:
                        dst = host
                        if fault == "corruption" and not injected and data:
                            data = CORRUPTION + data
                            injected = True
                        elif fault in ("reply", "clock_reply") and not injected:
                            cut.extend(data)
                            data = bytearray()
                            while cut and len(cut) >= cut[0]:
                                length = cut[0]
                                if not 5 <= length <= 64:
                                    raise ValueError("firmware produced invalid frame length")
                                # Bootstrap response ID zero; live clock ID from the native dictionary.
                                if length > 5 and (cut[1:3] == b"\x11\x00" if fault == "reply"
                                                   else cut[2] == ids[1]):
                                    matching += 1
                                drop = not injected and matching == (1 if fault == "reply" else 2)
                                if drop:
                                    injected = True
                                else:
                                    data.extend(cut[:length])
                                del cut[:length]
                            if injected:
                                data.extend(cut)
                                cut.clear()
                    else:
                        dst = stream
                        if fault == "silent":
                            continue
                        if fault == "request" and not injected:
                            cut.extend(data)
                            if not cut or len(cut) < cut[0]:
                                continue
                            if not 5 <= cut[0] <= 64:
                                raise ValueError("host produced invalid frame length")
                            data = bytes(cut[cut[0]:])
                            cut.clear()
                            injected = True
                        elif fault in ("clock_request", "restart") and not injected:
                            cut.extend(data)
                            data = bytearray()
                            while cut and len(cut) >= cut[0]:
                                length = cut[0]
                                if not 5 <= length <= 64:
                                    raise ValueError("host produced invalid frame length")
                                if length == 6 and cut[2] == ids[0]:
                                    matching += 1
                                if not injected and matching == 2:
                                    injected = True
                                    if fault == "restart":
                                        # The second clock request is pending in Rust and not
                                        # forwarded. Destroy its device and peer, then require
                                        # the next run to start on a fresh link and seed.
                                        fw.terminate()
                                        os.close(host)
                                        host_closed.set()
                                        return
                                else:
                                    data.extend(cut[:length])
                                del cut[:length]
                            if injected:
                                data.extend(cut)
                                cut.clear()
                    while data:
                        data = data[os.write(dst, data):]
        except (OSError, ValueError) as exc:
            errors.append(exc)

    thread = threading.Thread(target=relay, daemon=True)
    thread.start()
    try:
        run = subprocess.run(argv, stdin=client, stdout=client,
                             stderr=subprocess.PIPE, timeout=30, text=True, check=False)
    finally:
        stop.set()
        thread.join(timeout=1)
        if not host_closed.is_set():
            os.close(host)
        os.close(client)
    if errors:
        raise errors[0]
    if thread.is_alive():
        raise RuntimeError("virtual relay did not stop")
    return run


def fresh(mcu, binary, upstream, count, kind, fault=None, trace=False, ids=None):
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
                        begin = time.monotonic()
                        run = rust_run(binary, count, stream.fileno(), fault, trace, fw, ids)
                    if fault == "restart":
                        if not run.returncode or run.stderr.count("CLOCK ") != 1 or "DONE" in run.stderr:
                            raise RuntimeError(f"pending query survived lost device: {run.stderr}")
                        if not any(marker in run.stderr for marker in ["PTY EOF", "Input/output error"]):
                            raise RuntimeError(f"expected disconnect error, got {run.stderr}")
                        return run.stderr
                    if fault == "silent":
                        if run.returncode == 0 or "NoAck" not in run.stderr:
                            raise RuntimeError(f"expected explicit NoAck, got {run.stderr}")
                        if time.monotonic() - begin > 3:
                            raise RuntimeError("silent peer did not fail promptly")
                        return None
                    if run.returncode:
                        raise RuntimeError(run.stderr)
                    if trace:
                        return run.stderr
                    init, discarded, frame, reply, samples = run.stderr.strip().split(" ", 4)
                    expected = {"corruption": (len(CORRUPTION), 0, 0),
                                "request": (0, 1, 0), "reply": (0, 0, 1)}.get(fault, (0, 0, 0))
                    actual = (int(discarded), int(frame), int(reply))
                    if actual != expected:
                        raise RuntimeError(f"{fault}: expected counters {expected}, got {actual}")
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


def compare_trace(upstream, trace, count, fault):
    spec = importlib.util.spec_from_file_location("stock_clocksync", upstream / "klippy/clocksync.py")
    stock = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(stock)
    from clock_sync import run_stock
    lines = [line.split() for line in trace.splitlines()]
    assert len(lines) == count + 2 and lines[0][0] == "SEED" and lines[-1][0] == "DONE", trace
    _, hz, ticks, sent = lines[0]
    seed = (float(hz), int(ticks), float(sent))
    rx = []
    for line in lines[1:-1]:
        assert line[0] == "CLOCK"
        low, sent, recv, warm = int(line[1]), float(line[2]), float(line[3]), bool(int(line[4]))
        assert 1.0 <= recv and 0 <= sent <= recv
        rx.append((low, sent, recv, warm))
    assert [warm for _, _, _, warm in rx] == [i < 8 for i in range(count)]
    assert rx[-1][2] - rx[7][2] >= (count - 8) * 0.98
    expected_retries = {"clock_request": (1, 0), "clock_reply": (0, 1)}.get(fault, (0, 0))
    assert tuple(map(int, lines[-1][1:])) == expected_retries, trace
    assert [i for i, row in enumerate(rx) if row[1] == 0.0] == ([1] if fault == "clock_request" else []), trace
    # Same observed boundaries, clock values, and seed: arithmetic parity, not
    # a claim about timing accuracy across independent PTYs.
    for got, (want, kind, release) in zip(lines[1:-1], run_stock(stock, seed, rx)):
        assert int(got[5]) == want[0] and got[6] == kind, (got, want, kind)
        for a, b in zip(map(float, got[7:10]), want[1:4]):
            assert math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-8), (got, want)
        if release is not None:
            assert int(got[11]) == release[1], (got, release)
            for a, b in zip([float(got[10]), float(got[12])], [release[0], release[2]]):
                assert math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-8), (got, release)
        else:
            assert len(got) == 10
    return seed


def live_clock(mcu, binary, upstream, trace_output=None):
    dictionary = json.loads((upstream / "out" / "klipper.dict").read_bytes())
    cmd = dictionary["commands"]["get_clock"]
    reply = dictionary["responses"]["clock clock=%u"]
    if not (0 <= cmd < 96 and -32 <= reply < 96):
        raise ValueError("live fault injection expects one-byte clock IDs")
    ids = (cmd, reply & 127)
    logs = {}
    for case in [None, "clock_request", "clock_reply"]:
        label = case or "clean"
        logs[label] = fresh(mcu, binary, upstream, 10, "rust", fault=case, trace=True, ids=ids)
        compare_trace(upstream, logs[label], 10, case)
        print(f"{label}: 10 observed samples agree with pinned stock model")
    logs["interrupted"] = fresh(mcu, binary, upstream, 10, "rust", fault="restart", trace=True, ids=ids)
    # Start an independent
    # link. No timestamps or wire sequence from the failed process carry over.
    logs["reconnected"] = fresh(mcu, binary, upstream, 10, "rust", trace=True, ids=ids)
    compare_trace(upstream, logs["reconnected"], 10, None)
    print("reconnect: pending query failed on disconnect; new session seeded and calibrated from uptime")
    if trace_output:
        trace_output.write_text(json.dumps(logs, indent=2) + "\n")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path, help=f"disposable Klipper tree at {PIN}")
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=512)
    parser.add_argument("--rounds", type=int, default=6)
    parser.add_argument("--json", type=Path, help="save all raw measurements or live-clock traces")
    parser.add_argument("--live-clock", action="store_true", help="test disposable live clock and reconnect")
    args = parser.parse_args()
    if not 1 <= args.samples <= 100_000 or args.rounds < 1:
        parser.error("positive sample and round counts required")
    upstream = args.upstream.resolve()
    config = upstream / ".config"
    if "CONFIG_MACH_LINUX=y" not in config.read_text().splitlines():
        parser.error("build the upstream native Linux MCU first")
    mcu = upstream / "out" / "klipper.elf"
    if args.live_clock:
        live_clock(mcu, args.binary.resolve(), upstream, args.json)
        return
    data = {"rust": [], "stock": []}
    # Prime the C helper build and both implementations before collected rounds.
    for kind in data:
        fresh(mcu, args.binary.resolve(), upstream, 8, kind)
    for fault in ["corruption", "request", "reply", "silent"]:
        fresh(mcu, args.binary.resolve(), upstream, 32, "rust", fault=fault)
        print(f"fault case passed: {fault}")
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
