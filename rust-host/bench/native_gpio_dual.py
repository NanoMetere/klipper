# SPDX-License-Identifier: GPL-3.0-only
"""Two independently started, attested software-only MCU peers."""
from contextlib import contextmanager
import os
import pty
import select
import statistics
import subprocess
import sys
import threading
import tty

from mcu_config import fresh


@contextmanager
def observed_peer(firmware, label, events, fault=None):
    """Record framed traffic and optionally lose one matching test ACK."""
    with fresh(firmware) as port:
        front, endpoint = pty.openpty()
        tty.setraw(endpoint)
        back = port.fileno()
        stop = threading.Event()
        errors = []
        buffers = {front: bytearray(), back: bytearray()}

        def relay():
            drop_sequence = None
            dropped = False
            try:
                while True:
                    ready, _, _ = select.select(list(buffers), [], [], 0 if stop.is_set() else .05)
                    if not ready and stop.is_set():
                        break
                    for fd in ready:
                        received = os.read(fd, 512)
                        if not received:
                            return
                        raw = buffers[fd]
                        raw.extend(received)
                        while raw:
                            n = raw[0]
                            if n == 0x7e:
                                raw.pop(0)
                                continue
                            if not 5 <= n <= 64:
                                raise ValueError(f"invalid fixture length {n}")
                            if len(raw) < n:
                                break
                            frame = bytes(raw[:n])
                            del raw[:n]
                            if fd == front:
                                payload = frame[2:-3]
                                events.append(("send", label, payload))
                                if fault and not dropped and drop_sequence is None and fault(payload):
                                    drop_sequence = 0x10 | ((frame[1]+1)&15)
                            elif n == 5 and drop_sequence == frame[1]:
                                dropped = True
                                drop_sequence = None
                                events.append(("lost", label, b""))
                                continue
                            dst = back if fd == front else front
                            while frame:
                                frame = frame[os.write(dst, frame):]
            except Exception as e:
                errors.append(e)

        worker = threading.Thread(target=relay)
        worker.start()
        try:
            yield endpoint
        finally:
            stop.set()
            worker.join(timeout=1)
            os.close(endpoint)
            os.close(front)
            assert not worker.is_alive() and not errors, errors


def check_dual(upstream, mcu, binary, rounds=1):
    sys.path.insert(0, str(upstream / "klippy"))
    from msgproto import MessageParser

    def proc(primary, secondary, batched=False):
        a = secondary if isinstance(secondary, int) else secondary.fileno()
        mode = "--gpio-dual-packed" if batched else "--gpio-dual"
        return subprocess.run([binary, mode, str(a)], stdin=primary, stdout=primary,
                              pass_fds=(a,), stderr=subprocess.PIPE, text=True, timeout=12)

    def unpack(result):
        assert result.returncode == 0, result.stderr
        info, enqueue = result.stderr.strip().splitlines()
        tag, a, b, *positions = info.split()
        assert tag == "GPIO_DUAL" and int(a) > 0 and int(b) > 0, result.stderr
        assert list(map(int, positions)) == [20, 20, 0, -25, 50], result.stderr
        timer, f0, f1, nanos = enqueue.split()
        assert timer == "DUAL_ENQUEUE" and int(nanos) > 0, result.stderr
        assert 0 < int(f0) <= int(a) and 0 < int(f1) <= int(b), result.stderr
        return (int(a), int(b)), (int(f0), int(f1)), int(nanos)

    timing = {False: [], True: []}
    summary = {}
    for i in range(rounds):
        for batch in (False, True)[::1 if i % 2 == 0 else -1]:
            with fresh(mcu) as primary, fresh(mcu) as secondary:
                counts, frames, elapsed = unpack(proc(primary, secondary, batch))
            assert summary.setdefault(batch, (counts, frames)) == (counts, frames)
            timing[batch].append(elapsed)
    plain, packed = summary[False], summary[True]
    assert plain[0] == packed[0] and plain[0] == plain[1]
    assert all(p < q for p,q in zip(packed[1], plain[1], strict=True)), summary
    before, after = [statistics.median(timing[b])*1e-6 for b in (False, True)]
    print(f"Two fresh MCU PTYs, different virtual clock origins: {plain[0]} commands, "
          f"{plain[1]} vs {packed[1]} frames, five counters at 17 shared checkpoints")
    print(f"{rounds} paired runs: median host enqueue {before:.3f} -> {after:.3f} ms")

    parser = MessageParser()
    parser.process_identify((mcu.parent / "klipper.dict").read_bytes(), decompress=False)
    tag = lambda name: bytes(parser.messages_by_name[name].msgid_bytes)
    ident = tag("identify")
    motion = (tag("queue_step"), tag("set_next_step_dir"))
    advance = tag("rust_fixture_advance")
    traces = []
    for batch in (False, True):
        events = []
        with observed_peer(mcu, 0, events) as primary, observed_peer(mcu, 1, events) as secondary:
            out = proc(primary, secondary, batch)
        assert unpack(out)[:2] == summary[batch]
        trace = []
        for typ, board, payload in events:
            if typ=="send" and payload.startswith(motion):
                rows = parser.dump(bytes([len(payload)+5, 16, *payload, 0, 0, 126]))[1:]
                assert all(row.split()[0] in ("queue_step","set_next_step_dir") for row in rows)
                trace.extend((board, row) for row in rows)
        traces.append(trace)
    assert traces[0] == traces[1] and len(traces[0]) == sum(plain[0])
    print("Cross-board motion command order is identical before and after packing")

    events = []
    with observed_peer(mcu, 0, events) as primary, fresh(upstream / "out/klipper.elf") as normal:
        rejected = proc(primary, normal, True)
    assert rejected.returncode and "NotFixture" in rejected.stderr, rejected.stderr
    assert events and all(typ == "send" and p.startswith(ident) for typ, _, p in events), events
    print("Unmarked peer refused before the marked peer receives configuration")

    for batch in (False, True):
        events = []
        with observed_peer(mcu, 0, events) as primary, \
                observed_peer(mcu, 1, events, lambda p: p.startswith(motion)) as secondary:
            fail = proc(primary, secondary, batch)
        assert fail.returncode and "peer 1 motion: unknown config write outcome" in fail.stderr, fail.stderr
        assert events[-1] == ("lost", 1, b""), events[-8:]
        assert not any(p.startswith(advance) for _, _, p in events), events[-8:]
        assert sum(typ == "send" and board == 1 and p.startswith(motion) for typ, board, p in events) == 1
    print("Lost secondary motion ACK in either mode: no retry, later writes, or clock advances")
