# SPDX-License-Identifier: GPL-3.0-only
"""Two independently started, attested software-only MCU peers."""
from contextlib import contextmanager
import os
import pty
import select
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


def check_dual(upstream, mcu, binary):
    sys.path.insert(0, str(upstream / "klippy"))
    from msgproto import MessageParser

    def proc(primary, secondary):
        a = secondary if isinstance(secondary, int) else secondary.fileno()
        return subprocess.run([binary, "--gpio-dual", str(a)], stdin=primary, stdout=primary,
                              pass_fds=(a,), stderr=subprocess.PIPE, text=True, timeout=12)

    with fresh(mcu) as primary, fresh(mcu) as secondary:
        out = proc(primary, secondary)
    assert out.returncode == 0, out.stderr
    tag, a, b, *positions = out.stderr.strip().split()
    assert tag == "GPIO_DUAL" and int(a) > 0 and int(b) > 0, out.stderr
    assert list(map(int, positions)) == [20, 20, 0, -25, 50], out.stderr
    print(f"Two isolated PTYs, different virtual clock origins: {a}+{b} motion commands and "
          "five motor counters at 17 shared checkpoints")

    parser = MessageParser()
    parser.process_identify((mcu.parent / "klipper.dict").read_bytes(), decompress=False)
    tag = lambda name: bytes(parser.messages_by_name[name].msgid_bytes)
    ident = tag("identify")
    events = []
    with observed_peer(mcu, 0, events) as primary, fresh(upstream / "out/klipper.elf") as normal:
        rejected = proc(primary, normal)
    assert rejected.returncode and "NotFixture" in rejected.stderr, rejected.stderr
    assert events and all(typ == "send" and p.startswith(ident) for typ, _, p in events), events
    print("Unmarked peer refused before the marked peer receives configuration")

    motion = (tag("queue_step"), tag("set_next_step_dir"))
    advance = tag("rust_fixture_advance")
    events = []
    with observed_peer(mcu, 0, events) as primary,             observed_peer(mcu, 1, events, lambda p: p.startswith(motion)) as secondary:
        fail = proc(primary, secondary)
    assert fail.returncode and "peer 1 motion: unknown config write outcome" in fail.stderr, fail.stderr
    assert events[-1] == ("lost", 1, b""), events[-8:]
    assert not any(p.startswith(advance) for _, _, p in events), events[-8:]
    assert sum(typ == "send" and board == 1 and p.startswith(motion) for typ, board, p in events) == 1
    print("Lost secondary motion ACK: no retry, later writes, or clock advance on either peer")
