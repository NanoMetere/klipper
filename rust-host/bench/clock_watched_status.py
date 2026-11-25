#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare offline watch-status subscription frames to stock WebHooks."""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from clock_liveness import case
from planner import PIN


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("upstream", "dictionary", "replay"):
        p.add_argument(name, type=lambda path: Path(path).resolve())
    a = p.parse_args()
    assert subprocess.check_output(["git", "-C", a.upstream, "rev-parse", "HEAD"],
                                   text=True).strip() == PIN
    subprocess.run(["git", "-C", a.upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(a.upstream / "klippy"))
    import clocksync
    import mcu
    from webhooks import QueryStatusHelper, WebHooks, WebRequest, WebRequestError

    class Reactor:
        NOW = 0.
        NEVER = 1e30

        def register_timer(self, timer, wake):
            self.timer = timer
            return timer

        def unregister_timer(self, timer):
            pass

        def assert_no_pause(self):
            return nullcontext()

        def completion(self):
            complete = SimpleNamespace(complete=lambda result: setattr(complete, "result", result))

            def wait():
                self.timer(0.)
                return complete.result

            complete.wait = wait
            return complete

    ref = Reactor()
    outbound = []
    handler = object.__new__(WebHooks)
    handler._endpoints = {"list_endpoints": handler._handle_list_endpoints}
    data = {
        "print_stats": {"state": "printing"},
        "virtual_job": {"issued": 0, "state": "running"},
        "virtual_clock_liveness": {"mode": "simulation", "active": True,
                                   "timeout_reported": False, "file_output": False},
    }
    objects = {name: SimpleNamespace(get_status=lambda _t, key=name: dict(data[key]))
               for name in data}
    handler.printer = SimpleNamespace(
        get_reactor=lambda: ref,
        lookup_object=lambda name, fallback=None: handler if name == "webhooks"
        else objects.get(name, fallback),
    )
    QueryStatusHelper(handler.printer)
    client = SimpleNamespace(send=outbound.append, is_closed=lambda: False)
    class Connection:
        def send(self, msg):
            client.send(msg)

        def is_closed(self):
            return client.is_closed()

    # The stock subscription identifies clients by a hashable connection.
    connection = Connection()

    def send(request):
        wr = WebRequest(connection, json.dumps(request).encode())
        try:
            handler.get_callback(wr.get_method())(wr)
        except WebRequestError as exc:
            wr.set_error(exc)
        response = wr.finish()
        if response is not None:
            outbound.append(response)

    send({"id": 1, "method": "objects/subscribe", "params": {
        "objects": {name: list(value) for name, value in data.items()},
        "response_template": {"method": "clock_status_update"}}})
    data["virtual_job"]["issued"] = 1
    ref.timer(1.)
    policy = [tuple(map(int, row.split()[:2]))
              for row in case(False, "qqqqcqcr", clocksync, mcu)]
    data["virtual_clock_liveness"]["active"] = bool(policy[5][0])
    ref.timer(2.)
    data["virtual_clock_liveness"]["timeout_reported"] = bool(policy[6][1])
    assert data["virtual_clock_liveness"]["timeout_reported"] is True
    data["print_stats"]["state"] = "error"
    data["virtual_job"]["state"] = "faulted"
    ref.timer(3.)
    data["virtual_clock_liveness"]["active"] = bool(policy[7][0])
    ref.timer(4.)
    ref.timer(5.)  # unchanged state produces no notification
    send({"id": 2, "method": "gcode/script", "params": {"script": "M112"}})
    stock = {"messages": outbound, "late_output": 0}
    rust = json.loads(subprocess.check_output([a.replay, a.dictionary], timeout=10))
    assert len(outbound) == 6
    assert json.dumps(rust, sort_keys=True) == json.dumps(stock, sort_keys=True), (rust, stock)
    print("6 exact read-only stock/Rust frames; sticky fault after late reply; no device I/O")


if __name__ == "__main__":
    main()
