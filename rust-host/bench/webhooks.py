#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare the read-only request/response subset with stock Klippy WebHooks."""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from planner import PIN


def stock(upstream):
    sys.path.insert(0, str(upstream.resolve() / "klippy"))
    from webhooks import GCodeHelper, QueryStatusHelper, WebHooks, WebRequest, WebRequestError

    class Reactor:
        NOW = 0.
        NEVER = 1e30
        def register_timer(self, timer, wake):
            self.timer = timer
            return None

        def unregister_timer(self, timer):
            pass

        def assert_no_pause(self):
            return nullcontext()

        def completion(self):
            complete = SimpleNamespace(complete=lambda result: setattr(complete, "result", result))
            def wait():
                self.timer(7.)
                return complete.result
            complete.wait = wait
            return complete

    local = Reactor()
    handlers = object.__new__(WebHooks)
    handlers._endpoints = {"list_endpoints": handlers._handle_list_endpoints}
    handlers.register_endpoint("info", handlers._handle_info_request)
    gcode_help = {"G90": "Absolute XYZ", "M83": "Relative extrusion"}
    gcode = SimpleNamespace(get_command_help=lambda: gcode_help)
    objects = {
        "toolhead": SimpleNamespace(get_status=lambda _: {"position": [1.,2.,3.,0.], "homed_axes":"xyz"}),
        "gcode_move": SimpleNamespace(get_status=lambda _: {"speed":30., "absolute_coordinates":True}),
    }
    nons = {"webhooks": handlers, "gcode": gcode}
    handlers.printer = SimpleNamespace(lookup_objects=lambda: objects.items(),
        lookup_object=lambda n, fallback=None: nons.get(n, objects.get(n, fallback)),
        get_reactor=lambda: local, get_state_message=lambda: ("Offline fixture","ready"),
        get_start_args=lambda: {"software_version":"fixture-v1"})
    help_ = object.__new__(GCodeHelper)
    help_.gcode = gcode
    handlers.register_endpoint("gcode/help", help_._handle_help)
    QueryStatusHelper(handlers.printer)
    client = type("Client", (), {"set_client_info": lambda *_: None, "is_closed": lambda *_: False, "send": lambda *_: None})()

    def handle(raw):
        try:
            wr = WebRequest(client, raw)
        except (TypeError, ValueError):
            return None
        try:
            handlers.get_callback(wr.get_method())(wr)
        except WebRequestError as exc:
            wr.set_error(exc)
        reply = wr.finish()
        if reply and "result" in reply and wr.get_method() == "info":
            reply["result"] = {k: reply["result"][k] for k in ("state", "state_message", "software_version")}
        return reply
    return handle


def fixture():
    yield b"bad utf8\xff"
    yield b'{"method":false,"id":99}'
    yield b'{"method":"info","params":null,"id":99}'
    yield b'{"method":"gcode/help"}'
    yield b'{"method":"info","id":null}'
    yield b'{"method":"objects/subscribe","id":1,"params":{"objects":{"toolhead":["bad","position"]}}}'
    for i, (method, params) in enumerate([
        ("list_endpoints", {}),
        ("info", {}),
        ("info", {"client_info":{"version":"fixture","type":"simulated"}}),
        ("info", {"client_info":None}),
        ("gcode/help", {}),
        ("gcode/script", {"script":"no execution in fixture"}),
        ("gcode/restart", {}),
        ("objects/list", {}),
        ("objects/query", {}),
        ("objects/query", {"objects":12}),
        ("objects/query", {"objects":{"x":123}}),
        ("objects/query", {"objects":{"x":[1]}}),
        ("objects/query", {"objects":{"toolhead":None,"missing":None}}),
        ("objects/query", {"objects":{"gcode_move":["speed","bad"],"missing":["test"]}}),
        ("objects/query", {"objects":{"gcode_move":None},"response_template":[]}),
        ("objects/query", {"objects":{}, "response_template":{"id":12}}),
    ], start=2):
        yield json.dumps({"id":i, "method":method, "params":params}).encode()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("upstream", type=Path, help=f"checkout at {PIN}")
    p.add_argument("binary", type=Path)
    args = p.parse_args()
    handle = stock(args.upstream)
    frames = list(fixture())
    want = [out for data in frames if (out := handle(data)) is not None]
    wire = subprocess.check_output([args.binary.resolve()], input=b"\x03".join(frames)+b"\x03")
    actual = [json.loads(frame) for frame in wire.split(b"\x03") if frame]
    for i, (a, b) in enumerate(zip(want, actual, strict=True)):
        assert a == b, (i, a, b)
    print(f"{len(frames)} requests, {len(want)} responses matched stock exactly")


if __name__ == "__main__":
    main()
