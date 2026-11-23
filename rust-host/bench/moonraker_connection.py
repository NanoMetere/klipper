#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Exercise pinned Moonraker client components against a software Rust peer.

Only the dial uses a socket pair. Endpoint and event capture stay in memory;
no Moonraker daemon, HTTP listener, printer path, or hardware is started.
"""
import argparse
import asyncio
import contextlib
import logging
from pathlib import Path
import subprocess
import sys
import tempfile
import socket
from types import SimpleNamespace

PIN = "b559b9a81161f958a43c2f527b6ad96f73310d5d"


async def exercise(binary, root):
    from moonraker.common import KlippyState
    from moonraker.components.klippy_apis import KlippyAPI
    from moonraker.components.klippy_connection import KlippyConnection
    from moonraker.eventloop import EventLoop
    from moonraker.utils import ServerError

    missing_output = []

    class ExpectedFixtureWarning(logging.Filter):
        def filter(self, record):
            if record.getMessage() == "Unable to register gcode output subscription":
                error = record.exc_info[1] if record.exc_info else None
                if isinstance(error, ServerError) and "gcode/subscribe_output" in str(error):
                    missing_output.append(str(error))
                    return False
            return True

    class FixtureServer:
        error = ServerError

        def __init__(self):
            self.loop = EventLoop()
            self.events = asyncio.Queue()
            self.components = {}
            self.endpoints = set()
            self.callbacks = {}
            self.running = False
            self.values = {}

        def get_event_loop(self):
            return self.loop

        def get_app_args(self):
            return {"software_version": "test-only"}

        def register_endpoint(self, name, *args, **kwargs):
            self.endpoints.add(name)

        def register_event_handler(self, event, callback):
            self.callbacks.setdefault(event, []).append(callback)

        def add_log_rollover_item(self, *_):
            pass

        def lookup_component(self, name):
            return self.components[name]

        def is_running(self):
            return self.running

        def send_event(self, event, *args):
            async def deliver():
                for callback in self.callbacks.get(event, ()):
                    result = callback(*args)
                    if result is not None:
                        await result
                self.events.put_nowait((event, args))

            return self.loop.create_task(deliver())

        async def until(self, wanted):
            while True:
                event, args = await asyncio.wait_for(self.events.get(), 4)
                if event == wanted:
                    return args

    class FixtureConnection(KlippyConnection):
        # The real dial needs a named socket. Supply an already-connected
        # pair for this test; retain the stock lifecycle, RPC, and stream code.
        async def open_klippy_connection(self, primary=False):
            if not primary or offered.empty():
                raise FileNotFoundError("no disposable peer available")
            return await asyncio.open_unix_connection(sock=offered.get_nowait())

    async def unit_info(_name, _pid):
        return {}

    def put_state(label, x):
        alternate = root / "next-state"
        alternate.write_text(f"{label} {x}\n")
        alternate.replace(root / "state")

    async def offer_peer():
        left, right = socket.socketpair()
        try:
            child = await asyncio.create_subprocess_exec(
                binary, root / "state", stdin=right, stdout=subprocess.DEVNULL
            )
        except BaseException:
            left.close()
            raise
        finally:
            right.close()
        children.append(child)
        offered.put_nowait(left)
        return child

    async def missing_connection(api):
        try:
            await api.query_objects({"toolhead": None})
        except ServerError as error:
            assert error.status_code == 503, error
        else:
            raise AssertionError("disconnected query did not fail")

    offered = asyncio.Queue()
    children = []
    server = FixtureServer()
    marker = root / "availability"
    marker.touch()  # no named socket; used only by the stock existence guard
    config = SimpleNamespace(get_server=lambda: server, getpath=lambda *_: marker)
    server.components["machine"] = SimpleNamespace(
        get_system_provider=lambda: SimpleNamespace(extract_service_info=unit_info)
    )
    server.components["database"] = SimpleNamespace(
        insert_item=lambda namespace, key, value: server.values.__setitem__((namespace, key), value)
    )
    connection = server.components["klippy_connection"] = FixtureConnection(config)
    api = server.components["klippy_apis"] = KlippyAPI(config)
    updates = asyncio.Queue()

    async def ready(x):
        # The real client must remain in STARTUP during the first reply.
        await server.until("server:klippy_identified")
        assert connection.state == KlippyState.STARTUP
        put_state("ready", x)
        await server.until("server:klippy_ready")
        assert await connection.wait_started(timeout=3)
        assert connection.state == KlippyState.READY
        assert {"info", "objects/list", "objects/query", "objects/subscribe"} <= server.endpoints
        assert set(await api.get_object_list()) == {"webhooks", "toolhead"}
        assert await api.query_objects({"toolhead": ["position"]}) == {
            "toolhead": {"position": [x, 2, 3, 0]}
        }

    def status_update(values, _eventtime):
        if values:
            updates.put_nowait(values)

    async def subscribe(x):
        initial = await api.subscribe_objects({"toolhead": ["position"]}, callback=status_update)
        assert initial == {
            "webhooks": {"state": "ready", "state_message": "Offline Moonraker fixture"},
            "toolhead": {"position": [x, 2, 3, 0]},
        }

    logger = logging.getLogger()
    known_warning = ExpectedFixtureWarning()
    logger.addFilter(known_warning)
    try:
        await missing_connection(api)
        put_state("startup", 10)
        child = await offer_peer()
        server.running = True
        connection.connect()
        await ready(10)
        await subscribe(10)
        put_state("ready", 11)
        assert await asyncio.wait_for(updates.get(), 4) == {
            "toolhead": {"position": [11, 2, 3, 0]}
        }
        put_state("shutdown", 11)
        await server.until("server:klippy_shutdown")
        assert connection.state == KlippyState.SHUTDOWN
        child.terminate()
        await asyncio.wait_for(child.wait(), 4)
        await server.until("server:klippy_disconnect")
        assert not connection.get_subscription_cache()
        assert not api.subscription_callbacks
        await missing_connection(api)
        while not updates.empty():
            updates.get_nowait()

        put_state("startup", 20)
        child = await offer_peer()
        await ready(20)
        put_state("ready", 21)
        # Query proves the peer has observed the change independently.
        for _ in range(20):
            if (await api.query_objects({"toolhead": ["position"]}))["toolhead"]["position"][0] == 21:
                break
            await asyncio.sleep(0.02)
        else:
            raise AssertionError("reconnected peer did not expose the new position")
        assert updates.empty()
        await subscribe(21)
        put_state("ready", 22)
        assert await asyncio.wait_for(updates.get(), 4) == {
            "toolhead": {"position": [22, 2, 3, 0]}
        }
        assert len(missing_output) == 2, missing_output
        assert not {"gcode/script", "emergency_stop"} & server.endpoints
        print("Pinned Moonraker client: disconnected → startup → ready → shutdown "
              "→ disconnected → startup → ready; query, diff, and new subscription verified")
    finally:
        logger.removeFilter(known_warning)
        server.running = False
        await connection.close(wait_closed=True)
        while not offered.empty():
            offered.get_nowait().close()
        for child in children:
            with contextlib.suppress(ProcessLookupError):
                if child.returncode is None:
                    child.terminate()
            await asyncio.wait_for(child.wait(), 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("moonraker", type=lambda s: Path(s).resolve())
    parser.add_argument("fixture", type=lambda s: Path(s).resolve())
    args = parser.parse_args()
    pin = subprocess.check_output(["git", "-C", args.moonraker, "rev-parse", "HEAD"],
                                  text=True).strip()
    if pin != PIN:
        parser.error(f"Moonraker checkout must be {PIN}")
    subprocess.run(["git", "-C", args.moonraker, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(args.moonraker))
    with tempfile.TemporaryDirectory(prefix="moonraker-status-fixture-") as tmp:
        asyncio.run(asyncio.wait_for(exercise(args.fixture, Path(tmp)), 18))


if __name__ == "__main__":
    main()
