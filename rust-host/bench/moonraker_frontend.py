#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Exercise the pinned Moonraker HTTP/WebSocket server with a read-only Rust peer.

The Klippy dial is an injected anonymous socket pair. The only HTTP
listener is an ephemeral loopback socket with an ephemeral test key.
No real Klippy socket, device path, or printer command is used.
"""
import argparse
import asyncio
import contextlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import types
import uuid

from moonraker_connection import PIN


async def exercise(root, fixture):
    from moonraker.server import Server
    from moonraker.eventloop import EventLoop
    from tornado.httpclient import AsyncHTTPClient, HTTPClientError, HTTPRequest
    from tornado.websocket import websocket_connect

    state = root/"state"
    marker = root/"dial-marker"
    cfg = root/"moonraker.conf"
    cfg.write_text(f"""
[server]
host: 127.0.0.1
port: 0
klippy_uds_address: {marker}
[machine]
provider: none
validate_service: False
[announcements]
enable_moonlight: False
[file_manager]
enable_object_processing: False
""")
    for f in ("config","logs","misc","comms"):
        (root/f).mkdir()
    (root/"misc/usb.ids").write_text("# Offline fixture\n")
    app_args = {"data_path":str(root),"config_file":str(cfg),"log_file":"",
                "backup_config":None,"startup_warnings":[],"verbose":False,"debug":False,
                "asyncio_debug":False,"is_backup_config":False,"is_python_package":False,
                "is_default_data_path":False,"instance_uuid":uuid.uuid4().hex,
                "unix_socket_path":str(root/"comms/moonraker.sock"),"structured_logging":False,
                "python_version":sys.version,"launch_args":"offline fixture",
                "msgspec_enabled":False,"uvloop_enabled":False}
    app_args["software_version"] = f"moonraker-fixture-{PIN[:8]}"
    server = Server(app_args,types.SimpleNamespace(set_rollover_info=lambda *a:None, set_server=lambda *a:None),EventLoop())
    offered = asyncio.Queue()
    children = []
    # Only the inbound Klippy dial is replaced. The app bootstraps ordinary
    # Moonraker components and listens on an ephemeral loopback port.
    async def dial(self, primary=False):
        if not primary or offered.empty():
            raise FileNotFoundError("fixture peer not offered")
        return await asyncio.open_unix_connection(sock=offered.get_nowait())
    server.klippy_connection.open_klippy_connection=types.MethodType(dial,server.klippy_connection)

    def set_state(kind, x):
        tmp=root/"next"; tmp.write_text(f"{kind} {x}\n"); tmp.replace(state)
    async def launch(kind, x):
        set_state(kind,x)
        left,right=socket.socketpair()
        try:
            child=await asyncio.create_subprocess_exec(fixture,state,stdin=right,
                                                       stdout=asyncio.subprocess.DEVNULL)
        except BaseException:
            left.close()
            raise
        finally:
            right.close()
        children.append(child)
        offered.put_nowait(left)
        marker.touch()
        return child

    client = AsyncHTTPClient()
    ws = None
    try:
        server.load_components()
        await server.server_init()
        sockets = list(server.moonraker_app.http_server._sockets.values())
        assert len(sockets) == 1
        host, port = sockets[0].getsockname()
        assert host == "127.0.0.1" and port > 0
        base=f"http://{host}:{port}"
        empty=await client.fetch(base+"/server/info",raise_error=False,request_timeout=3)
        assert empty.code==401, empty.code
        key=server.lookup_component("authorization").get_api_key()
        assert key
        headers={"X-Api-Key":key}
        async def get(path):
            response=await client.fetch(base+path,headers=headers,request_timeout=3)
            return json.loads(response.body)["result"]
        async def state_is(target):
            for _ in range(80):
                row=await get("/server/info")
                if row["klippy_state"]==target:
                    return row
                await asyncio.sleep(.04)
            raise AssertionError((target,row))

        row=await state_is("disconnected")
        assert not row["klippy_connected"]
        first=await launch("startup",10)
        assert (await state_is("startup"))["klippy_connected"]
        set_state("ready",10)
        await state_is("ready")
        for i in range(30):
            try:
                ans=await get("/printer/objects/query?toolhead=position")
                break
            except HTTPClientError as e:
                if e.code not in (404,503):
                    raise
                await asyncio.sleep(.05)
        else:
            raise AssertionError("stock endpoint was never registered")
        assert ans["status"]=={"toolhead":{"position":[10,2,3,0]}},ans
        assert set((await get("/printer/objects/list"))["objects"]) == {"webhooks", "toolhead"}
        ws=await websocket_connect(HTTPRequest(base.replace("http:","ws:")+"/websocket",headers=headers))
        async def receive(predicate):
            for _ in range(40):
                raw=await asyncio.wait_for(ws.read_message(),4)
                assert raw is not None
                v=json.loads(raw)
                if predicate(v):
                    return v
            raise AssertionError("no matching websocket message")
        async def subscribe(request_id, position, require_clean=False):
            await ws.write_message(json.dumps({"jsonrpc":"2.0","id":request_id,
                 "method":"printer.objects.subscribe","params":{"objects":{"toolhead":["position"]}}}))
            def subscribed(frame):
                if require_clean and frame.get("method") == "notify_status_update":
                    assert "toolhead" not in frame["params"][0], "prior subscription survived reconnect"
                return frame.get("id") == request_id
            frame=await receive(subscribed)
            assert frame["result"]["status"]=={"toolhead":{"position":[position,2,3,0]}},frame
        async def updated(position):
            frame=await receive(lambda v:v.get("method")=="notify_status_update" and
                                v.get("params") and v["params"][0].get("toolhead")==
                                {"position":[position,2,3,0]})
            assert "position" in frame["params"][0]["toolhead"]
        await subscribe(1,10)
        set_state("ready",11)
        await updated(11)
        set_state("shutdown",11)
        await state_is("shutdown")
        await receive(lambda v:v.get("method")=="notify_klippy_shutdown")
        first.terminate()
        await first.wait()
        assert not (await state_is("disconnected"))["klippy_connected"]
        await receive(lambda v:v.get("method")=="notify_klippy_disconnected")
        await launch("startup",20)
        assert (await state_is("startup"))["klippy_connected"]
        set_state("ready",20)
        await state_is("ready")
        await receive(lambda v:v.get("method")=="notify_klippy_ready")
        set_state("ready",21)
        for _ in range(30):
            ans=await get("/printer/objects/query?toolhead=position")
            if ans["status"]["toolhead"]["position"][0]==21:
                break
            await asyncio.sleep(.03)
        else:
            raise AssertionError("reconnected Rust status not updated")
        await subscribe(2,21,require_clean=True)
        set_state("ready",22)
        await updated(22)
        print("Moonraker read-only frontend: auth, HTTP readiness/query, WebSocket changes, "
              "shutdown/disconnect and renewed subscription verified on loopback",flush=True)
    finally:
        if ws is not None:
            ws.close()
        client.close()
        await server._stop_server("terminate")
        while not offered.empty():
            offered.get_nowait().close()
        for child in children:
            if child.returncode is None:
                with contextlib.suppress(ProcessLookupError):
                    child.terminate()
            await asyncio.wait_for(child.wait(),3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("moonraker", type=lambda s: Path(s).resolve())
    parser.add_argument("fixture", type=lambda s: Path(s).resolve())
    args = parser.parse_args()
    pin = subprocess.check_output(["git","-C",args.moonraker,"rev-parse","HEAD"],text=True).strip()
    if pin != PIN:
        parser.error(f"Moonraker checkout must be {PIN}")
    subprocess.run(["git","-C",args.moonraker,"diff","--quiet","HEAD","--"],check=True)
    sys.path.insert(0,str(args.moonraker))
    with tempfile.TemporaryDirectory(prefix="klipper-moonraker-frontend-") as d:
        asyncio.run(asyncio.wait_for(exercise(Path(d),args.fixture),23))


if __name__ == "__main__":
    main()
