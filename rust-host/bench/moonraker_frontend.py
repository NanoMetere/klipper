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


def stock_display_timeline(upstream):
    """Take byte/active fields from stock VirtualSD, then drive stock DisplayStatus."""
    from display_status import stock as display_stock
    from planner import PIN as KLIPPY_PIN
    from virtual_sd import stock as sd_stock

    revision = subprocess.check_output(["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip()
    assert revision == KLIPPY_PIN, revision
    subprocess.run(["git", "-C", upstream, "diff", "--quiet", "HEAD", "--"], check=True)
    sys.path.insert(0, str(upstream / "klippy"))
    from extras.virtual_sdcard import VirtualSD

    body = "; 한글\r\nG90\n".encode()
    def capture(cancel):
        rows = sd_stock(VirtualSD, body, 0, 1, 0, 0, with_status=True, with_cancel=cancel,
                        fixture_path="/virtual/상태.gcode")
        return [json.loads(row[7:]) for row in rows if row.startswith("STATUS ")]
    selected, running, pending, paused, resumed, complete = capture(False)
    *_, canceled, reset = capture(True)
    sd = [selected, running, pending, paused, paused, resumed, complete,
          selected, canceled, reset]
    commands = {1: ["M73 P40", "M117 준비 중"], 5: ['SET_DISPLAY_TEXT MSG=""'], 6: ["M117"]}
    rows = [(8.5 if step == 2 else step * 3., status["is_active"], status["progress"], command)
            for step, status in enumerate(sd) for command in commands.get(step, [None])]
    samples = list(display_stock(upstream, rows))
    end = 0
    pairs = []
    for step, status in enumerate(sd):
        end += len(commands.get(step, [None]))
        pairs.append((status, samples[end - 1]))
    assert len(pairs) == 10 and all(pair[1] is not None for pair in pairs)
    return pairs


async def exercise(root, fixture, with_program=False, delay_ready=False, stock_timeline=None):
    from moonraker.server import Server
    from moonraker.eventloop import EventLoop
    from tornado.httpclient import AsyncHTTPClient, HTTPClientError, HTTPRequest
    from tornado.websocket import websocket_connect

    state = root/"state"
    control = root/"program-control"
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
    if delay_ready:
        # Test-only: make public READY precede completed init deterministically.
        verify = server.klippy_connection._verify_klippy_requirements

        async def delayed_requirements():
            await asyncio.sleep(.85)
            await verify()

        server.klippy_connection._verify_klippy_requirements = delayed_requirements
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
    def set_program(step):
        tmp = root/"next-program"
        tmp.write_text(f"{step}\n")
        tmp.replace(control)

    async def launch(kind, x):
        set_state(kind,x)
        if with_program:
            set_program(0)
        left,right=socket.socketpair()
        try:
            child=await asyncio.create_subprocess_exec(
                fixture,state,*([control] if with_program else []),stdin=right,
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
        # HTTP reports ready before Moonraker finishes its own init hooks.
        # Injecting shutdown during that interval can suppress the WebSocket
        # notification. Exercise shutdown only after stock init completes.
        assert await server.klippy_connection.wait_started(timeout=4)
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
        objects = {"webhooks", "toolhead"}
        if with_program:
            objects.update(("configfile", "fan", "virtual_mixed_job", "virtual_fixture",
                            "virtual_sdcard", "print_stats", "display_status"))
        assert set((await get("/printer/objects/list"))["objects"]) == objects
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
        if with_program:
            # KlipperScreen reads the fan config and subscribes to fan.speed.
            # Test only this read-only protocol contract, not the GTK client.
            fan_query = "/printer/objects/query?configfile=config&fan"
            def check_fan(status, speed):
                assert "fan" in status["configfile"]["config"], status
                assert status["fan"] == {"speed": speed, "rpm": None}, status
            check_fan((await get(fan_query))["status"], 0.0)
            await ws.write_message(json.dumps({
                "jsonrpc":"2.0","id":100,"method":"printer.objects.subscribe",
                "params":{"objects":{"virtual_mixed_job":None,"virtual_fixture":None,
                                     "fan":["speed"], "virtual_sdcard": None,
                                     "print_stats": ["state", "filename"], "display_status": None}},
            }))
            subscribed=await receive(lambda v:v.get("id")==100)
            initial=subscribed["result"]["status"]["virtual_mixed_job"]
            assert initial["mode"]=="simulation" and initial["state"]=="ready"
            assert initial["active_tool"]==2 and initial["peer_proof"] is None
            assert subscribed["result"]["status"]["fan"] == {"speed": 0.0}
            sd_query = "/printer/objects/query?print_stats&virtual_sdcard&display_status"
            sd_path = "/virtual/상태.gcode"
            states = ["standby", "printing", "printing", "paused", "paused", "printing",
                      "complete", "standby", "cancelled", "standby"]
            sizes = [14, 14, 14, 14, 14, 14, 14, 14, 0, 0]
            positions = [0, 0, 0, 10, 10, 10, 14, 0, 0, 0]
            active_steps = {1, 2, 5}
            def check_sd(status, step):
                sd, stats = status["virtual_sdcard"], status["print_stats"]
                size, position = sizes[step], positions[step]
                assert sd == {"file_path": None if step in (6, 8, 9) else sd_path,
                              "file_position": position, "file_size": size,
                              "progress": position / size if size else 0.,
                              "is_active": step in active_steps}, (step, sd)
                assert stats["state"] == states[step] and stats["filename"] == ("" if step == 9 else "상태.gcode"), stats
                expected_progress = .4 if step in (1, 2) else (position / size if size else 0.0)
                expected_message = "준비 중" if 1 <= step <= 4 else ("" if step == 5 else None)
                display = status["display_status"]
                assert display == {"progress": expected_progress, "message": expected_message}, (step, display)
                if stock_timeline is not None:
                    expected_sd, expected_display = stock_timeline[step]
                    assert sd == expected_sd and display == expected_display, (step, status)
            check_sd((await get(sd_query))["status"], 0)
            assert subscribed["result"]["status"]["print_stats"] == {
                "state": "standby", "filename": "상태.gcode"
            }
            await ws.write_message(json.dumps({"jsonrpc":"2.0", "id":103,
                                              "method":"printer.gcode.script",
                                              "params":{"script":"M105"}}))
            denied = await receive(lambda v: v.get("id") == 103)
            assert "error" in denied and "result" not in denied, denied
            check_sd((await get(sd_query))["status"], 0)
            generations = []
            for step in range(1, 10):
                set_program(step)
                pushed=await receive(lambda v: v.get("method")=="notify_status_update" and
                                    v.get("params") and v["params"][0].get("virtual_fixture",{})
                                    .get("fixture_step")==step)
                changed = pushed["params"][0]
                diff = changed.get("virtual_mixed_job",{})
                check_sd((await get(sd_query))["status"], step)
                if step in (2, 4):
                    assert "display_status" not in changed, changed
                    assert "virtual_sdcard" not in changed and "print_stats" not in changed, changed
                elif step in (3, 6):
                    assert changed["virtual_sdcard"]["file_position"] == positions[step], changed
                    assert changed["print_stats"] == {"state": states[step]}, changed
                elif step == 8:
                    assert changed["virtual_sdcard"]["file_size"] == 0, changed
                    assert changed["print_stats"] == {"state": "cancelled"}, changed
                elif step == 9:
                    assert "virtual_sdcard" not in changed, changed
                    assert changed["print_stats"] == {"state": "standby", "filename": ""}, changed
                if step == 3:
                    assert changed["display_status"] == {"progress": 10 / 14}, changed
                elif step == 5:
                    assert changed["display_status"] == {"message": ""}, changed
                elif step == 6:
                    assert changed["display_status"] == {"progress": 1.0, "message": None}, changed
                if step != 5:
                    assert "fan" not in changed, changed
                if step==3:
                    assert diff["state"]=="waiting_for_motion_proof"
                    assert [r["peer"] for r in diff["peer_proof"]["requirements"]]==[0,1]
                elif step in (4, 8):
                    assert not diff, changed
                elif step == 7:
                    assert diff["state"] == "waiting_for_motion_proof"
                    assert [r["peer"] for r in diff["peer_proof"]["requirements"]] == [0, 2]
                elif step == 9:
                    assert diff["state"] == "finished" and diff["peer_proof"] is None
                    assert diff["line"] is None
                elif step==5:
                    assert diff["active_tool"]==5 and diff["print_fan_demand"]==0.4
                    assert changed["fan"] == {"speed": 0.4}, changed
                    assert diff["state"]=="waiting_for_temperature" and diff["peer_proof"] is None
                row=await get("/printer/objects/query?virtual_mixed_job")
                job=row["status"]["virtual_mixed_job"]
                if step in (1,5):
                    assert job["state"]=="waiting_for_temperature"
                elif step in (2, 6):
                    assert job["state"]=="running"
                elif step == 9:
                    assert job["state"] == "finished" and job["peer_proof"] is None
                    assert job["pending_stages"] == 0 and job["line"] is None
                else:
                    assert job["state"]=="waiting_for_motion_proof"
                    requirements=job["peer_proof"]["requirements"]
                    expected = [(0, 10), (1, 3)] if step in (3, 4) else [(0, 10), (2, 3)]
                    assert [(p["peer"],p["accepted_commands"]) for p in requirements] == expected
                    generations.append(job["peer_proof"]["generation"])
                if step<5:
                    assert job["active_tool"]==2 and job["print_fan_demand"] == 0.0
                else:
                    assert job["active_tool"]==5 and job["print_fan_demand"]==0.4
                    assert job["heater_targets_c"]==[210.,220.]
                    if step not in (7, 8):
                        assert job["peer_proof"] is None
            first_gen, same_first, second_gen, same_second = generations
            assert first_gen == same_first and first_gen < second_gen == same_second
            check_fan((await get(fan_query))["status"], 0.4)
            print("Moonraker streamed two virtual phases; partial and stale proofs left status unchanged, "
                  "and the exact second receipt completed the job", flush=True)
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
        assert await server.klippy_connection.wait_started(timeout=4)
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
        if with_program:
            check_fan((await get(fan_query))["status"], 0.0)
            fresh = (await get("/printer/objects/query?virtual_mixed_job"))["status"]["virtual_mixed_job"]
            assert fresh["state"] == "ready" and fresh["peer_proof"] is None and fresh["active_tool"] == 2
            check_sd((await get(sd_query))["status"], 0)
            print("Read-only print_stats/virtual_sdcard: UTF-8 byte checkpoints, pending pause, "
                  "completed/cancelled/reset, denied G-code and fresh reconnect", flush=True)
            await ws.write_message(json.dumps({
                "jsonrpc":"2.0","id":101,"method":"printer.objects.subscribe",
                "params":{"objects":{"fan":["speed"], "toolhead":["position"]}},
            }))
            frame = await receive(lambda v: v.get("id") == 101)
            assert frame["result"]["status"] == {
                "fan": {"speed": 0.0}, "toolhead": {"position": [21, 2, 3, 0]}
            }, frame
            print("Read-only fan contract: HTTP 0%/40%, a projected WebSocket change, "
                  "and a fresh 0% on reconnect; no tachometer or control", flush=True)
        set_state("ready",22)
        await updated(22)
        if stock_timeline is not None:
            print("All ten composed virtual SD/display snapshots matched pinned Klippy handlers", flush=True)
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
    parser.add_argument("--program",action="store_true",help="also stream the routed virtual job state")
    parser.add_argument("--delay-ready", action="store_true", help="exercise a slow stock initialization callback")
    parser.add_argument("--stock-klippy", type=lambda s: Path(s).resolve(),
                        help="compare the --program display/SD timeline to these pinned stock handlers")
    args = parser.parse_args()
    if args.stock_klippy and not args.program:
        parser.error("--stock-klippy requires --program")
    timeline = stock_display_timeline(args.stock_klippy) if args.stock_klippy else None
    pin = subprocess.check_output(["git","-C",args.moonraker,"rev-parse","HEAD"],text=True).strip()
    if pin != PIN:
        parser.error(f"Moonraker checkout must be {PIN}")
    subprocess.run(["git","-C",args.moonraker,"diff","--quiet","HEAD","--"],check=True)
    sys.path.insert(0,str(args.moonraker))
    with tempfile.TemporaryDirectory(prefix="klipper-moonraker-frontend-") as d:
        asyncio.run(asyncio.wait_for(exercise(Path(d),args.fixture,args.program,args.delay_ready,timeline),23))


if __name__ == "__main__":
    main()
