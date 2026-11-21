# SPDX-License-Identifier: GPL-3.0-only
"""Generate hex request/response fixtures using a pinned upstream checkout."""
import importlib.util
import json
import pathlib
import sys
import zlib

root = pathlib.Path(sys.argv[1])
spec = importlib.util.spec_from_file_location("msgproto", root / "klippy/msgproto.py")
msgproto = importlib.util.module_from_spec(spec)
spec.loader.exec_module(msgproto)
parser = msgproto.MessageParser()
response = parser.messages_by_name["identify_response"]
dictionary = json.loads(pathlib.Path(sys.argv[2]).read_bytes())
# Omit builder-specific time/hostname and compiler version from the public fixture.
dictionary["version"] = "native-linux-identify-fixture"
dictionary["build_versions"] = ""
dictionary = json.dumps(dictionary, sort_keys=True, separators=(",", ":")).encode()

def wire(seq, payload):
    parts = parser.encode_msgblock(seq, payload)
    result = bytes(parts[:-2] + parts[-2] + [parts[-1]])
    assert parser.check_packet(result) == len(result)
    return result.hex()

zip_data = zlib.compress(dictionary)
for index, offset in enumerate(range(0, len(zip_data) + 40, 40)):
    offset = min(offset, len(zip_data))
    seq = index & 15
    command = parser.create_command(f"identify offset={offset} count=40")
    reply = response.encode((offset, zip_data[offset:offset + 40]))
    print(wire(seq, command), wire(seq, reply))
    if offset == len(zip_data):
        break

if len(sys.argv) > 3:
    parser.process_identify(dictionary, decompress=False)
    with pathlib.Path(sys.argv[3]).open("w") as output:
        for index, (kind, high, low) in enumerate((
            ("clock", 0, 0x5f), ("clock", 0, 0x60),
            ("clock", 0, 0x12345678), ("clock", 0, 0xffffffff),
            ("uptime", 0, 0), ("uptime", 1, 0xffffffff),
            ("uptime", 0xffffffff, 0x80000000),
        )):
            request = parser.create_command(f"get_{kind}")
            params = (low,) if kind == "clock" else (high, low)
            for wire_params in (params, tuple(x if x < 0x80000000 else x - 0x100000000 for x in params)):
                response = parser.messages_by_name[kind].encode(wire_params)
                print(kind, high, low, wire(index, request), wire(index, response), file=output)
