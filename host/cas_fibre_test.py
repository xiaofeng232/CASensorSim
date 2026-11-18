#!/usr/bin/env python3
"""
Host-side verification for the cas-sim app (FK407M2-ZGT6).

Connects to the device over its USB CDC-ACM virtual serial port (the Type-C
connector; the board has no USB-UART bridge, so no adapter wiring is needed)
and exercises the CasIntf Fibre interface end to end:

  1. fetch the interface JSON from endpoint 0 (offset based reads)
  2. read `firmware_version` (readonly, expect 0x00010000) and `tick_hz`
     (expect 100000)
  3. read `rpm` (0.0 at power-up) and `sample_count` (must advance)
  4. call `set_rpm(60)` and read back rpm ~= 60 (profile switched to CONST)
  5. call `set_dig_fault(0, immediate)` -> oe_mask bit0 and dig_mask bit0
     must clear (FR-30 high-Z), then `clear_faults()` restores bit0
  6. call `set_ssi_fault(0, immediate, pfb=True)` -> SSI1 bit12 set, then
     `clear_faults()` clears it again
  7. reject invalid arguments (channel out of range, unknown trigger)

The protocol layer comes from the Fibre python package in the west module
checkout (modules/lib/fibre/python/fibre/protocol.py); only the serial
stream adapter lives here.

usage: cas_fibre_test.py <serial-port> [baudrate]
       cas_fibre_test.py /dev/ttyACM0
"""

import json
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.realpath(__file__))
FIBRE_PYTHON = os.path.normpath(os.path.join(
    HERE, '..', '..', '..', 'modules', 'lib', 'fibre', 'python'))
sys.path.insert(0, FIBRE_PYTHON)

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

# The fibre package's __init__.py loads a prebuilt libfibre shared object at
# import time (libfibre.py), which is neither present nor needed here: the
# legacy protocol layer only depends on fibre.utils. Register the package
# without executing its __init__ so the submodules import standalone.
import importlib
import types

_fibre_pkg = types.ModuleType('fibre')
_fibre_pkg.__path__ = [os.path.join(FIBRE_PYTHON, 'fibre')]
sys.modules['fibre'] = _fibre_pkg
importlib.import_module('fibre.utils')

from fibre.utils import Event, Logger, TimeoutError as FibreTimeout
from fibre.protocol import (
    Channel,
    PacketFromStreamConverter,
    StreamBasedPacketSink,
    StreamSource,
    StreamSink,
    calc_crc16,
    PROTOCOL_VERSION,
)

EXPECTED_FIRMWARE_VERSION = 0x00010000
EXPECTED_TICK_HZ = 100000
JSON_CHUNK_SIZE = 96  # legacy protocol packets carry at most 127 bytes

TRIGGER_IMMEDIATE = 0

# pack formats per interface type (legacy endpoint payload = raw C value)
PACK = {
    'uint8': '<B', 'int8': '<b',
    'uint16': '<H', 'int16': '<h',
    'uint32': '<I', 'int32': '<i',
    'uint64': '<Q', 'int64': '<q',
    'float32': '<f',
}


class SerialStreamSource(StreamSource):
    """fibre.protocol StreamSource backed by a pyserial port."""

    def __init__(self, ser):
        self._ser = ser

    def get_bytes(self, n_bytes, deadline):
        """Returns up to n_bytes, waiting at most until the deadline."""
        self._ser.timeout = max(0.0, deadline - time.monotonic())
        return self._ser.read(n_bytes)

    def get_bytes_or_fail(self, n_bytes, deadline):
        """Blocks until exactly n_bytes were received or the deadline is
        reached (raises fibre.utils.TimeoutError)."""
        data = bytearray()
        while len(data) < n_bytes:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise FibreTimeout()
            self._ser.timeout = remaining
            chunk = self._ser.read(n_bytes - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)


class SerialStreamSink(StreamSink):
    """fibre.protocol StreamSink backed by a pyserial port."""

    def __init__(self, ser):
        self._ser = ser

    def process_bytes(self, data):
        self._ser.write(data)
        self._ser.flush()


def read_prop(channel, endpoint, size):
    data = channel.remote_endpoint_operation(endpoint['id'], b'', True, size)
    return data


def fetch_interface_json(channel):
    """Reads the interface definition from endpoint 0 (offset based) and the
    JSON version ID (offset 0xFFFFFFFF). Returns (json_dict, json_crc)."""
    json_bytes = bytearray()
    while True:
        chunk = channel.remote_endpoint_operation(
            0, struct.pack('<I', len(json_bytes)), True, JSON_CHUNK_SIZE)
        if not chunk:
            break
        json_bytes.extend(chunk)

    version_id = struct.unpack('<I', channel.remote_endpoint_operation(
        0, struct.pack('<I', 0xFFFFFFFF), True, 4))[0]
    json_crc = version_id >> 16

    computed_crc = calc_crc16(PROTOCOL_VERSION, json_bytes)
    if json_crc != computed_crc:
        raise RuntimeError(
            f"JSON CRC mismatch: device reports 0x{json_crc:04x}, "
            f"computed 0x{computed_crc:04x}")

    return json.loads(json_bytes.decode('utf-8')), json_crc


def find_endpoint(json_def, name):
    for ep in json_def:
        if ep.get('name') == name:
            return ep
    raise RuntimeError(f"endpoint '{name}' not found in interface JSON")


def call_function(channel, endpoint, *args):
    """Legacy protocol function call: write the input endpoints (each
    directly after the trigger endpoint in the table), then access the
    trigger endpoint to execute. The single `out` value is returned."""
    for ep_in, value in zip(endpoint['inputs'], args):
        fmt = PACK[ep_in['type']]
        channel.remote_endpoint_operation(
            ep_in['id'], struct.pack(fmt, value), True, 0)
    return channel.remote_endpoint_operation(endpoint['id'], b'', True, 0)


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"  [{status}] {name}" + (f" ({detail})" if detail else ""))
    return condition


def main():
    if len(sys.argv) < 2:
        sys.exit(f"usage: {sys.argv[0]} <serial-port> [baudrate]")
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    ser = serial.Serial(port, baud, timeout=1.0)
    print(f"Opened {port} at {baud} baud")

    channel = Channel(
        "cas-sim-uart",
        PacketFromStreamConverter(SerialStreamSource(ser)),
        StreamBasedPacketSink(SerialStreamSink(ser)),
        Event(),  # cancellation token
        Logger(),
    )

    print("Fetching interface definition (endpoint 0)...")
    json_def, json_crc = fetch_interface_json(channel)
    # All non-ep0 operations must carry the interface CRC as packet trailer.
    channel._interface_definition_crc = json_crc
    print(f"Interface JSON: {len(json_def)} endpoints, CRC 0x{json_crc:04x}")

    ok = True

    # -- readonly identity / telemetry -------------------------------------
    fw = struct.unpack('<I', read_prop(
        channel, find_endpoint(json_def, 'firmware_version'), 4))[0]
    ok &= check("firmware_version", fw == EXPECTED_FIRMWARE_VERSION,
                f"0x{fw:08x}, expected 0x{EXPECTED_FIRMWARE_VERSION:08x}")

    tick_hz = struct.unpack('<I', read_prop(
        channel, find_endpoint(json_def, 'tick_hz'), 4))[0]
    ok &= check("tick_hz", tick_hz == EXPECTED_TICK_HZ,
                f"{tick_hz}, expected {EXPECTED_TICK_HZ}")

    rpm0 = struct.unpack('<f', read_prop(
        channel, find_endpoint(json_def, 'rpm'), 4))[0]
    ok &= check("rpm starts at 0 (engine stopped)", rpm0 == 0.0, f"{rpm0}")

    cnt0 = struct.unpack('<Q', read_prop(
        channel, find_endpoint(json_def, 'sample_count'), 8))[0]
    time.sleep(0.5)
    cnt1 = struct.unpack('<Q', read_prop(
        channel, find_endpoint(json_def, 'sample_count'), 8))[0]
    ok &= check("sample_count advances (~50k in 0.5 s at 100 kHz)",
                cnt1 > cnt0, f"{cnt0} -> {cnt1}")

    # -- set_rpm ------------------------------------------------------------
    ok_ret = call_function(channel, find_endpoint(json_def, 'set_rpm'), 60000)
    ok &= check("set_rpm(60.000 rpm) accepted", len(ok_ret) == 1 and ok_ret[0],
                f"out={ok_ret.hex() if ok_ret else b''}")
    time.sleep(0.2)
    rpm1 = struct.unpack('<f', read_prop(
        channel, find_endpoint(json_def, 'rpm'), 4))[0]
    ok &= check("rpm reads back ~60", abs(rpm1 - 60.0) < 0.5, f"{rpm1}")

    # -- FR-30 digital channel break (IMMEDIATE) ----------------------------
    ok_ret = call_function(channel, find_endpoint(json_def, 'set_dig_fault'),
                           0, 1, TRIGGER_IMMEDIATE, 0, 1)
    ok &= check("set_dig_fault(ch0, immediate) accepted",
                len(ok_ret) == 1 and ok_ret[0])
    time.sleep(0.1)
    oe = struct.unpack('<B', read_prop(
        channel, find_endpoint(json_def, 'oe_mask'), 1))[0]
    dig = struct.unpack('<B', read_prop(
        channel, find_endpoint(json_def, 'dig_mask'), 1))[0]
    ok &= check("oe_mask bit0 cleared (high-Z)", (oe & 0x01) == 0, f"0x{oe:02x}")
    ok &= check("dig_mask bit0 cleared (line reads 0)", (dig & 0x01) == 0,
                f"0x{dig:02x}")

    # -- FR-31 SSI PFB bit (IMMEDIATE) ---------------------------------------
    ok_ret = call_function(channel, find_endpoint(json_def, 'set_ssi_fault'),
                           0, 1, TRIGGER_IMMEDIATE, 0, 1,
                           1, 0, 0, 0, 0)
    ok &= check("set_ssi_fault(ch0, pfb) accepted",
                len(ok_ret) == 1 and ok_ret[0])
    time.sleep(0.1)
    ssi1 = struct.unpack('<i', read_prop(
        channel, find_endpoint(json_def, 'ssi1'), 4))[0]
    ok &= check("ssi1 bit12 (PFB) set", (ssi1 & 0x1000) != 0, f"{ssi1:#06x}")

    # -- clear_faults restores the FR-34 all-off state ------------------------
    call_function(channel, find_endpoint(json_def, 'clear_faults'))
    time.sleep(0.1)
    oe = struct.unpack('<B', read_prop(
        channel, find_endpoint(json_def, 'oe_mask'), 1))[0]
    ssi1 = struct.unpack('<i', read_prop(
        channel, find_endpoint(json_def, 'ssi1'), 4))[0]
    ok &= check("oe_mask bit0 restored", (oe & 0x01) == 1, f"0x{oe:02x}")
    ok &= check("ssi1 PFB cleared", (ssi1 & 0x1000) == 0, f"{ssi1:#06x}")

    # -- invalid arguments rejected ------------------------------------------
    ok_ret = call_function(channel, find_endpoint(json_def, 'set_dig_fault'),
                           8, 1, TRIGGER_IMMEDIATE, 0, 1)
    ok &= check("set_dig_fault(ch8) rejected", len(ok_ret) == 1 and not ok_ret[0])
    ok_ret = call_function(channel, find_endpoint(json_def, 'set_ssi_fault'),
                           2, 1, TRIGGER_IMMEDIATE, 0, 1, 0, 0, 0, 0, 0)
    ok &= check("set_ssi_fault(ch2) rejected", len(ok_ret) == 1 and not ok_ret[0])
    ok_ret = call_function(channel, find_endpoint(json_def, 'set_dig_fault'),
                           0, 1, 4, 0, 1)
    ok &= check("set_dig_fault(trigger=4) rejected",
                len(ok_ret) == 1 and not ok_ret[0])

    # -- stop the engine again ------------------------------------------------
    call_function(channel, find_endpoint(json_def, 'set_rpm'), 0)

    ser.close()

    print()
    if ok:
        print("cas-sim: ALL CHECKS PASSED")
        return 0
    print("cas-sim: SOME CHECKS FAILED")
    return 1


if __name__ == '__main__':
    sys.exit(main())
