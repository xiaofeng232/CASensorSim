#!/usr/bin/env python3
"""
Host-side verification for the cas-sim app (FK407M2-ZGT6).

Connects to the device over its native USB vendor bulk interface (the
Type-C connector, via libusb; the board has no USB-UART bridge, so no
adapter wiring is needed) and exercises the CasIntf Fibre interface end to
end:

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
checkout (modules/lib/fibre/python/fibre/protocol.py); only the libusb
stream adapter lives here.

The device exposes one vendor-specific interface with the ODrive
native-interface class codes (0xFF/0x01/0x01) and one bulk IN / bulk OUT
endpoint pair; the Fibre protocol runs as a raw byte stream over them.

usage: cas_fibre_test.py [vid:pid]
       cas_fibre_test.py                # first matching device
       cas_fibre_test.py 2fe3:0004      # filter by VID:PID
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
    import usb.core
    import usb.util
except ImportError:
    sys.exit("pyusb is required: pip install pyusb (libusb-1.0 must be "
             "installed; on Linux add a udev rule or run as root for raw "
             "USB access)")

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


# Vendor-interface class codes (ODrive native interface), matched by the
# device discovery below.
INTF_CLASS = 0xFF
INTF_SUBCLASS = 0x01
INTF_PROTOCOL = 0x01


def find_fibre_device(vid_pid=None):
    """Returns the first USB device exposing a vendor interface with the
    Fibre class codes, optionally filtered by (vid, pid)."""
    def match(dev):
        if vid_pid and (dev.idVendor, dev.idProduct) != vid_pid:
            return False
        for cfg in dev:
            for intf in cfg:
                if (intf.bInterfaceClass == INTF_CLASS and
                        intf.bInterfaceSubClass == INTF_SUBCLASS and
                        intf.bInterfaceProtocol == INTF_PROTOCOL):
                    return True
        return False

    dev = usb.core.find(custom_match=match)
    if dev is None:
        raise RuntimeError(
            "no USB device with a Fibre vendor interface "
            f"(class {INTF_CLASS:#04x}/{INTF_SUBCLASS:#04x}/"
            f"{INTF_PROTOCOL:#04x}) found"
            + (f" for {vid_pid[0]:04x}:{vid_pid[1]:04x}" if vid_pid else ""))
    return dev


class UsbBulkStreamSource(StreamSource):
    """fibre.protocol StreamSource backed by a libusb bulk IN endpoint."""

    def __init__(self, dev, ep_addr):
        self._dev = dev
        self._ep = ep_addr

    def get_bytes(self, n_bytes, deadline):
        """Returns up to n_bytes, waiting at most until the deadline."""
        data = bytearray()
        while len(data) < n_bytes:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            chunk = self._dev.read(self._ep, n_bytes - len(data),
                                   max(1, int(remaining * 1000)))
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)

    def get_bytes_or_fail(self, n_bytes, deadline):
        """Blocks until exactly n_bytes were received or the deadline is
        reached (raises fibre.utils.TimeoutError)."""
        data = bytearray()
        while len(data) < n_bytes:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise FibreTimeout()
            chunk = self._dev.read(self._ep, n_bytes - len(data),
                                   max(1, int(remaining * 1000)))
            if chunk:
                data.extend(chunk)
        return bytes(data)


class UsbBulkStreamSink(StreamSink):
    """fibre.protocol StreamSink backed by a libusb bulk OUT endpoint."""

    def __init__(self, dev, ep_addr, timeout_ms=1000):
        self._dev = dev
        self._ep = ep_addr
        self._timeout_ms = timeout_ms

    def process_bytes(self, data):
        # libusb segments writes larger than the endpoint MPS automatically;
        # the device also NAKs OUT packets while its Fibre stack is busy, so
        # retry until the whole buffer was accepted.
        data = bytes(data)
        sent = 0
        while sent < len(data):
            sent += self._dev.write(self._ep, data[sent:], self._timeout_ms)


def open_fibre_device(vid_pid=None):
    """Finds the device, claims the Fibre vendor interface and returns
    (dev, bulk_in_addr, bulk_out_addr)."""
    dev = find_fibre_device(vid_pid)
    dev.set_configuration()

    cfg = dev.get_active_configuration()
    intf = usb.util.find_descriptor(
        cfg, custom_match=lambda i: (
            i.bInterfaceClass == INTF_CLASS and
            i.bInterfaceSubClass == INTF_SUBCLASS and
            i.bInterfaceProtocol == INTF_PROTOCOL))
    if intf is None:
        raise RuntimeError("Fibre vendor interface not found in the active "
                           "configuration")

    ep_in = usb.util.find_descriptor(
        intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) ==
            usb.util.ENDPOINT_IN)
    ep_out = usb.util.find_descriptor(
        intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) ==
            usb.util.ENDPOINT_OUT)
    if ep_in is None or ep_out is None:
        raise RuntimeError("bulk endpoints not found on the Fibre interface")

    # pyusb claims the interface on first use; detach kernel drivers if one
    # grabbed it (e.g. on setups where a generic driver binds the interface).
    if dev.is_kernel_driver_active(intf.bInterfaceNumber):
        dev.detach_kernel_driver(intf.bInterfaceNumber)

    product = ""
    try:
        product = usb.util.get_string(dev, dev.iProduct)
    except (usb.core.USBError, ValueError):
        pass
    print(f"Opened USB device {dev.idVendor:04x}:{dev.idProduct:04x} "
          f"bus {dev.bus} address {dev.address}"
          + (f" ({product})" if product else ""))
    return dev, ep_in.bEndpointAddress, ep_out.bEndpointAddress


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
    vid_pid = None
    if len(sys.argv) > 1:
        try:
            vid_s, pid_s = sys.argv[1].split(':')
            vid_pid = (int(vid_s, 16), int(pid_s, 16))
        except ValueError:
            sys.exit(f"usage: {sys.argv[0]} [vid:pid]")

    dev, ep_in, ep_out = open_fibre_device(vid_pid)

    channel = Channel(
        "cas-sim-usb",
        PacketFromStreamConverter(UsbBulkStreamSource(dev, ep_in)),
        StreamBasedPacketSink(UsbBulkStreamSink(dev, ep_out)),
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

    usb.util.dispose_resources(dev)

    print()
    if ok:
        print("cas-sim: ALL CHECKS PASSED")
        return 0
    print("cas-sim: SOME CHECKS FAILED")
    return 1


if __name__ == '__main__':
    sys.exit(main())
