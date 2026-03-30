#!/usr/bin/env python3
"""Nintendo Switch Pro Controller USB protocol tester.

Interactive tool for sending raw HID commands to a real Pro Controller
over USB-C, recording responses, and mapping protocol behavior.

Usage:
    uv run python switch_probe.py          # Interactive REPL
    uv run python switch_probe.py --scan   # List HID devices
    uv run python switch_probe.py --auto   # Auto handshake + dump info
"""

from __future__ import annotations

import argparse
import json
import readline  # noqa: F401 — enables arrow-key history in input()
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TextIO

try:
    import hid
except ImportError:
    print("Error: hidapi not installed. Run: uv sync")
    sys.exit(1)

# Nintendo Switch Pro Controller USB IDs
SWITCH_PRO_VID = 0x057E
SWITCH_PRO_PID = 0x2009

# Report IDs
REPORT_USB_CMD = 0x80
REPORT_USB_REPLY = 0x81
REPORT_SUBCMD = 0x01
REPORT_SUBCMD_REPLY = 0x21
REPORT_RUMBLE = 0x10
REPORT_INPUT = 0x30
REPORT_SIMPLE_INPUT = 0x3F

# USB handshake sub-commands
USB_CMD_STATUS = 0x01
USB_CMD_HANDSHAKE = 0x02
USB_CMD_HIGH_SPEED = 0x03
USB_CMD_FORCE_USB = 0x04
USB_CMD_DISABLE_TIMEOUT = 0x05

# Button bitmasks
BUTTONS_RIGHT = {
    0x01: "Y",
    0x02: "X",
    0x04: "B",
    0x08: "A",
    0x40: "R",
    0x80: "ZR",
}
BUTTONS_SHARED = {
    0x01: "Minus",
    0x02: "Plus",
    0x04: "RStick",
    0x08: "LStick",
    0x10: "Home",
    0x20: "Capture",
}
BUTTONS_LEFT = {
    0x01: "Down",
    0x02: "Up",
    0x04: "Right",
    0x08: "Left",
    0x40: "L",
    0x80: "ZL",
}

# Known SPI flash addresses
SPI_ADDRESSES = {
    0x6000: ("Serial number", 16),
    0x6012: ("Device type", 1),
    0x6020: ("Factory IMU calibration", 24),
    0x603D: ("Factory left stick cal", 9),
    0x6046: ("Factory right stick cal", 9),
    0x6050: ("Body color", 3),
    0x6053: ("Button color", 3),
    0x6056: ("Left grip color", 3),
    0x6059: ("Right grip color", 3),
    0x8010: ("User left stick cal", 11),
    0x801B: ("User right stick cal", 11),
    0x8026: ("User IMU cal", 26),
    0x5000: ("Shipment state", 1),
}

# Subcommand names for decoding
SUBCMD_NAMES = {
    0x01: "Bluetooth manual pairing",
    0x02: "Request device info",
    0x03: "Set input report mode",
    0x04: "Trigger buttons elapsed time",
    0x05: "Get page list state",
    0x06: "Set HCI state",
    0x07: "Reset pairing info",
    0x08: "Set shipment low power state",
    0x10: "SPI flash read",
    0x11: "SPI flash write",
    0x12: "SPI sector erase",
    0x21: "Set NFC/IR MCU config",
    0x22: "Set NFC/IR MCU state",
    0x30: "Set player lights",
    0x31: "Get player lights",
    0x38: "Set HOME light",
    0x40: "Enable IMU (6-axis)",
    0x41: "Set IMU sensitivity",
    0x42: "Write to IMU registers",
    0x43: "Read IMU registers",
    0x48: "Enable vibration",
    0x50: "Get regulated voltage",
}


@dataclass
class ProbeSession:
    """Tracks state for a protocol probing session."""

    device: hid.Device | None = None
    packet_counter: int = 0
    handshake_done: bool = False
    recording: bool = False
    record_file: TextIO | None = None
    record_path: str = ""
    tx_count: int = 0
    rx_count: int = 0
    errors: list = field(default_factory=list)

    def next_counter(self) -> int:
        val = self.packet_counter
        self.packet_counter = (self.packet_counter + 1) & 0x0F
        return val


def hex_dump(data: bytes | list[int], prefix: str = "") -> str:
    """Format bytes as a hex dump with ASCII sidebar."""
    if isinstance(data, list):
        data = bytes(data)
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i : i + 16]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{prefix}{i:04X}: {hex_part:<48} |{ascii_part}|")
    return "\n".join(lines)


def decode_buttons(right: int, shared: int, left: int) -> list[str]:
    """Decode button bytes into human-readable names."""
    pressed = []
    for mask, name in BUTTONS_RIGHT.items():
        if right & mask:
            pressed.append(name)
    for mask, name in BUTTONS_SHARED.items():
        if shared & mask:
            pressed.append(name)
    for mask, name in BUTTONS_LEFT.items():
        if left & mask:
            pressed.append(name)
    return pressed


def decode_stick(data: bytes, offset: int) -> tuple[int, int]:
    """Decode 12-bit packed stick data into (x, y)."""
    x = data[offset] | ((data[offset + 1] & 0x0F) << 8)
    y = ((data[offset + 1] >> 4) & 0x0F) | (data[offset + 2] << 4)
    return x, y


def decode_input_report(data: bytes) -> dict:
    """Decode a 0x30 standard input report."""
    if len(data) < 13:
        return {"error": "report too short"}

    timer = data[0]
    battery = (data[1] >> 4) & 0x0F
    conn_info = data[1] & 0x0F
    buttons = decode_buttons(data[2], data[3], data[4])
    lx, ly = decode_stick(data, 5)
    rx, ry = decode_stick(data, 8)

    return {
        "timer": timer,
        "battery": battery,
        "connection": f"0x{conn_info:01X}",
        "buttons": buttons,
        "left_stick": {"x": lx, "y": ly},
        "right_stick": {"x": rx, "y": ry},
    }


def decode_subcmd_reply(data: bytes) -> dict:
    """Decode a 0x21 sub-command reply."""
    if len(data) < 14:
        return {"error": "reply too short"}

    timer = data[0]
    ack = data[12]
    subcmd_id = data[13]
    subcmd_name = SUBCMD_NAMES.get(subcmd_id, f"Unknown(0x{subcmd_id:02X})")
    reply_data = data[14:]

    result = {
        "timer": timer,
        "ack": f"0x{ack:02X}",
        "ack_success": bool(ack & 0x80),
        "subcmd_id": f"0x{subcmd_id:02X}",
        "subcmd_name": subcmd_name,
        "reply_data_hex": " ".join(f"{b:02X}" for b in reply_data),
    }

    # Decode specific subcommand replies
    if subcmd_id == 0x02 and len(reply_data) >= 12:
        result["firmware"] = f"{reply_data[0]}.{reply_data[1]}"
        device_types = {0x01: "Joy-Con L", 0x02: "Joy-Con R", 0x03: "Pro Controller"}
        result["device_type"] = device_types.get(
            reply_data[2], f"0x{reply_data[2]:02X}"
        )
        result["mac"] = ":".join(f"{b:02X}" for b in reply_data[4:10])
        color_sources = {0x01: "firmware defaults", 0x02: "SPI colors"}
        result["color_source"] = color_sources.get(
            reply_data[11], f"0x{reply_data[11]:02X}"
        )

    elif subcmd_id == 0x10 and len(reply_data) >= 5:
        addr = struct.unpack_from("<I", bytes(reply_data[:4]))[0]
        size = reply_data[4]
        spi_data = reply_data[5 : 5 + size]
        addr_name = SPI_ADDRESSES.get(addr, (f"0x{addr:04X}",))[0]
        result["spi_addr"] = f"0x{addr:04X} ({addr_name})"
        result["spi_size"] = size
        result["spi_data_hex"] = " ".join(f"{b:02X}" for b in spi_data)

    return result


def record_event(session: ProbeSession, direction: str, report_id: int, data: bytes):
    """Record a TX/RX event to the session log file."""
    if not session.recording or session.record_file is None:
        return
    event = {
        "timestamp": time.time(),
        "direction": direction,
        "report_id": f"0x{report_id:02X}",
        "data_hex": " ".join(f"{b:02X}" for b in data),
        "length": len(data),
    }
    session.record_file.write(json.dumps(event) + "\n")
    session.record_file.flush()


# ---- Device Communication ----


def send_raw(session: ProbeSession, data: bytes) -> None:
    """Send raw bytes to the controller."""
    if session.device is None:
        print("  [!] No device connected")
        return
    # hidapi on some platforms needs the report ID as first byte
    session.device.write(data)
    session.tx_count += 1
    report_id = data[0] if data else 0
    record_event(session, "TX", report_id, data)
    print(f"  TX ({len(data)} bytes):")
    print(hex_dump(data, "    "))


def read_response(session: ProbeSession, timeout_ms: int = 500) -> bytes | None:
    """Read a response from the controller."""
    if session.device is None:
        return None
    data = session.device.read(64, timeout_ms)
    if data:
        data = bytes(data)
        session.rx_count += 1
        report_id = data[0] if data else 0
        record_event(session, "RX", report_id, data)
    return data


def send_usb_cmd(session: ProbeSession, subcmd: int) -> bytes | None:
    """Send a 0x80 USB command and read the 0x81 reply."""
    pkt = bytes([REPORT_USB_CMD, subcmd]) + bytes(62)
    send_raw(session, pkt)
    # Read response (0x80 0x04 and 0x05 have no reply)
    if subcmd in (USB_CMD_FORCE_USB, USB_CMD_DISABLE_TIMEOUT):
        print("  (no reply expected)")
        return None
    resp = read_response(session, timeout_ms=1000)
    if resp:
        print(f"  RX ({len(resp)} bytes):")
        print(hex_dump(resp, "    "))
    else:
        print("  [!] No response (timeout)")
    return resp


def send_subcmd(
    session: ProbeSession, subcmd_id: int, args: bytes = b""
) -> bytes | None:
    """Send a 0x01 subcommand with rumble data and read 0x21 reply."""
    pkt = bytearray(64)
    pkt[0] = REPORT_SUBCMD
    pkt[1] = session.next_counter()
    # Neutral rumble
    pkt[2:10] = bytes([0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40])
    pkt[10] = subcmd_id
    for i, b in enumerate(args):
        if 11 + i < 64:
            pkt[11 + i] = b
    send_raw(session, bytes(pkt))

    # Read reply — may need to drain input reports first
    for _ in range(10):
        resp = read_response(session, timeout_ms=1000)
        if resp is None:
            print("  [!] No response (timeout)")
            return None
        if resp[0] == REPORT_SUBCMD_REPLY:
            print(f"  RX 0x21 reply ({len(resp)} bytes):")
            print(hex_dump(resp, "    "))
            decoded = decode_subcmd_reply(resp[1:])  # skip report ID
            print(f"  Decoded: {json.dumps(decoded, indent=4)}")
            return resp
        elif resp[0] == REPORT_INPUT:
            # Drain input reports while waiting for subcmd reply
            continue
        else:
            print(f"  RX unexpected report 0x{resp[0]:02X} ({len(resp)} bytes):")
            print(hex_dump(resp, "    "))
    print("  [!] No 0x21 reply received after 10 reads")
    return None


# ---- High-Level Commands ----


def cmd_scan():
    """List all HID devices, highlighting Switch controllers."""
    print("\n=== HID Device Scan ===\n")
    devices = hid.enumerate()
    switch_found = False
    for d in devices:
        vid = d["vendor_id"]
        pid = d["product_id"]
        marker = ""
        if vid == SWITCH_PRO_VID and pid == SWITCH_PRO_PID:
            marker = " <<<< SWITCH PRO CONTROLLER"
            switch_found = True
        elif vid == SWITCH_PRO_VID:
            marker = " <<<< NINTENDO DEVICE"
        product = d.get("product_string", "") or ""
        mfg = d.get("manufacturer_string", "") or ""
        path = d.get("path", b"").decode("utf-8", errors="replace")
        print(f"  VID:PID = 0x{vid:04X}:0x{pid:04X}  {mfg} {product}{marker}")
        print(f"    Path: {path}")
        print(f"    Interface: {d.get('interface_number', -1)}")
        print()
    if not switch_found:
        print(
            "  No Switch Pro Controller found.\n"
            "  Connect via USB-C and ensure udev rules are set (Linux)."
        )


def cmd_connect(session: ProbeSession) -> bool:
    """Open the HID device for the Pro Controller."""
    if session.device is not None:
        print("  Already connected. Use 'disconnect' first.")
        return True
    try:
        dev = hid.Device(SWITCH_PRO_VID, SWITCH_PRO_PID)
        session.device = dev
        print(f"  Connected: {dev.manufacturer} {dev.product}")
        print(f"  Serial: {dev.serial}")
        return True
    except hid.HIDException as e:
        print(f"  [!] Failed to open device: {e}")
        print("  Hint: On Linux, check udev rules or run with sudo")
        return False


def cmd_disconnect(session: ProbeSession):
    """Close the HID device."""
    if session.device is not None:
        session.device.close()
        session.device = None
        session.handshake_done = False
        print("  Disconnected")
    else:
        print("  Not connected")


def cmd_handshake(session: ProbeSession):
    """Run the full USB handshake sequence."""
    print("\n=== USB Handshake Sequence ===\n")

    print("--- Step 1: Status Request (0x80 0x01) ---")
    resp = send_usb_cmd(session, USB_CMD_STATUS)
    if resp and len(resp) > 3:
        # Parse MAC from reply
        controller_type = {0x01: "Joy-Con L", 0x02: "Joy-Con R", 0x03: "Pro Controller"}
        if len(resp) >= 10:
            ctype = resp[3] if len(resp) > 3 else 0
            mac = resp[4:10] if len(resp) >= 10 else b""
            print(f"  Controller type: {controller_type.get(ctype, f'0x{ctype:02X}')}")
            print(f"  MAC: {':'.join(f'{b:02X}' for b in mac)}")
    time.sleep(0.1)

    print("\n--- Step 2: Handshake (0x80 0x02) ---")
    send_usb_cmd(session, USB_CMD_HANDSHAKE)
    time.sleep(0.1)

    print("\n--- Step 3: High Speed (0x80 0x03) ---")
    send_usb_cmd(session, USB_CMD_HIGH_SPEED)
    time.sleep(0.1)

    print("\n--- Step 4: Force USB (0x80 0x04) ---")
    send_usb_cmd(session, USB_CMD_FORCE_USB)
    session.handshake_done = True
    time.sleep(0.1)

    print("\n=== Handshake Complete ===\n")


def cmd_info(session: ProbeSession):
    """Request device info (subcommand 0x02)."""
    if not session.handshake_done:
        print("  [!] Run 'handshake' first")
        return
    print("\n=== Device Info (subcmd 0x02) ===\n")
    send_subcmd(session, 0x02)


def cmd_spi_read(session: ProbeSession, addr: int, size: int):
    """Read SPI flash at given address."""
    if not session.handshake_done:
        print("  [!] Run 'handshake' first")
        return
    if size > 29:
        print("  [!] Max SPI read size is 29 (0x1D) bytes")
        size = 29
    addr_name = SPI_ADDRESSES.get(addr, (f"0x{addr:04X}",))[0]
    print(f"\n=== SPI Read: 0x{addr:04X} ({addr_name}), {size} bytes ===\n")
    addr_bytes = struct.pack("<I", addr)
    send_subcmd(session, 0x10, addr_bytes + bytes([size]))


def cmd_dump_calibration(session: ProbeSession):
    """Dump all known calibration data from SPI flash."""
    if not session.handshake_done:
        print("  [!] Run 'handshake' first")
        return
    print("\n=== Full Calibration Dump ===\n")
    for addr, (name, size) in SPI_ADDRESSES.items():
        print(f"--- {name} (0x{addr:04X}, {size} bytes) ---")
        addr_bytes = struct.pack("<I", addr)
        read_size = min(size, 29)
        send_subcmd(session, 0x10, addr_bytes + bytes([read_size]))
        time.sleep(0.05)
        print()


def cmd_dump_colors(session: ProbeSession):
    """Read body, button, and grip colors."""
    if not session.handshake_done:
        print("  [!] Run 'handshake' first")
        return
    print("\n=== Controller Colors ===\n")
    for addr in [0x6050, 0x6053, 0x6056, 0x6059]:
        name = SPI_ADDRESSES[addr][0]
        print(f"--- {name} ---")
        addr_bytes = struct.pack("<I", addr)
        resp = send_subcmd(session, 0x10, addr_bytes + bytes([3]))
        if resp and len(resp) >= 24:
            # Extract RGB from SPI data in reply
            r, g, b = resp[20], resp[21], resp[22]
            print(f"  RGB: ({r}, {g}, {b}) = #{r:02X}{g:02X}{b:02X}")
        time.sleep(0.05)
        print()


def cmd_poll(session: ProbeSession, duration: float = 5.0):
    """Poll input reports for a duration."""
    if session.device is None:
        print("  [!] Not connected")
        return
    print(f"\n=== Polling input reports for {duration}s ===\n")
    # First set input report mode to standard full (0x30)
    if session.handshake_done:
        print("  Setting input report mode to 0x30 (standard full)...")
        send_subcmd(session, 0x03, bytes([0x30]))
        time.sleep(0.1)

    start = time.time()
    count = 0
    while time.time() - start < duration:
        data = read_response(session, timeout_ms=100)
        if data is None:
            continue
        count += 1
        report_id = data[0]
        if report_id == REPORT_INPUT:
            decoded = decode_input_report(data[1:])
            buttons = ", ".join(decoded.get("buttons", [])) or "none"
            ls = decoded.get("left_stick", {})
            rs = decoded.get("right_stick", {})
            print(
                f"  [{count:4d}] 0x30 | "
                f"Btns: {buttons:<30} | "
                f"LS({ls.get('x', 0):4d},{ls.get('y', 0):4d}) "
                f"RS({rs.get('x', 0):4d},{rs.get('y', 0):4d}) | "
                f"Bat:{decoded.get('battery', 0)}"
            )
        elif report_id == REPORT_SIMPLE_INPUT:
            print(
                f"  [{count:4d}] 0x3F simple input: "
                f"{' '.join(f'{b:02X}' for b in data[1:])}"
            )
        else:
            print(
                f"  [{count:4d}] Report 0x{report_id:02X}: "
                f"{' '.join(f'{b:02X}' for b in data[1:12])}"
            )
    print(f"\n  Total: {count} reports in {duration}s ({count / duration:.1f}/s)")


def cmd_buttons(session: ProbeSession):
    """Live button/stick monitor. Ctrl+C to stop."""
    if session.device is None:
        print("  [!] Not connected")
        return
    if session.handshake_done:
        send_subcmd(session, 0x03, bytes([0x30]))
        time.sleep(0.1)

    print("\n=== Live Button Monitor (Ctrl+C to stop) ===\n")
    try:
        while True:
            data = read_response(session, timeout_ms=100)
            if data is None or data[0] != REPORT_INPUT:
                continue
            decoded = decode_input_report(data[1:])
            buttons = ", ".join(decoded.get("buttons", [])) or "-"
            ls = decoded.get("left_stick", {})
            rs = decoded.get("right_stick", {})
            # Overwrite same line
            line = (
                f"\r  Buttons: {buttons:<40} "
                f"LS({ls.get('x', 0):4d},{ls.get('y', 0):4d}) "
                f"RS({rs.get('x', 0):4d},{rs.get('y', 0):4d})"
            )
            print(line, end="", flush=True)
    except KeyboardInterrupt:
        print("\n  Stopped.")


def cmd_record_start(session: ProbeSession, filename: str):
    """Start recording all traffic to a JSONL file."""
    if session.recording:
        print(f"  Already recording to {session.record_path}")
        return
    session.record_path = filename
    session.record_file = open(filename, "a")  # noqa: SIM115
    session.recording = True
    print(f"  Recording to {filename}")


def cmd_record_stop(session: ProbeSession):
    """Stop recording."""
    if not session.recording:
        print("  Not recording")
        return
    session.record_file.close()
    session.record_file = None
    session.recording = False
    print(f"  Stopped recording ({session.record_path})")


def cmd_replay(session: ProbeSession, filename: str):
    """Replay TX commands from a recorded session."""
    if not Path(filename).exists():
        print(f"  [!] File not found: {filename}")
        return
    print(f"\n=== Replaying {filename} ===\n")
    with open(filename) as f:
        for line_num, line in enumerate(f, 1):
            try:
                event = json.loads(line.strip())
            except json.JSONDecodeError:
                continue
            if event.get("direction") != "TX":
                continue
            data = bytes.fromhex(event["data_hex"].replace(" ", ""))
            print(f"  [{line_num}] Replaying {event['report_id']} ({len(data)} bytes)")
            send_raw(session, data)
            resp = read_response(session, timeout_ms=500)
            if resp:
                print(f"  RX ({len(resp)} bytes):")
                print(hex_dump(resp, "    "))
            time.sleep(0.05)
    print("\n=== Replay Complete ===\n")


def cmd_arbitrary_subcmd(session: ProbeSession, subcmd_id: int, args: list[int]):
    """Send an arbitrary subcommand with given args."""
    if not session.handshake_done:
        print("  [!] Run 'handshake' first")
        return
    name = SUBCMD_NAMES.get(subcmd_id, "Unknown")
    print(f"\n=== Subcommand 0x{subcmd_id:02X} ({name}) ===\n")
    send_subcmd(session, subcmd_id, bytes(args))


def cmd_raw_send(session: ProbeSession, hex_bytes: str):
    """Send raw hex bytes as an output report."""
    try:
        data = bytes.fromhex(hex_bytes.replace(" ", ""))
    except ValueError:
        print("  [!] Invalid hex string")
        return
    print(f"\n=== Raw Send ({len(data)} bytes) ===\n")
    send_raw(session, data)
    resp = read_response(session, timeout_ms=500)
    if resp:
        print(f"  RX ({len(resp)} bytes):")
        print(hex_dump(resp, "    "))


def print_help():
    """Print REPL help."""
    print(
        """
Switch Pro Controller USB Protocol Tester — Commands:

  Connection:
    scan                    List all HID devices
    connect                 Open connection to Pro Controller
    disconnect              Close connection

  Protocol:
    handshake               Run full USB handshake (0x80 01→04)
    info                    Request device info (subcmd 0x02)
    spi <addr> <len>        SPI flash read (addr in hex, e.g. spi 6050 3)
    subcmd <id> [args...]   Send subcommand (id and args in hex)
    raw <hex_bytes>         Send raw output report bytes
    set-mode <mode>         Set input report mode (30=standard, 3F=simple)
    enable-imu              Enable IMU (subcmd 0x40 0x01)
    enable-vibration        Enable vibration (subcmd 0x48 0x01)
    set-lights <bitmask>    Set player lights (hex bitmask)

  Monitoring:
    poll [seconds]          Poll input reports (default 5s)
    buttons                 Live button/stick display (Ctrl+C to stop)

  Data Dumps:
    dump-cal                Dump all calibration data
    dump-colors             Read body/button/grip colors

  Recording:
    record <filename>       Start recording traffic to JSONL file
    stop                    Stop recording
    replay <filename>       Replay recorded TX commands

  Session:
    status                  Show session stats
    help                    Show this help
    quit / exit             Exit
"""
    )


def repl(session: ProbeSession):
    """Interactive REPL for protocol exploration."""
    print("\nSwitch Pro Controller USB Protocol Tester")
    print("Type 'help' for commands, 'quit' to exit\n")

    # Auto-connect on start
    if session.device is None:
        print("Attempting auto-connect...")
        cmd_connect(session)

    while True:
        try:
            line = input("probe> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd in ("quit", "exit", "q"):
                break
            elif cmd == "help":
                print_help()
            elif cmd == "scan":
                cmd_scan()
            elif cmd == "connect":
                cmd_connect(session)
            elif cmd == "disconnect":
                cmd_disconnect(session)
            elif cmd == "handshake":
                if session.device is None and not cmd_connect(session):
                    continue
                cmd_handshake(session)
            elif cmd == "info":
                cmd_info(session)
            elif cmd == "spi":
                if len(parts) < 3:
                    print("  Usage: spi <addr_hex> <length>")
                    continue
                addr = int(parts[1], 16)
                size = int(parts[2])
                cmd_spi_read(session, addr, size)
            elif cmd == "subcmd":
                if len(parts) < 2:
                    print("  Usage: subcmd <id_hex> [arg_hex...]")
                    continue
                subcmd_id = int(parts[1], 16)
                args = [int(x, 16) for x in parts[2:]]
                cmd_arbitrary_subcmd(session, subcmd_id, args)
            elif cmd == "raw":
                if len(parts) < 2:
                    print("  Usage: raw <hex_bytes>")
                    continue
                cmd_raw_send(session, " ".join(parts[1:]))
            elif cmd == "poll":
                duration = float(parts[1]) if len(parts) > 1 else 5.0
                cmd_poll(session, duration)
            elif cmd == "buttons":
                cmd_buttons(session)
            elif cmd == "dump-cal":
                cmd_dump_calibration(session)
            elif cmd == "dump-colors":
                cmd_dump_colors(session)
            elif cmd == "record":
                if len(parts) < 2:
                    print("  Usage: record <filename>")
                    continue
                cmd_record_start(session, parts[1])
            elif cmd == "stop":
                cmd_record_stop(session)
            elif cmd == "replay":
                if len(parts) < 2:
                    print("  Usage: replay <filename>")
                    continue
                cmd_replay(session, parts[1])
            elif cmd == "set-mode":
                if len(parts) < 2:
                    print("  Usage: set-mode <mode_hex> (e.g. 30 or 3F)")
                    continue
                mode = int(parts[1], 16)
                cmd_arbitrary_subcmd(session, 0x03, [mode])
            elif cmd == "enable-imu":
                cmd_arbitrary_subcmd(session, 0x40, [0x01])
            elif cmd == "enable-vibration":
                cmd_arbitrary_subcmd(session, 0x48, [0x01])
            elif cmd == "set-lights":
                if len(parts) < 2:
                    print("  Usage: set-lights <bitmask_hex>")
                    continue
                mask = int(parts[1], 16)
                cmd_arbitrary_subcmd(session, 0x30, [mask])
            elif cmd == "status":
                print(f"  Connected: {session.device is not None}")
                print(f"  Handshake: {session.handshake_done}")
                print(f"  TX: {session.tx_count}  RX: {session.rx_count}")
                print(f"  Recording: {session.recording} ({session.record_path})")
            else:
                print(f"  Unknown command: {cmd}. Type 'help' for commands.")
        except Exception as e:
            print(f"  [!] Error: {e}")
            session.errors.append(str(e))


def main():
    parser = argparse.ArgumentParser(
        description="Nintendo Switch Pro Controller USB protocol tester"
    )
    parser.add_argument("--scan", action="store_true", help="List HID devices and exit")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Auto handshake + dump device info",
    )
    parser.add_argument("--record", metavar="FILE", help="Record session to file")
    parser.add_argument("--replay", metavar="FILE", help="Replay recorded commands")
    args = parser.parse_args()

    if args.scan:
        cmd_scan()
        return

    session = ProbeSession()

    if args.record:
        cmd_record_start(session, args.record)

    if args.replay:
        if cmd_connect(session):
            cmd_replay(session, args.replay)
        cmd_disconnect(session)
        return

    if args.auto:
        if cmd_connect(session):
            cmd_handshake(session)
            cmd_info(session)
            cmd_dump_calibration(session)
            cmd_dump_colors(session)
        cmd_disconnect(session)
        return

    try:
        repl(session)
    finally:
        if session.recording:
            cmd_record_stop(session)
        cmd_disconnect(session)
        print(f"\nSession stats: TX={session.tx_count} RX={session.rx_count}")
        if session.errors:
            print(f"Errors: {len(session.errors)}")


if __name__ == "__main__":
    main()
