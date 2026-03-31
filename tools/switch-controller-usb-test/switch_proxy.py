#!/usr/bin/env python3
"""Nintendo Switch Pro Controller protocol handler via ESP32-S3 USB proxy.

The ESP32-S3 runs a thin proxy firmware that forwards raw HID reports
between the Switch and this script over UART. All protocol logic runs
here in Python — iterate without reflashing.

The proxy firmware uses a simple framing protocol:
    [0xAA] [len] [direction] [report_data...] [checksum]

    direction: 'S' (0x53) = from Switch, 'P' (0x50) = from PC

Usage:
    uv run python switch_proxy.py                    # Interactive REPL
    uv run python switch_proxy.py --port /dev/ttyUSB0
    uv run python switch_proxy.py --auto             # Auto-respond to handshake
"""

from __future__ import annotations

import argparse
import json
import readline  # noqa: F401 — enables arrow-key history in input()
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import TextIO

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: uv sync")
    sys.exit(1)

# --- Constants ---

FRAME_SYNC = 0xAA
DIR_FROM_SWITCH = ord("S")  # 0x53
DIR_FROM_PC = ord("P")  # 0x50

# Report IDs
REPORT_INPUT = 0x30
REPORT_SUBCMD_REPLY = 0x21
REPORT_USB_REPLY = 0x81
REPORT_SUBCMD = 0x01
REPORT_RUMBLE = 0x10
REPORT_USB_CMD = 0x80

# Fake MAC address (matches proxy firmware)
FAKE_MAC = bytes([0x00, 0x00, 0x5E, 0x00, 0x53, 0x01])

SUBCMD_NAMES = {
    0x02: "Request device info",
    0x03: "Set input report mode",
    0x04: "Trigger buttons elapsed time",
    0x08: "Set shipment low power state",
    0x10: "SPI flash read",
    0x30: "Set player lights",
    0x38: "Set HOME light",
    0x40: "Enable IMU",
    0x41: "Set IMU sensitivity",
    0x48: "Enable vibration",
}

# SPI flash emulation
SPI_FLASH: dict[int, bytes] = {
    0x6012: bytes([0x03]),
    0x6020: bytes(
        [
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x40,
            0x00,
            0x40,
            0x00,
            0x40,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x3B,
            0x34,
            0x3B,
            0x34,
            0x3B,
            0x34,
        ]
    ),
    0x603D: bytes([0x00, 0x07, 0x70, 0x00, 0x08, 0x80, 0x00, 0x07, 0x70]),
    0x6046: bytes([0x00, 0x08, 0x80, 0x00, 0x07, 0x70, 0x00, 0x07, 0x70]),
    0x6050: bytes([0x32, 0x32, 0x32, 0xFF, 0xFF, 0xFF]),
    0x6056: bytes([0x32, 0x32, 0x32]),
    0x6059: bytes([0x32, 0x32, 0x32]),
    0x6000: bytes([0xFF] * 16),
    0x8010: bytes([0xFF]),
    0x801B: bytes([0xFF]),
    0x8026: bytes([0xFF]),
    0x5000: bytes([0x00]),
}

SPI_LABELS = {
    0x6012: "device type",
    0x6020: "IMU cal",
    0x603D: "L-stick cal",
    0x6046: "R-stick cal",
    0x6050: "body color",
    0x6056: "L-grip",
    0x6059: "R-grip",
    0x6000: "serial",
    0x8010: "user L-stick",
    0x801B: "user R-stick",
    0x8026: "user IMU",
    0x5000: "shipment",
}


@dataclass
class ProxyState:
    """Tracks protocol state."""

    port: serial.Serial | None = None
    timer: int = 0
    handshake_done: bool = False
    setup_done: bool = False
    auto_respond: bool = True
    running: bool = False
    buttons_right: int = 0
    buttons_shared: int = 0
    buttons_left: int = 0
    lx: int = 0x800
    ly: int = 0x800
    rx: int = 0x800
    ry: int = 0x800
    tx_count: int = 0
    rx_count: int = 0
    recording: bool = False
    record_file: TextIO | None = None
    record_path: str = ""
    reader_thread: threading.Thread | None = None
    input_thread: threading.Thread | None = None
    errors: list[str] = field(default_factory=list)
    # Verbose logging
    verbose: bool = False


def hex_line(data: bytes, max_bytes: int = 32) -> str:
    s = " ".join(f"{b:02X}" for b in data[:max_bytes])
    if len(data) > max_bytes:
        s += f" ... (+{len(data) - max_bytes})"
    return s


def record_event(state: ProxyState, direction: str, report_id: int, data: bytes):
    if not state.recording or state.record_file is None:
        return
    event = {
        "timestamp": time.time(),
        "direction": direction,
        "report_id": f"0x{report_id:02X}",
        "data_hex": " ".join(f"{b:02X}" for b in data),
        "length": len(data),
    }
    state.record_file.write(json.dumps(event) + "\n")
    state.record_file.flush()


# --- UART Framing ---


def calc_checksum(data: bytes) -> int:
    xor = 0
    for b in data:
        xor ^= b
    return xor


def send_frame(state: ProxyState, direction: int, report: bytes):
    """Send a framed packet to the proxy firmware."""
    if state.port is None or not state.port.is_open:
        return
    payload = bytes([direction]) + report
    checksum = calc_checksum(payload)
    frame = bytes([FRAME_SYNC, len(payload)]) + payload + bytes([checksum])
    state.port.write(frame)
    state.tx_count += 1
    report_id = report[0] if report else 0
    record_event(state, "TX", report_id, report)


def read_frame(state: ProxyState, timeout: float = 0.05) -> tuple[int, bytes] | None:
    """Read a framed packet from the proxy firmware."""
    if state.port is None or not state.port.is_open:
        return None

    # Read until we get a sync byte
    old_timeout = state.port.timeout
    state.port.timeout = timeout

    try:
        while True:
            byte = state.port.read(1)
            if not byte:
                return None
            if byte[0] == FRAME_SYNC:
                break

        # Read length
        len_byte = state.port.read(1)
        if not len_byte:
            return None
        payload_len = len_byte[0]
        if payload_len < 2 or payload_len > 65:
            return None

        # Read payload + checksum
        data = state.port.read(payload_len + 1)
        if len(data) != payload_len + 1:
            return None

        # Verify checksum
        payload = data[:payload_len]
        expected = calc_checksum(payload)
        if data[payload_len] != expected:
            return None

        direction = payload[0]
        report = payload[1:]
        state.rx_count += 1
        report_id = report[0] if report else 0
        record_event(state, "RX", report_id, report)
        return (direction, bytes(report))
    finally:
        state.port.timeout = old_timeout


# --- Protocol Handlers ---


def pack_sticks(lx: int, ly: int, rx: int, ry: int) -> bytes:
    buf = bytearray(6)
    buf[0] = lx & 0xFF
    buf[1] = ((ly & 0x0F) << 4) | ((lx >> 8) & 0x0F)
    buf[2] = (ly >> 4) & 0xFF
    buf[3] = rx & 0xFF
    buf[4] = ((ry & 0x0F) << 4) | ((rx >> 8) & 0x0F)
    buf[5] = (ry >> 4) & 0xFF
    return bytes(buf)


def build_subcmd_reply(
    state: ProxyState, subcmd_id: int, ack: int = 0x80, reply_data: bytes = b""
) -> bytes:
    """Build a full 0x21 report (report_id + 63 bytes)."""
    report = bytearray(64)
    report[0] = REPORT_SUBCMD_REPLY
    report[1] = state.timer & 0xFF
    state.timer = (state.timer + 1) & 0xFF
    report[2] = 0x8E  # USB, Pro Controller
    # Bytes 3-5: buttons (neutral)
    sticks = pack_sticks(state.lx, state.ly, state.rx, state.ry)
    report[6:12] = sticks
    report[13] = ack
    report[14] = subcmd_id
    copy_len = min(len(reply_data), 49)
    report[15 : 15 + copy_len] = reply_data[:copy_len]
    return bytes(report)


def build_usb_reply(subcmd: int, data: bytes = b"") -> bytes:
    """Build a full 0x81 report."""
    report = bytearray(64)
    report[0] = REPORT_USB_REPLY
    report[1] = subcmd
    copy_len = min(len(data), 62)
    report[2 : 2 + copy_len] = data[:copy_len]
    return bytes(report)


def build_input_report(state: ProxyState) -> bytes:
    """Build a 0x30 standard input report."""
    report = bytearray(64)
    report[0] = REPORT_INPUT
    report[1] = state.timer & 0xFF
    state.timer = (state.timer + 1) & 0xFF
    report[2] = 0x8E
    report[3] = state.buttons_right
    report[4] = state.buttons_shared
    report[5] = state.buttons_left
    sticks = pack_sticks(state.lx, state.ly, state.rx, state.ry)
    report[6:12] = sticks
    return bytes(report)


def spi_read(addr: int, read_len: int) -> bytes:
    """Look up emulated SPI flash data."""
    if addr in SPI_FLASH:
        stored = SPI_FLASH[addr]
        data = stored[:read_len]
        if len(data) < read_len:
            data += bytes(read_len - len(data))
        return data
    for base_addr, stored in SPI_FLASH.items():
        if base_addr <= addr < base_addr + len(stored):
            offset = addr - base_addr
            data = stored[offset : offset + read_len]
            if len(data) < read_len:
                data += bytes(read_len - len(data))
            return data
    return bytes(read_len)


def handle_switch_report(state: ProxyState, report: bytes):
    """Handle an output report from the Switch (forwarded by proxy firmware)."""
    if len(report) < 2:
        return

    report_id = report[0]

    if report_id == REPORT_USB_CMD:
        subcmd = report[1]
        if subcmd == 0x01:
            print("  << USB CMD: Status request")
            if state.auto_respond:
                data = bytes([0x00, 0x03]) + FAKE_MAC
                reply = build_usb_reply(0x01, data)
                send_frame(state, DIR_FROM_PC, reply)
                print("  >> Auto-replied: status + MAC")
        elif subcmd == 0x02:
            print("  << USB CMD: Handshake")
            if state.auto_respond:
                reply = build_usb_reply(0x02)
                send_frame(state, DIR_FROM_PC, reply)
                print("  >> Auto-replied: handshake ACK")
        elif subcmd == 0x03:
            print("  << USB CMD: High speed")
            if state.auto_respond:
                reply = build_usb_reply(0x03)
                send_frame(state, DIR_FROM_PC, reply)
                print("  >> Auto-replied: high speed ACK")
        elif subcmd == 0x04:
            print("  << USB CMD: Force USB — handshake complete!")
            state.handshake_done = True
        elif subcmd == 0x05:
            print("  << USB CMD: Disable USB timeout")
        else:
            print(f"  << USB CMD: Unknown 0x{subcmd:02X}")
            print(f"     {hex_line(report)}")

    elif report_id == REPORT_SUBCMD:
        if len(report) < 11:
            return
        subcmd_id = report[10]
        name = SUBCMD_NAMES.get(subcmd_id, f"Unknown(0x{subcmd_id:02X})")
        print(f"  << Subcmd 0x{subcmd_id:02X}: {name}")

        if not state.auto_respond:
            print(f"     Data: {hex_line(report[11:])}")
            return

        # Auto-respond
        if subcmd_id == 0x02:  # Device info
            reply_data = bytearray(12)
            reply_data[0] = 0x04
            reply_data[1] = 0x33
            reply_data[2] = 0x03
            reply_data[3] = 0x02
            reply_data[4:10] = FAKE_MAC
            reply_data[10] = 0x01
            reply_data[11] = 0x02
            reply = build_subcmd_reply(state, subcmd_id, reply_data=bytes(reply_data))
            send_frame(state, DIR_FROM_PC, reply)
            print("  >> Device info reply")

        elif subcmd_id == 0x10:  # SPI flash read
            if len(report) < 16:
                return
            addr = struct.unpack_from("<I", report, 11)[0]
            read_len = report[15]
            spi_data = spi_read(addr, read_len)
            reply_data = bytearray(
                struct.pack("<I", addr) + bytes([read_len]) + spi_data
            )
            reply = build_subcmd_reply(
                state, subcmd_id, ack=0x90, reply_data=reply_data
            )
            send_frame(state, DIR_FROM_PC, reply)
            label = SPI_LABELS.get(addr, f"0x{addr:04X}")
            print(
                f"  >> SPI 0x{addr:04X} ({label}) [{read_len}B]: {hex_line(spi_data)}"
            )

        elif subcmd_id == 0x30:  # Player lights
            light_mask = report[11] if len(report) > 11 else 0
            reply = build_subcmd_reply(state, subcmd_id)
            send_frame(state, DIR_FROM_PC, reply)
            if not state.setup_done:
                state.setup_done = True
                print(f"  >> Player lights: 0x{light_mask:02X} — SETUP COMPLETE!")
                print(
                    "     Controller is now live. Input reports will be sent at 125 Hz."
                )
            else:
                print(f"  >> Player lights: 0x{light_mask:02X}")

        elif subcmd_id == 0x40:  # Enable IMU
            enable = report[11] if len(report) > 11 else 0
            reply = build_subcmd_reply(state, subcmd_id)
            send_frame(state, DIR_FROM_PC, reply)
            print(f"  >> IMU {'enabled' if enable else 'disabled'}")

        elif subcmd_id == 0x48:  # Enable vibration
            enable = report[11] if len(report) > 11 else 0
            reply = build_subcmd_reply(state, subcmd_id)
            send_frame(state, DIR_FROM_PC, reply)
            print(f"  >> Vibration {'enabled' if enable else 'disabled'}")

        else:
            reply = build_subcmd_reply(state, subcmd_id)
            send_frame(state, DIR_FROM_PC, reply)
            if len(report) > 11:
                args_hex = hex_line(report[11 : min(len(report), 20)])
                print(f"  >> ACK (args: {args_hex})")
            else:
                print("  >> ACK")

    elif report_id == REPORT_RUMBLE:
        if state.verbose:
            print(f"  << Rumble: {hex_line(report[1:9])}")

    else:
        print(f"  << Unknown report 0x{report_id:02X}: {hex_line(report)}")


# --- Background Threads ---


def reader_loop(state: ProxyState):
    """Background thread: reads frames from proxy firmware and handles them."""
    while state.running:
        result = read_frame(state, timeout=0.02)
        if result is None:
            continue
        direction, report = result
        if direction == DIR_FROM_SWITCH:
            handle_switch_report(state, report)
        else:
            # Text messages from the proxy firmware (boot messages etc.)
            try:
                text = report.decode("utf-8", errors="replace").strip()
                if text:
                    print(f"  [proxy] {text}")
            except Exception:
                pass


def input_report_loop(state: ProxyState):
    """Background thread: sends 0x30 input reports at ~125 Hz once setup is done."""
    while state.running:
        if state.setup_done:
            report = build_input_report(state)
            send_frame(state, DIR_FROM_PC, report)
            time.sleep(0.008)  # ~125 Hz
        else:
            time.sleep(0.05)


# --- REPL ---

BUTTON_MAP = {
    "a": ("right", 0x08),
    "b": ("right", 0x04),
    "x": ("right", 0x02),
    "y": ("right", 0x01),
    "r": ("right", 0x40),
    "zr": ("right", 0x80),
    "l": ("left", 0x40),
    "zl": ("left", 0x80),
    "up": ("left", 0x02),
    "down": ("left", 0x01),
    "left": ("left", 0x08),
    "right": ("left", 0x04),
    "plus": ("shared", 0x02),
    "+": ("shared", 0x02),
    "minus": ("shared", 0x01),
    "-": ("shared", 0x01),
    "home": ("shared", 0x10),
    "capture": ("shared", 0x20),
    "ls": ("shared", 0x08),
    "rs": ("shared", 0x04),
}


def cmd_press(state: ProxyState, button_name: str):
    name = button_name.lower()
    if name not in BUTTON_MAP:
        print(f"  Unknown button '{name}'. Valid: {', '.join(sorted(BUTTON_MAP))}")
        return
    group, mask = BUTTON_MAP[name]
    if group == "right":
        state.buttons_right |= mask
    elif group == "shared":
        state.buttons_shared |= mask
    elif group == "left":
        state.buttons_left |= mask
    print(f"  {button_name} pressed")


def cmd_release(state: ProxyState, button_name: str):
    name = button_name.lower()
    if name not in BUTTON_MAP:
        return
    group, mask = BUTTON_MAP[name]
    if group == "right":
        state.buttons_right &= ~mask
    elif group == "shared":
        state.buttons_shared &= ~mask
    elif group == "left":
        state.buttons_left &= ~mask
    print(f"  {button_name} released")


def cmd_tap(state: ProxyState, button_name: str, duration: float = 0.1):
    cmd_press(state, button_name)
    time.sleep(duration)
    cmd_release(state, button_name)


def cmd_list_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  No serial ports found")
        return
    for p in ports:
        print(f"  {p.device}  {p.description}  [{p.hwid}]")


def print_help():
    print("""
Switch Pro Controller USB Proxy — Commands:

  Connection:
    list-ports              List available serial ports
    connect [port]          Connect to proxy firmware (default: auto-detect)
    disconnect              Disconnect

  Protocol:
    auto on|off             Toggle auto-respond mode (default: on)
    verbose on|off          Toggle verbose logging
    wait                    Wait for setup to complete

  Input (sent at 125 Hz after setup):
    press <button>          Press button
                            (a/b/x/y/l/r/zl/zr/up/down/left/right
                             /+/-/home/capture/ls/rs)
    release <button>        Release button
    tap <button> [secs]     Press and release (default 0.1s)
    release-all             Release all buttons + center sticks
    stick l|r <x> <y>       Set stick position (0-4095, center=2048)
    center                  Center both sticks

  SPI Flash:
    spi-set <addr> <hex>    Override SPI data
    spi-dump                Show all SPI flash entries

  Raw:
    send-raw <hex>          Send raw report bytes to Switch
    send-reply <id> <hex>   Send 0x81 reply with given subcmd + data

  Recording:
    record <filename>       Start recording traffic
    stop                    Stop recording

  Session:
    status                  Show state
    help                    Show this help
    quit / exit             Exit
""")


def detect_port() -> str:
    """Try to find the USB-UART adapter."""
    ports = serial.tools.list_ports.comports()
    # Look for common USB-UART chips
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(
            chip in desc or chip in hwid
            for chip in ["cp210", "ch340", "ft232", "pl2303", "uart", "serial"]
        ):
            return p.device
    # Fall back to first port
    if ports:
        return ports[0].device
    return ""


def repl(state: ProxyState, initial_port: str = ""):
    """Interactive REPL."""
    print("\nSwitch Pro Controller USB Proxy Client")
    print("Type 'help' for commands, 'quit' to exit\n")

    if initial_port:
        try:
            state.port = serial.Serial(initial_port, 921600, timeout=0.1)
            state.running = True
            state.reader_thread = threading.Thread(
                target=reader_loop, args=(state,), daemon=True
            )
            state.reader_thread.start()
            state.input_thread = threading.Thread(
                target=input_report_loop, args=(state,), daemon=True
            )
            state.input_thread.start()
            print(f"  Connected to {initial_port}")
        except serial.SerialException as e:
            print(f"  [!] Failed to open {initial_port}: {e}")

    while True:
        try:
            prompt = "proxy"
            if state.setup_done:
                prompt += " [READY]"
            elif state.handshake_done:
                prompt += " [HANDSHAKE]"
            line = input(f"{prompt}> ").strip()
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
            elif cmd == "list-ports":
                cmd_list_ports()
            elif cmd == "connect":
                if state.port and state.port.is_open:
                    print("  Already connected. Use 'disconnect' first.")
                    continue
                port = parts[1] if len(parts) > 1 else detect_port()
                if not port:
                    print("  No serial port found. Specify: connect /dev/ttyUSB0")
                    continue
                state.port = serial.Serial(port, 921600, timeout=0.1)
                state.running = True
                state.reader_thread = threading.Thread(
                    target=reader_loop, args=(state,), daemon=True
                )
                state.reader_thread.start()
                state.input_thread = threading.Thread(
                    target=input_report_loop, args=(state,), daemon=True
                )
                state.input_thread.start()
                print(f"  Connected to {port}")
            elif cmd == "disconnect":
                state.running = False
                if state.port:
                    state.port.close()
                    state.port = None
                state.handshake_done = False
                state.setup_done = False
                print("  Disconnected")
            elif cmd == "auto":
                if len(parts) > 1:
                    state.auto_respond = parts[1].lower() in ("on", "true", "1", "yes")
                print(f"  Auto-respond: {state.auto_respond}")
            elif cmd == "verbose":
                if len(parts) > 1:
                    state.verbose = parts[1].lower() in ("on", "true", "1", "yes")
                print(f"  Verbose: {state.verbose}")
            elif cmd == "wait":
                print("  Waiting for setup to complete...")
                while not state.setup_done:
                    time.sleep(0.1)
                print("  Setup complete!")
            elif cmd == "press":
                if len(parts) < 2:
                    print("  Usage: press <button>")
                    continue
                cmd_press(state, parts[1])
            elif cmd == "release":
                if len(parts) < 2:
                    print("  Usage: release <button>")
                    continue
                cmd_release(state, parts[1])
            elif cmd == "tap":
                if len(parts) < 2:
                    print("  Usage: tap <button> [duration_secs]")
                    continue
                dur = float(parts[2]) if len(parts) > 2 else 0.1
                cmd_tap(state, parts[1], dur)
            elif cmd == "release-all":
                state.buttons_right = 0
                state.buttons_shared = 0
                state.buttons_left = 0
                state.lx = state.ly = 0x800
                state.rx = state.ry = 0x800
                print("  All released")
            elif cmd == "stick":
                if len(parts) < 4:
                    print("  Usage: stick l|r <x> <y>  (0-4095, center=2048)")
                    continue
                x = max(0, min(4095, int(parts[2])))
                y = max(0, min(4095, int(parts[3])))
                if parts[1].lower() in ("l", "left", "ls"):
                    state.lx, state.ly = x, y
                    print(f"  Left stick: ({x}, {y})")
                else:
                    state.rx, state.ry = x, y
                    print(f"  Right stick: ({x}, {y})")
            elif cmd == "center":
                state.lx = state.ly = 0x800
                state.rx = state.ry = 0x800
                print("  Sticks centered")
            elif cmd == "spi-set":
                if len(parts) < 3:
                    print("  Usage: spi-set <addr_hex> <data_hex>")
                    continue
                addr = int(parts[1], 16)
                data = bytes.fromhex(parts[2])
                SPI_FLASH[addr] = data
                print(f"  SPI 0x{addr:04X} = {hex_line(data)}")
            elif cmd == "spi-dump":
                for addr in sorted(SPI_FLASH):
                    label = SPI_LABELS.get(addr, "")
                    data = SPI_FLASH[addr]
                    print(f"  0x{addr:04X} ({label:>12}): {hex_line(data)}")
            elif cmd == "send-raw":
                if len(parts) < 2:
                    print("  Usage: send-raw <hex_bytes>")
                    continue
                data = bytes.fromhex("".join(parts[1:]))
                send_frame(state, DIR_FROM_PC, data)
                print(f"  Sent {len(data)} bytes")
            elif cmd == "send-reply":
                if len(parts) < 2:
                    print("  Usage: send-reply <subcmd_hex> [data_hex]")
                    continue
                subcmd = int(parts[1], 16)
                data = bytes.fromhex(parts[2]) if len(parts) > 2 else b""
                reply = build_usb_reply(subcmd, data)
                send_frame(state, DIR_FROM_PC, reply)
                print(f"  Sent 0x81 reply for subcmd 0x{subcmd:02X}")
            elif cmd == "record":
                if len(parts) < 2:
                    print("  Usage: record <filename>")
                    continue
                state.record_path = parts[1]
                state.record_file = open(parts[1], "a")  # noqa: SIM115
                state.recording = True
                print(f"  Recording to {parts[1]}")
            elif cmd == "stop":
                if state.recording and state.record_file:
                    state.record_file.close()
                    state.record_file = None
                    state.recording = False
                    print(f"  Stopped recording ({state.record_path})")
            elif cmd == "status":
                connected = state.port.port if state.port else "not connected"
                print(f"  Port:      {connected}")
                print(f"  Handshake: {state.handshake_done}")
                print(f"  Setup:     {state.setup_done}")
                print(f"  Auto:      {state.auto_respond}")
                print(f"  TX: {state.tx_count}  RX: {state.rx_count}")
                print(
                    f"  Buttons:   R=0x{state.buttons_right:02X}"
                    f" S=0x{state.buttons_shared:02X}"
                    f" L=0x{state.buttons_left:02X}"
                )
                print(f"  Sticks:    L({state.lx},{state.ly}) R({state.rx},{state.ry})")
            else:
                print(f"  Unknown command: {cmd}. Type 'help' for commands.")
        except Exception as e:
            print(f"  [!] Error: {e}")
            state.errors.append(str(e))


def main():
    parser = argparse.ArgumentParser(
        description="Nintendo Switch Pro Controller protocol handler via USB proxy"
    )
    parser.add_argument(
        "--port", default="", help="Serial port (auto-detected if omitted)"
    )
    parser.add_argument(
        "--baud", type=int, default=921600, help="Baud rate (default: 921600)"
    )
    parser.add_argument(
        "--auto", action="store_true", help="Auto setup and wait for handshake"
    )
    parser.add_argument("--record", metavar="FILE", help="Record traffic to file")
    parser.add_argument("--list-ports", action="store_true", help="List serial ports")
    args = parser.parse_args()

    if args.list_ports:
        cmd_list_ports()
        return

    state = ProxyState()

    if args.record:
        state.record_path = args.record
        state.record_file = open(args.record, "a")  # noqa: SIM115
        state.recording = True

    port = args.port or detect_port()

    if args.auto:
        if not port:
            print("[!] No serial port found. Specify: --port /dev/ttyUSB0")
            sys.exit(1)
        state.port = serial.Serial(port, args.baud, timeout=0.1)
        state.running = True
        state.reader_thread = threading.Thread(
            target=reader_loop, args=(state,), daemon=True
        )
        state.reader_thread.start()
        state.input_thread = threading.Thread(
            target=input_report_loop, args=(state,), daemon=True
        )
        state.input_thread.start()
        print(f"Connected to {port}")
        print("Waiting for Switch to connect and complete handshake...")
        try:
            while not state.setup_done:
                time.sleep(0.1)
            print("Setup complete! Controller is live.")
            print("Press Ctrl+C to stop.\n")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            state.running = False
            if state.port:
                state.port.close()
        return

    try:
        repl(state, initial_port=port)
    finally:
        state.running = False
        if state.recording and state.record_file:
            state.record_file.close()
        if state.port and state.port.is_open:
            state.port.close()
        print(f"\nSession stats: TX={state.tx_count} RX={state.rx_count}")


if __name__ == "__main__":
    main()
