#!/usr/bin/env python3
"""Nintendo Switch Pro Controller emulator via Linux USB Gadget.

Makes the PC appear as a wired Pro Controller to the Switch using
Linux's configfs USB gadget subsystem + /dev/hidgN.

This lets you iterate on protocol responses in pure Python without
any firmware flashing. The Switch plugs directly into the PC.

Requirements:
    - Linux with USB OTG/UDC hardware (Raspberry Pi, some laptops)
    - dwc2 or similar UDC driver loaded
    - Root access (for configfs + /dev/hidgN)

Usage:
    sudo uv run python switch_gadget.py              # Interactive mode
    sudo uv run python switch_gadget.py --auto       # Auto-handshake
    sudo uv run python switch_gadget.py --udc dummy  # Override UDC name
    sudo uv run python switch_gadget.py --list-udc   # List available UDCs
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import readline  # noqa: F401 — enables arrow-key history in input()
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TextIO

# --- Constants ---

VID = 0x057E
PID = 0x2009
MANUFACTURER = "Nintendo Co., Ltd."
PRODUCT = "Pro Controller"
SERIAL = "000000000001"

# Report IDs
REPORT_INPUT = 0x30
REPORT_SUBCMD_REPLY = 0x21
REPORT_USB_REPLY = 0x81
REPORT_SUBCMD = 0x01
REPORT_RUMBLE = 0x10
REPORT_USB_CMD = 0x80

# HID report descriptor (must match the real Pro Controller)
HID_REPORT_DESCRIPTOR = bytes(
    [
        0x05,
        0x01,
        0x15,
        0x00,
        0x09,
        0x04,
        0xA1,
        0x01,
        # Report 0x30: Standard input
        0x85,
        0x30,
        0x05,
        0x01,
        0x05,
        0x09,
        0x19,
        0x01,
        0x29,
        0x0A,
        0x15,
        0x00,
        0x25,
        0x01,
        0x75,
        0x01,
        0x95,
        0x0A,
        0x55,
        0x00,
        0x65,
        0x00,
        0x81,
        0x02,
        0x05,
        0x09,
        0x19,
        0x0B,
        0x29,
        0x0E,
        0x15,
        0x00,
        0x25,
        0x01,
        0x75,
        0x01,
        0x95,
        0x04,
        0x81,
        0x02,
        0x75,
        0x01,
        0x95,
        0x02,
        0x81,
        0x03,
        0x0B,
        0x01,
        0x00,
        0x01,
        0x00,
        0xA1,
        0x00,
        0x0B,
        0x30,
        0x00,
        0x01,
        0x00,
        0x0B,
        0x31,
        0x00,
        0x01,
        0x00,
        0x0B,
        0x32,
        0x00,
        0x01,
        0x00,
        0x0B,
        0x35,
        0x00,
        0x01,
        0x00,
        0x15,
        0x00,
        0x27,
        0xFF,
        0xFF,
        0x00,
        0x00,
        0x75,
        0x10,
        0x95,
        0x04,
        0x81,
        0x02,
        0xC0,
        0x0B,
        0x39,
        0x00,
        0x01,
        0x00,
        0x15,
        0x00,
        0x25,
        0x07,
        0x35,
        0x00,
        0x46,
        0x3B,
        0x01,
        0x65,
        0x14,
        0x75,
        0x04,
        0x95,
        0x01,
        0x81,
        0x02,
        0x05,
        0x09,
        0x19,
        0x0F,
        0x29,
        0x12,
        0x15,
        0x00,
        0x25,
        0x01,
        0x75,
        0x01,
        0x95,
        0x04,
        0x81,
        0x02,
        0x75,
        0x08,
        0x95,
        0x34,
        0x81,
        0x03,
        # Report 0x21: Sub-command reply (input)
        0x06,
        0x00,
        0xFF,
        0x85,
        0x21,
        0x09,
        0x01,
        0x75,
        0x08,
        0x95,
        0x3F,
        0x81,
        0x03,
        # Report 0x81: USB command reply (input)
        0x85,
        0x81,
        0x09,
        0x02,
        0x75,
        0x08,
        0x95,
        0x3F,
        0x81,
        0x03,
        # Report 0x01: Sub-command (output)
        0x85,
        0x01,
        0x09,
        0x03,
        0x75,
        0x08,
        0x95,
        0x3F,
        0x91,
        0x83,
        # Report 0x10: Rumble only (output)
        0x85,
        0x10,
        0x09,
        0x04,
        0x75,
        0x08,
        0x95,
        0x3F,
        0x91,
        0x83,
        # Report 0x80: USB command (output)
        0x85,
        0x80,
        0x09,
        0x05,
        0x75,
        0x08,
        0x95,
        0x3F,
        0x91,
        0x83,
        # Report 0x82: (output - unused)
        0x85,
        0x82,
        0x09,
        0x06,
        0x75,
        0x08,
        0x95,
        0x3F,
        0x91,
        0x83,
        0xC0,
    ]
)

CONFIGFS_BASE = "/sys/kernel/config/usb_gadget"
GADGET_NAME = "switch_pro"

# Fake MAC address
FAKE_MAC = bytes([0x00, 0x00, 0x5E, 0x00, 0x53, 0x01])

# Button bitmasks (for display)
BUTTONS_RIGHT = {0x01: "Y", 0x02: "X", 0x04: "B", 0x08: "A", 0x40: "R", 0x80: "ZR"}
BUTTONS_SHARED = {
    0x01: "-",
    0x02: "+",
    0x04: "RS",
    0x08: "LS",
    0x10: "Home",
    0x20: "Cap",
}
BUTTONS_LEFT = {
    0x01: "Down",
    0x02: "Up",
    0x04: "Right",
    0x08: "Left",
    0x40: "L",
    0x80: "ZL",
}

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

# SPI flash emulation data
SPI_FLASH: dict[int, bytes] = {
    0x6012: bytes([0x03]),  # Device type: Pro Controller
    0x6020: bytes(
        [  # Factory IMU calibration (24 bytes)
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,  # Accel origin
            0x00,
            0x40,
            0x00,
            0x40,
            0x00,
            0x40,  # Accel sensitivity
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,  # Gyro origin
            0x3B,
            0x34,
            0x3B,
            0x34,
            0x3B,
            0x34,  # Gyro sensitivity
        ]
    ),
    0x603D: bytes(
        [0x00, 0x07, 0x70, 0x00, 0x08, 0x80, 0x00, 0x07, 0x70]
    ),  # Left stick cal
    0x6046: bytes(
        [0x00, 0x08, 0x80, 0x00, 0x07, 0x70, 0x00, 0x07, 0x70]
    ),  # Right stick cal
    0x6050: bytes([0x32, 0x32, 0x32, 0xFF, 0xFF, 0xFF]),  # Body + button color
    0x6056: bytes([0x32, 0x32, 0x32]),  # Left grip
    0x6059: bytes([0x32, 0x32, 0x32]),  # Right grip
    0x6000: bytes([0xFF] * 16),  # Serial number (unused)
    0x8010: bytes([0xFF]),  # User left stick cal (none)
    0x801B: bytes([0xFF]),  # User right stick cal (none)
    0x8026: bytes([0xFF]),  # User IMU cal (none)
    0x5000: bytes([0x00]),  # Shipment state
}


@dataclass
class GadgetState:
    """Tracks the emulated controller state."""

    hidg_fd: int | None = None
    hidg_path: str = ""
    gadget_path: str = ""
    udc_name: str = ""
    timer: int = 0
    handshake_done: bool = False
    setup_done: bool = False
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
    input_thread: threading.Thread | None = None
    errors: list[str] = field(default_factory=list)


def hex_dump(data: bytes, prefix: str = "") -> str:
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i : i + 16]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{prefix}{i:04X}: {hex_part:<48} |{ascii_part}|")
    return "\n".join(lines)


def record_event(state: GadgetState, direction: str, report_id: int, data: bytes):
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


# --- Configfs USB Gadget Setup ---


def find_udc() -> str:
    """Find the first available USB Device Controller."""
    udc_dir = Path("/sys/class/udc")
    if not udc_dir.exists():
        return ""
    udcs = list(udc_dir.iterdir())
    return udcs[0].name if udcs else ""


def list_udcs():
    """List all available UDCs."""
    udc_dir = Path("/sys/class/udc")
    if not udc_dir.exists():
        print("  No /sys/class/udc directory — USB gadget not supported on this kernel")
        return
    udcs = list(udc_dir.iterdir())
    if not udcs:
        print("  No UDC drivers found.")
        print("  On Raspberry Pi, add 'dtoverlay=dwc2' to /boot/config.txt")
        print("  Then: sudo modprobe dwc2 libcomposite")
        return
    print("  Available UDCs:")
    for u in udcs:
        state = (
            (u / "state").read_text().strip() if (u / "state").exists() else "unknown"
        )
        print(f"    {u.name}  (state: {state})")


def setup_gadget(state: GadgetState, udc: str = "") -> bool:
    """Set up the USB gadget via configfs."""
    if os.geteuid() != 0:
        print("[!] Must run as root (sudo) for USB gadget access")
        return False

    # Load required kernel modules
    os.system("modprobe libcomposite 2>/dev/null")
    os.system("modprobe usb_f_hid 2>/dev/null")

    if not udc:
        udc = find_udc()
        if not udc:
            print("[!] No USB Device Controller found.")
            print("    This machine may not have USB OTG hardware.")
            print("    On Raspberry Pi: add 'dtoverlay=dwc2' to /boot/config.txt")
            return False

    state.udc_name = udc
    gadget_path = Path(CONFIGFS_BASE) / GADGET_NAME
    state.gadget_path = str(gadget_path)

    # Clean up any existing gadget
    if gadget_path.exists():
        teardown_gadget(state)

    print(f"  Setting up USB gadget '{GADGET_NAME}' on UDC '{udc}'...")

    try:
        # Create gadget directory
        gadget_path.mkdir(parents=True, exist_ok=True)

        # Set IDs
        (gadget_path / "idVendor").write_text(f"0x{VID:04x}")
        (gadget_path / "idProduct").write_text(f"0x{PID:04x}")
        (gadget_path / "bcdDevice").write_text("0x0200")
        (gadget_path / "bcdUSB").write_text("0x0200")

        # Strings
        strings = gadget_path / "strings" / "0x409"
        strings.mkdir(parents=True, exist_ok=True)
        (strings / "manufacturer").write_text(MANUFACTURER)
        (strings / "product").write_text(PRODUCT)
        (strings / "serialnumber").write_text(SERIAL)

        # Configuration
        config = gadget_path / "configs" / "c.1"
        config.mkdir(parents=True, exist_ok=True)
        (config / "MaxPower").write_text("500")
        config_strings = config / "strings" / "0x409"
        config_strings.mkdir(parents=True, exist_ok=True)
        (config_strings / "configuration").write_text("Pro Controller Config")

        # HID function
        hid_func = gadget_path / "functions" / "hid.usb0"
        hid_func.mkdir(parents=True, exist_ok=True)
        (hid_func / "protocol").write_text("0")
        (hid_func / "subclass").write_text("0")
        (hid_func / "report_length").write_text("64")
        (hid_func / "report_desc").write_bytes(HID_REPORT_DESCRIPTOR)

        # Link function to config
        link = config / "hid.usb0"
        if not link.exists():
            link.symlink_to(hid_func)

        # Bind to UDC
        (gadget_path / "UDC").write_text(udc)

        # Wait for /dev/hidg0 to appear
        hidg_path = "/dev/hidg0"
        for _ in range(20):
            if Path(hidg_path).exists():
                break
            time.sleep(0.1)
        else:
            print(f"  [!] {hidg_path} did not appear after gadget setup")
            return False

        # Open the HID gadget device
        state.hidg_fd = os.open(hidg_path, os.O_RDWR | os.O_NONBLOCK)
        state.hidg_path = hidg_path

        print(f"  USB gadget active on {udc}")
        print(f"  HID device: {hidg_path}")
        print("  Connect the Switch dock USB to this machine's OTG port")
        return True

    except Exception as e:
        print(f"  [!] Failed to set up gadget: {e}")
        teardown_gadget(state)
        return False


def teardown_gadget(state: GadgetState):
    """Remove the USB gadget from configfs."""
    if state.hidg_fd is not None:
        with contextlib.suppress(OSError):
            os.close(state.hidg_fd)
        state.hidg_fd = None

    gadget_path = Path(CONFIGFS_BASE) / GADGET_NAME
    if not gadget_path.exists():
        return

    try:
        # Unbind from UDC
        udc_file = gadget_path / "UDC"
        if udc_file.exists():
            udc_file.write_text("")

        # Remove function link from config
        link = gadget_path / "configs" / "c.1" / "hid.usb0"
        if link.is_symlink():
            link.unlink()

        # Remove config strings
        config_strings = gadget_path / "configs" / "c.1" / "strings" / "0x409"
        if config_strings.exists():
            config_strings.rmdir()

        # Remove config
        config = gadget_path / "configs" / "c.1"
        if config.exists():
            config.rmdir()

        # Remove function
        hid_func = gadget_path / "functions" / "hid.usb0"
        if hid_func.exists():
            hid_func.rmdir()

        # Remove strings
        strings = gadget_path / "strings" / "0x409"
        if strings.exists():
            strings.rmdir()

        # Remove gadget
        gadget_path.rmdir()

        print("  USB gadget removed")
    except Exception as e:
        print(f"  [!] Error cleaning up gadget: {e}")


# --- HID I/O ---


def send_report(state: GadgetState, report_id: int, data: bytes):
    """Send an input report (controller → Switch) via /dev/hidgN."""
    if state.hidg_fd is None:
        return
    # Linux HID gadget expects report_id as first byte
    pkt = bytes([report_id]) + data
    try:
        os.write(state.hidg_fd, pkt)
        state.tx_count += 1
        record_event(state, "TX", report_id, pkt)
    except BlockingIOError:
        pass  # Switch not ready yet
    except OSError as e:
        if e.errno != 11:  # EAGAIN
            state.errors.append(f"write: {e}")


def read_output_report(state: GadgetState, timeout: float = 0.5) -> bytes | None:
    """Read an output report (Switch → controller) from /dev/hidgN."""
    if state.hidg_fd is None:
        return None
    import select

    readable, _, _ = select.select([state.hidg_fd], [], [], timeout)
    if not readable:
        return None
    try:
        data = os.read(state.hidg_fd, 64)
        if data:
            state.rx_count += 1
            report_id = data[0] if data else 0
            record_event(state, "RX", report_id, data)
        return data
    except (BlockingIOError, OSError):
        return None


# --- Protocol Handlers ---


def pack_sticks(lx: int, ly: int, rx: int, ry: int) -> bytes:
    """Pack 12-bit stick values into 6 bytes."""
    buf = bytearray(6)
    buf[0] = lx & 0xFF
    buf[1] = ((ly & 0x0F) << 4) | ((lx >> 8) & 0x0F)
    buf[2] = (ly >> 4) & 0xFF
    buf[3] = rx & 0xFF
    buf[4] = ((ry & 0x0F) << 4) | ((rx >> 8) & 0x0F)
    buf[5] = (ry >> 4) & 0xFF
    return bytes(buf)


def build_subcmd_reply(
    state: GadgetState, subcmd_id: int, ack: int = 0x80, reply_data: bytes = b""
) -> bytes:
    """Build a 0x21 sub-command reply (63 bytes after report ID)."""
    reply = bytearray(63)
    reply[0] = state.timer & 0xFF
    state.timer = (state.timer + 1) & 0xFF
    reply[1] = 0x8E  # USB, Pro Controller
    # Bytes 2-4: buttons (neutral)
    # Bytes 5-10: sticks (neutral center)
    stick_data = pack_sticks(0x800, 0x800, 0x800, 0x800)
    reply[5:11] = stick_data
    reply[12] = ack
    reply[13] = subcmd_id
    # Copy reply data (max 49 bytes = 63 - 14)
    copy_len = min(len(reply_data), 49)
    reply[14 : 14 + copy_len] = reply_data[:copy_len]
    return bytes(reply)


def handle_spi_read(addr: int, read_len: int) -> bytes:
    """Look up emulated SPI flash data for the given address."""
    # Check exact match first
    if addr in SPI_FLASH:
        stored = SPI_FLASH[addr]
        data = stored[:read_len]
        if len(data) < read_len:
            data += bytes(read_len - len(data))
        return data

    # Check if addr falls within a stored range
    for base_addr, stored in SPI_FLASH.items():
        if base_addr <= addr < base_addr + len(stored):
            offset = addr - base_addr
            data = stored[offset : offset + read_len]
            if len(data) < read_len:
                data += bytes(read_len - len(data))
            return data

    # Unknown address: return zeros
    return bytes(read_len)


def handle_output_report(state: GadgetState, data: bytes):
    """Process an output report from the Switch."""
    if len(data) < 2:
        return

    report_id = data[0]

    if report_id == REPORT_USB_CMD:
        handle_usb_cmd(state, data)
    elif report_id == REPORT_SUBCMD:
        handle_subcmd(state, data)
    elif report_id == REPORT_RUMBLE:
        # Rumble-only: just ACK it
        pass
    else:
        print(f"  [?] Unknown output report 0x{report_id:02X}")
        print(hex_dump(data, "      "))


def handle_usb_cmd(state: GadgetState, data: bytes):
    """Handle 0x80 USB handshake commands from the Switch."""
    subcmd = data[1] if len(data) > 1 else 0
    response = bytearray(63)

    if subcmd == 0x01:  # Status
        print("  << USB CMD: Status request")
        response[0] = 0x01
        response[1] = 0x00  # Padding
        response[2] = 0x03  # Pro Controller
        response[3:9] = FAKE_MAC
        send_report(state, REPORT_USB_REPLY, bytes(response))
        print("  >> Replied with status + MAC")

    elif subcmd == 0x02:  # Handshake
        print("  << USB CMD: Handshake")
        response[0] = 0x02
        send_report(state, REPORT_USB_REPLY, bytes(response))
        print("  >> Replied with handshake ACK")

    elif subcmd == 0x03:  # High speed
        print("  << USB CMD: High speed")
        response[0] = 0x03
        send_report(state, REPORT_USB_REPLY, bytes(response))
        print("  >> Replied with high speed ACK")

    elif subcmd == 0x04:  # Force USB
        print("  << USB CMD: Force USB — handshake complete!")
        state.handshake_done = True

    elif subcmd == 0x05:  # Disable USB timeout
        print("  << USB CMD: Disable USB timeout (no reply)")

    else:
        print(f"  << USB CMD: Unknown 0x{subcmd:02X}")
        print(hex_dump(data, "      "))


def handle_subcmd(state: GadgetState, data: bytes):
    """Handle 0x01 sub-command from the Switch."""
    if len(data) < 11:
        return

    subcmd_id = data[10]
    name = SUBCMD_NAMES.get(subcmd_id, f"Unknown(0x{subcmd_id:02X})")
    print(f"  << Subcmd 0x{subcmd_id:02X}: {name}")

    if subcmd_id == 0x02:  # Request device info
        reply_data = bytearray(12)
        reply_data[0] = 0x04  # FW major
        reply_data[1] = 0x33  # FW minor
        reply_data[2] = 0x03  # Pro Controller
        reply_data[3] = 0x02  # Unknown
        reply_data[4:10] = FAKE_MAC
        reply_data[10] = 0x01  # Unknown
        reply_data[11] = 0x02  # SPI colors
        reply = build_subcmd_reply(state, subcmd_id, reply_data=bytes(reply_data))
        send_report(state, REPORT_SUBCMD_REPLY, reply)
        print("  >> Device info reply sent")

    elif subcmd_id == 0x10:  # SPI flash read
        if len(data) < 16:
            return
        addr = struct.unpack_from("<I", data, 11)[0]
        read_len = data[15]
        spi_data = handle_spi_read(addr, read_len)
        # Reply: addr(4) + len(1) + data
        reply_data = bytearray(struct.pack("<I", addr) + bytes([read_len]) + spi_data)
        reply = build_subcmd_reply(state, subcmd_id, ack=0x90, reply_data=reply_data)
        send_report(state, REPORT_SUBCMD_REPLY, reply)
        addr_label = ""
        for base, stored in SPI_FLASH.items():
            if base <= addr < base + len(stored) or base == addr:
                for known_addr, _known_data in SPI_FLASH.items():
                    if known_addr == addr:
                        # Find label from well-known addresses
                        labels = {
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
                        addr_label = labels.get(addr, "")
                        break
                break
        label = f" ({addr_label})" if addr_label else ""
        print(
            f"  >> SPI read 0x{addr:04X}{label} [{read_len}B]: "
            f"{' '.join(f'{b:02X}' for b in spi_data)}"
        )

    elif subcmd_id == 0x30:  # Set player lights
        light_mask = data[11] if len(data) > 11 else 0
        reply = build_subcmd_reply(state, subcmd_id)
        send_report(state, REPORT_SUBCMD_REPLY, reply)
        if not state.setup_done:
            state.setup_done = True
            print(f"  >> Player lights: 0x{light_mask:02X} — SETUP COMPLETE")
        else:
            print(f"  >> Player lights: 0x{light_mask:02X}")

    elif subcmd_id == 0x40:  # Enable IMU
        enable = data[11] if len(data) > 11 else 0
        reply = build_subcmd_reply(state, subcmd_id)
        send_report(state, REPORT_SUBCMD_REPLY, reply)
        print(f"  >> IMU {'enabled' if enable else 'disabled'}")

    elif subcmd_id == 0x48:  # Enable vibration
        enable = data[11] if len(data) > 11 else 0
        reply = build_subcmd_reply(state, subcmd_id)
        send_report(state, REPORT_SUBCMD_REPLY, reply)
        print(f"  >> Vibration {'enabled' if enable else 'disabled'}")

    else:
        # Generic ACK for anything else
        reply = build_subcmd_reply(state, subcmd_id)
        send_report(state, REPORT_SUBCMD_REPLY, reply)
        if len(data) > 11:
            args_hex = " ".join(f"{b:02X}" for b in data[11 : min(len(data), 20)])
            print(f"  >> ACK (args: {args_hex})")
        else:
            print("  >> ACK")


def send_input_report(state: GadgetState):
    """Send a 0x30 standard input report with current button/stick state."""
    report = bytearray(63)
    report[0] = state.timer & 0xFF
    state.timer = (state.timer + 1) & 0xFF
    report[1] = 0x8E  # USB, Pro Controller
    report[2] = state.buttons_right
    report[3] = state.buttons_shared
    report[4] = state.buttons_left
    stick_data = pack_sticks(state.lx, state.ly, state.rx, state.ry)
    report[5:11] = stick_data
    # Bytes 12-47: IMU data (zeros)
    send_report(state, REPORT_INPUT, bytes(report))


# --- Input Reader Thread ---


def input_reader_loop(state: GadgetState):
    """Background thread that reads output reports from the Switch."""
    while state.running:
        data = read_output_report(state, timeout=0.05)
        if data:
            handle_output_report(state, data)

        # Send continuous input reports once setup is done
        if state.setup_done:
            send_input_report(state)
            time.sleep(0.008)  # ~125 Hz


# --- REPL ---


def cmd_status(state: GadgetState):
    print(f"  Gadget:    {state.gadget_path or 'not set up'}")
    print(f"  UDC:       {state.udc_name or 'none'}")
    print(f"  HID:       {state.hidg_path or 'not open'}")
    print(f"  Handshake: {state.handshake_done}")
    print(f"  Setup:     {state.setup_done}")
    print(f"  TX: {state.tx_count}  RX: {state.rx_count}")
    print(
        f"  Buttons:   R=0x{state.buttons_right:02X} S=0x{state.buttons_shared:02X}"
        f" L=0x{state.buttons_left:02X}"
    )
    print(f"  Sticks:    L({state.lx},{state.ly}) R({state.rx},{state.ry})")
    print(f"  Recording: {state.recording}")


def cmd_press(state: GadgetState, button_name: str):
    """Press a button by name."""
    button_map = {
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
    name = button_name.lower()
    if name not in button_map:
        print(f"  Unknown button '{name}'. Valid: {', '.join(sorted(button_map))}")
        return
    group, mask = button_map[name]
    if group == "right":
        state.buttons_right |= mask
    elif group == "shared":
        state.buttons_shared |= mask
    elif group == "left":
        state.buttons_left |= mask
    print(f"  {button_name} pressed")


def cmd_release(state: GadgetState, button_name: str):
    """Release a button by name."""
    button_map = {
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
    name = button_name.lower()
    if name not in button_map:
        print(f"  Unknown button '{name}'.")
        return
    group, mask = button_map[name]
    if group == "right":
        state.buttons_right &= ~mask
    elif group == "shared":
        state.buttons_shared &= ~mask
    elif group == "left":
        state.buttons_left &= ~mask
    print(f"  {button_name} released")


def cmd_tap(state: GadgetState, button_name: str, duration: float = 0.1):
    """Press and release a button."""
    cmd_press(state, button_name)
    time.sleep(duration)
    cmd_release(state, button_name)


def cmd_stick(state: GadgetState, stick: str, x: int, y: int):
    """Set stick position (0-4095, center=2048)."""
    x = max(0, min(4095, x))
    y = max(0, min(4095, y))
    if stick.lower() in ("l", "left", "ls"):
        state.lx, state.ly = x, y
        print(f"  Left stick: ({x}, {y})")
    elif stick.lower() in ("r", "right", "rs"):
        state.rx, state.ry = x, y
        print(f"  Right stick: ({x}, {y})")
    else:
        print("  Usage: stick l|r <x> <y>  (0-4095, center=2048)")


def cmd_center(state: GadgetState):
    """Center both sticks."""
    state.lx = state.ly = 0x800
    state.rx = state.ry = 0x800
    print("  Sticks centered")


def cmd_release_all(state: GadgetState):
    """Release all buttons and center sticks."""
    state.buttons_right = 0
    state.buttons_shared = 0
    state.buttons_left = 0
    state.lx = state.ly = 0x800
    state.rx = state.ry = 0x800
    print("  All released")


def cmd_spi_set(addr: int, data: bytes):
    """Override SPI flash data for testing."""
    SPI_FLASH[addr] = data
    print(f"  SPI 0x{addr:04X} = {' '.join(f'{b:02X}' for b in data)}")


def print_help():
    print("""
Switch Pro Controller USB Gadget Emulator — Commands:

  Setup:
    list-udc                List available USB Device Controllers
    setup [udc]             Set up USB gadget (auto-detects UDC if omitted)
    teardown                Remove USB gadget
    status                  Show state

  Protocol (auto-handled in background, but can be manually triggered):
    wait                    Wait for Switch handshake to complete

  Input (sent at 125 Hz once setup is done):
    press <button>          Press button (a/b/x/y/l/r/zl/zr/up/down/left/right/+/-/home/capture/ls/rs)
    release <button>        Release button
    tap <button> [secs]     Press and release (default 0.1s)
    release-all             Release all buttons + center sticks
    stick l|r <x> <y>       Set stick position (0-4095, center=2048)
    center                  Center both sticks

  SPI Flash (modify emulated data):
    spi-set <addr> <hex>    Override SPI data (e.g. spi-set 6050 FF0000)

  Recording:
    record <filename>       Start recording traffic
    stop                    Stop recording

  Session:
    help                    Show this help
    quit / exit             Exit and remove gadget
""")


def repl(state: GadgetState):
    """Interactive REPL."""
    print("\nSwitch Pro Controller USB Gadget Emulator")
    print("Type 'help' for commands, 'quit' to exit\n")

    while True:
        try:
            prompt = "gadget"
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
            elif cmd == "list-udc":
                list_udcs()
            elif cmd == "setup":
                udc = parts[1] if len(parts) > 1 else ""
                if setup_gadget(state, udc):
                    # Start background reader thread
                    state.running = True
                    state.input_thread = threading.Thread(
                        target=input_reader_loop, args=(state,), daemon=True
                    )
                    state.input_thread.start()
                    print("  Background protocol handler started")
            elif cmd == "teardown":
                state.running = False
                if state.input_thread:
                    state.input_thread.join(timeout=2)
                teardown_gadget(state)
            elif cmd == "status":
                cmd_status(state)
            elif cmd == "wait":
                print("  Waiting for Switch handshake...")
                while not state.setup_done:
                    time.sleep(0.1)
                print("  Setup complete — controller is live!")
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
                cmd_release_all(state)
            elif cmd == "stick":
                if len(parts) < 4:
                    print("  Usage: stick l|r <x> <y>")
                    continue
                cmd_stick(state, parts[1], int(parts[2]), int(parts[3]))
            elif cmd == "center":
                cmd_center(state)
            elif cmd == "spi-set":
                if len(parts) < 3:
                    print("  Usage: spi-set <addr_hex> <data_hex>")
                    continue
                addr = int(parts[1], 16)
                data = bytes.fromhex(parts[2])
                cmd_spi_set(addr, data)
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
            else:
                print(f"  Unknown command: {cmd}. Type 'help' for commands.")
        except Exception as e:
            print(f"  [!] Error: {e}")
            state.errors.append(str(e))


def main():
    parser = argparse.ArgumentParser(
        description="Nintendo Switch Pro Controller emulator via Linux USB Gadget"
    )
    parser.add_argument(
        "--list-udc", action="store_true", help="List USB Device Controllers"
    )
    parser.add_argument("--udc", default="", help="UDC name (auto-detected if omitted)")
    parser.add_argument(
        "--auto", action="store_true", help="Auto setup and wait for Switch handshake"
    )
    parser.add_argument("--record", metavar="FILE", help="Record traffic to file")
    args = parser.parse_args()

    if args.list_udc:
        list_udcs()
        return

    state = GadgetState()

    if args.record:
        state.record_path = args.record
        state.record_file = open(args.record, "a")  # noqa: SIM115
        state.recording = True

    if args.auto:
        if not setup_gadget(state, args.udc):
            sys.exit(1)
        state.running = True
        state.input_thread = threading.Thread(
            target=input_reader_loop, args=(state,), daemon=True
        )
        state.input_thread.start()
        print("\nWaiting for Switch to connect and complete handshake...")
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
            if state.input_thread:
                state.input_thread.join(timeout=2)
            teardown_gadget(state)
        return

    try:
        repl(state)
    finally:
        state.running = False
        if state.input_thread:
            state.input_thread.join(timeout=2)
        if state.recording and state.record_file:
            state.record_file.close()
        teardown_gadget(state)
        print(f"\nSession stats: TX={state.tx_count} RX={state.rx_count}")


if __name__ == "__main__":
    main()
