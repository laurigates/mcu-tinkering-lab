# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Switch USB Proxy** is a thin ESP32-S3 firmware that acts as a USB HID ↔ UART bridge. It presents as a Nintendo Switch Pro Controller (VID 0x057E / PID 0x2009) over USB, and forwards all HID traffic to/from a PC over UART. The PC runs Python protocol logic (`tools/switch-controller-usb-test/switch_proxy.py`), enabling rapid protocol iteration without reflashing.

**Target hardware:** Waveshare ESP32-S3-Zero (same as xbox-switch-bridge).

**Relationship to xbox-switch-bridge:** This is a companion tool. The xbox-switch-bridge handles the full BLE→USB bridge on-device; this project offloads protocol logic to the PC for faster development and debugging of the Switch Pro Controller USB protocol.

## Build Commands

Builds run in Docker (`espressif/idf:v5.4`); flash runs natively.

```bash
just build              # Build firmware (containerized)
just flash              # Flash to ESP32-S3 (auto-detects port)
just monitor            # Serial monitor via USB-Serial-JTAG
just flash-monitor      # Flash then monitor
just clean              # Remove build artifacts
just info               # Show project and port info
```

Override USB port: `just flash PORT=/dev/cu.usbmodem1101`

## Architecture

Single-file firmware (`main/main.c`) with two tasks:

- **Main task**: USB init, UART init, monitors USB mount status, notifies PC
- **Bridge task**: Reads UART frames from PC, sends as USB HID reports to Switch

### Data Flow

```
Switch ──USB HID──→ ESP32-S3 ──UART──→ PC (switch_proxy.py)
Switch ←─USB HID──  ESP32-S3 ←─UART──  PC (switch_proxy.py)
```

### UART Frame Format

```
[0xAA] [len] [direction] [report_data...] [checksum]

direction: 'S' (0x53) = Switch → PC
           'P' (0x50) = PC → Switch
checksum:  XOR of all payload bytes
```

### Hardware Connections

| Pin | Function | Notes |
|-----|----------|-------|
| GPIO19/20 | USB D-/D+ | To Switch dock via USB-C |
| GPIO43 | UART TX | To PC via USB-UART adapter (CP2102/CH340) |
| GPIO44 | UART RX | From PC via USB-UART adapter |

UART baud: 921600 (fast enough for 125 Hz × 64 bytes).

## Hardware Constraints

Same USB PHY sharing constraint as xbox-switch-bridge: once TinyUSB owns the PHY, USB-Serial-JTAG disconnects. Monitor via the UART adapter, not USB-C.

## Dependencies

- `espressif/esp_tinyusb` ≥1.4.0 — USB device stack (managed component)
- No Bluetooth, no WiFi — minimal firmware

## Related Projects

- `packages/esp32-projects/xbox-switch-bridge/` — Full BLE→USB bridge (production firmware)
- `tools/switch-controller-usb-test/` — PC-side Python tools (switch_proxy.py, switch_gadget.py, switch_probe.py)
- `packages/esp32-projects/xbox-switch-bridge/docs/switch-pro-controller-protocol.md` — Protocol reference
