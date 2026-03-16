# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Switch Controller USB Test** is a set of Python tools for testing and developing the Nintendo Switch Pro Controller USB protocol without the build/flash/plug cycle.

Three tools, three approaches:

| Tool | Approach | Hardware |
|------|----------|----------|
| `switch_probe.py` | Talk to a real Pro Controller over USB HID | Pro Controller + USB-C cable |
| `switch_gadget.py` | PC emulates a Pro Controller via Linux USB gadget | Linux with USB OTG (Raspberry Pi) |
| `switch_proxy.py` | ESP32-S3 handles USB, PC handles protocol via UART | ESP32-S3 + USB-UART adapter |

**Relationship to xbox-switch-bridge:** These tools support development of the Switch Pro Controller USB protocol used by `packages/esp32-projects/xbox-switch-bridge/components/switch_pro_usb/`. The proxy tool pairs with `packages/esp32-projects/switch-usb-proxy/` firmware.

## Commands

```bash
just setup              # Install dependencies (uv sync)
just probe              # Interactive protocol probe REPL
just scan               # List HID devices
just auto               # Auto handshake + dump device info
just gadget             # Start USB gadget emulator (root + OTG required)
just proxy              # Start UART proxy client
just proxy-auto         # Auto-connect proxy
```

Or directly:

```bash
uv sync                                    # Install dependencies
uv run python switch_probe.py --auto       # Probe a real controller
uv run python switch_gadget.py --auto      # Emulate a controller (Linux OTG)
uv run python switch_proxy.py --auto       # Proxy via ESP32-S3 UART
```

## Architecture

All three tools share the same Switch Pro Controller protocol knowledge:

- **SPI flash emulation** — Calibration data, device colors, IMU data at known addresses
- **Subcommand handling** — 0x80 USB commands, 0x01 subcmd requests, 0x10 SPI reads
- **Input report generation** — 0x30 standard input reports at ~125 Hz
- **UART framing** (proxy only) — `[0xAA] [len] [dir] [data...] [checksum]`

### Key Protocol Details

- ACK byte for SPI flash read (subcmd 0x10) is **0x90**, not 0x80
- IMU calibration at 0x6020 (24 bytes): accel origin/sensitivity + gyro origin/sensitivity
- User calibration validity: magic byte 0xB2 at 0x8010 = valid, 0xFF = no user cal
- Device type at 0x6012: 0x03 = Pro Controller

## Dependencies

- `hidapi` ≥0.14.0 — USB HID communication (probe tool)
- `pyserial` ≥3.5 — Serial communication (proxy tool)
- Python ≥3.11

## Related Projects

- `packages/esp32-projects/xbox-switch-bridge/` — Production firmware using this protocol
- `packages/esp32-projects/switch-usb-proxy/` — Thin ESP32-S3 firmware for the proxy tool
- `packages/esp32-projects/xbox-switch-bridge/docs/switch-pro-controller-protocol.md` — Protocol reference
