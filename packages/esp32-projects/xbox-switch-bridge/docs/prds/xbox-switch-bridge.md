# Xbox-Switch Bridge — Product Requirements Document

**Status**: Feature Complete (v1.0)
**Last Updated**: 2026-03-12
**Confidence**: 9/10 (derived from git history and source code analysis)

## Overview

Xbox-Switch Bridge is an ESP32-S3 firmware that bridges Xbox Series wireless controllers (BLE) to Nintendo Switch consoles (USB), emulating a wired Pro Controller. The device scans for Xbox controllers over Bluetooth Low Energy using Bluepad32, maps inputs to the Switch Pro Controller format, and presents itself as a wired Pro Controller over USB HID via TinyUSB.

## Target Users

- Gamers wanting to use Xbox Series controllers on Nintendo Switch
- Hardware hobbyists building custom controller bridges
- Developers exploring BLE-to-USB protocol translation

## Architecture

```
Xbox Series Controller  --BLE-->  ESP32-S3-Zero  --USB HID-->  Nintendo Switch Dock
   (Bluepad32 v3.x)              (125 Hz bridge)              (Pro Controller VID/PID)
```

**Dual-core assignment:**
- Core 0: BTstack/Bluepad32 BLE event loop (does not return)
- Core 1: 125 Hz bridge loop (input mapping + USB HID reports + LED updates)

**Hardware**: Waveshare ESP32-S3-Zero (ESP32-S3, 4MB flash, 2MB PSRAM, GPIO21 WS2812)

## Features — Implemented

| FR Code | Requirement | Status | Implemented In |
|---------|-------------|--------|----------------|
| FR1 | BLE Gamepad Host — scan/connect to Xbox Series controllers via Bluepad32 v3.x custom platform | Completed | `components/bluepad32_host/` |
| FR2 | Button Mapping — Xbox A/B/X/Y position-based swap to Switch B/A/Y/X layout | Completed | `components/button_mapper/` |
| FR3 | Analog Stick Remapping — Bluepad32 signed range (-511..512) to Switch 12-bit unsigned (0-4095) with Y-inversion | Completed | `components/button_mapper/` |
| FR4 | Trigger Handling — digital button mapping + analog threshold (>128) for ZL/ZR | Completed | `components/button_mapper/` |
| FR5 | USB Pro Controller Emulation — VID 0x057E / PID 0x2009 with TinyUSB HID | Completed | `components/switch_pro_usb/` |
| FR6 | Switch USB Handshake Protocol — 0x80 commands (STATUS/HANDSHAKE/HIGH_SPEED/FORCE_USB) + 0x01 sub-commands (device info, SPI reads, LED config) | Completed | `components/switch_pro_usb/` |
| FR7 | Status LED — 5 WS2812 states: SCANNING (blue blink), USB_ERROR (red solid), CONNECTED_NO_USB (purple blink), CONNECTED_USB (yellow blink), BRIDGING (green solid) | Completed | `components/status_led/` |
| FR8 | WiFi UDP Log Broadcasting — SoftAP "xbox-bridge-log" (open), UDP broadcast on port 4444 | Completed | `components/log_udp/` |
| FR9 | Build Variants — 5 sdkconfig overlays: production, debug-JTAG, debug-UART, debug-USB, wifi-test | Completed | `sdkconfig.defaults`, `sdkconfig.debug-jtag`, `sdkconfig.debug-uart`, `sdkconfig.debug-usb`, `sdkconfig.wifi-test` |

## Features — Planned

| FR Code | Requirement | Status | Notes |
|---------|-------------|--------|-------|
| FR10 | Rumble Output — HID output report handler for vibration feedback | Not Started | HID descriptor already includes 0x10 rumble report; handler stub exists |
| FR11 | Runtime Button Remapping — configurable mapping via credentials.h or NVS | Not Started | Currently hardcoded in `button_mapper.c` |
| FR12 | Multi-Controller Support — connect and bridge multiple Xbox controllers | Not Started | Bluepad32 supports multiple controllers architecturally |

## Non-Functional Requirements

| Requirement | Specification |
|-------------|---------------|
| Input latency | ≤8ms (one bridge loop cycle at 125 Hz) |
| USB poll rate | 125 Hz (8ms interval, matching Switch expectations) |
| Controller compatibility | Xbox Series X/S wireless controllers (BLE) |
| Power | USB bus-powered via Switch dock |
| Flash usage | 4MB (custom partition table) |
| PSRAM | 2MB (quad-SPI at 80 MHz) |

## Hardware Constraints

See [ADR-0001](../adrs/0001-esp32s3-target-hardware.md) for target hardware rationale.

- **USB PHY sharing**: USB-Serial-JTAG and USB-OTG share one PHY on ESP32-S3. TinyUSB claims the PHY exclusively in production builds.
- **RMT + TinyUSB DMA conflict**: Status LED must use `with_dma = false` + `mem_block_symbols = 48`.
- **BLE + SoftAP coexistence**: Rated "C1: unstable" by Espressif. WiFi pinned to core 1 to reduce contention.

## Dependencies

- ESP-IDF v5.4+ (via Docker container `espressif/idf:v5.4`)
- Bluepad32 v3.10.3 + btstack (cloned into `external/`)
- `espressif/esp_tinyusb` managed component
- `espressif/led_strip` ≥2.0.0 managed component

## References

- [Nintendo Switch Reverse Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering) — Pro Controller protocol documentation
- [Bluepad32 documentation](https://github.com/ricardoquesada/bluepad32) — BLE gamepad library
