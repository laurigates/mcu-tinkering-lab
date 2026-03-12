# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Xbox-Switch Bridge** is an ESP32-S3 firmware that bridges Xbox Series wireless controllers (BLE) to Nintendo Switch consoles (USB), emulating a wired Pro Controller.

**Current status**: Feature complete (v1.0) — all 9 core features implemented.
**Planned**: Rumble output (FR10), runtime button remapping (FR11), multi-controller support (FR12).

**Target hardware:** Waveshare ESP32-S3-Zero (ESP32-S3, 4MB flash, 2MB PSRAM, WS2812 RGB on GPIO21).

## Build Commands

Builds run in Docker (`espressif/idf:v5.4`); flash/monitor run natively (USB passthrough on macOS is unreliable).

```bash
just fetch-deps   # Clone bluepad32 + btstack (one-time setup)
just build        # Production build (containerized)
just build-debug  # TinyUSB disabled, USB-Serial-JTAG logging active
just build-debug-usb  # TinyUSB ON, verbose logging over WiFi UDP
just build-wifi-test  # No BLE, isolates SoftAP visibility
just flash        # Flash to device (auto-detects ESP32-S3 port)
just monitor      # Serial monitor via USB-Serial-JTAG (debug builds only)
just log-listen   # Listen for UDP log broadcast (connect to xbox-bridge-log WiFi first)
just menuconfig   # Interactive sdkconfig in Docker
just clean        # Remove build artifacts
just info         # Show project and port info
```

Override USB port: `just flash PORT=/dev/cu.usbmodem101`

**Credentials setup:**
```bash
cp main/credentials.h.example main/credentials.h
# Edit with WiFi credentials for future STA-mode features (gitignored)
```

## Architecture

### Dual-Core Assignment

- **Core 0**: BTstack/Bluepad32 BLE event loop (`bp32_host_start()` — does not return)
- **Core 1**: 125 Hz bridge task (input mapping + USB HID reports + LED updates)

### Initialization Sequence (`main/main.c`)

1. USB-Serial-JTAG init (debug builds only, when TinyUSB is disabled)
2. `status_led_init()` → WS2812 RMT driver
3. `init_nvs()` → NVS flash (required for BT stack)
4. `log_udp_init(4444)` → WiFi SoftAP + UDP broadcast
5. `switch_pro_usb_init()` → TinyUSB HID (graceful failure if not connected to dock)
6. `bp32_host_init()` → register connection callback
7. `xTaskCreatePinnedToCore(bridge_task, ...)` → spawn bridge on core 1
8. `bp32_host_start()` → start BTstack event loop on core 0 (blocks forever)

### Components

| Component | Purpose |
|-----------|---------|
| `bluepad32_host` | Bluepad32 v3.x custom platform — BLE gamepad scan/connect, shared state |
| `button_mapper` | Xbox → Switch input mapping (face button swap, stick remapping, triggers) |
| `switch_pro_usb` | TinyUSB HID (VID 0x057E / PID 0x2009), 0x80 handshake, 0x30 input reports |
| `status_led` | WS2812 RMT driver — caller-driven via `status_led_update()` at 125 Hz |
| `log_udp` | WiFi SoftAP "xbox-bridge-log" + UDP broadcast log sink on port 4444 |

### Bridge State Machine

```
INIT → SCANNING → CONNECTED → BRIDGING
                      ↑            ↓
                      └────────────┘ (on disconnect)
```

### Status LED Modes

| Mode | Color/Pattern | Meaning |
|------|--------------|---------|
| `SCANNING` | Blue blink (1 Hz) | Waiting for Xbox controller |
| `USB_ERROR` | Red solid | TinyUSB init failed |
| `CONNECTED_NO_USB` | Purple blink (1 Hz) | Xbox paired, USB not enumerated |
| `CONNECTED_USB` | Yellow blink (2.5 Hz) | USB enumerated, handshake pending |
| `BRIDGING` | Green solid | Actively forwarding inputs |

## Hardware Constraints

**USB PHY sharing (critical):** USB-Serial-JTAG and USB-OTG share a single internal PHY on ESP32-S3. Once `tinyusb_driver_install()` is called, JTAG disconnects entirely. Use build variants to control which owns the PHY.

**Only reliable monitoring (production builds):** WiFi UDP logging via `just log-listen`. Connect to "xbox-bridge-log" WiFi first.

**RMT + TinyUSB DMA conflict:** `with_dma = true` on the RMT channel breaks after TinyUSB init. Status LED uses `with_dma = false` + `mem_block_symbols = 48`.

**BLE + SoftAP coexistence:** Rated "C1: unstable" by Espressif. WiFi pinned to core 1 to reduce contention with BTstack on core 0.

**TinyUSB init can legitimately fail** when the ESP32-S3 is connected to a computer USB port (not a Switch dock). Firmware handles this gracefully — logs a warning and continues; BLE scanning still works.

## Build Variants

| Variant | Command | TinyUSB | BLE | USB-JTAG | WiFi |
|---------|---------|---------|-----|----------|------|
| Production | `just build` | ON | ON | OFF | ON |
| Debug-UART | `just build-debug` | OFF | ON | ON | ON |
| Debug-USB | `just build-debug-usb` | ON | ON | OFF | ON (verbose) |
| WiFi-Test | `just build-wifi-test` | OFF | OFF | ON | ON |

## Dependencies

- **Bluepad32 v3.10.3** + btstack — cloned into `external/` by `just fetch-deps`
- `espressif/esp_tinyusb` — USB device stack (managed component)
- `espressif/led_strip` ≥2.0.0 — WS2812 RMT driver (managed component)
- ESP-IDF core: `nvs_flash`, `driver`, `esp_timer`, `esp_wifi`, `freertos`

## Documentation

See `docs/` for detailed project documentation:

- `docs/prds/xbox-switch-bridge.md` — Product Requirements Document
- `docs/adrs/` — Architecture Decision Records (8 ADRs)
- `docs/prps/` — Product Requirement Prompts (rumble output, button remapping)
- `docs/blueprint/` — Feature tracker and manifest
