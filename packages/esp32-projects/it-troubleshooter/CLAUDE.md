# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**IT Troubleshooter** is a phased ESP32-S3 firmware project that emulates a USB composite device (HID keyboard + CDC serial). The goal is network troubleshooting automation via USB command injection and bidirectional communication.

**Current phase:** Phase 1 PoC — HID keyboard + CDC-ACM serial.
**Planned:** WiFi connectivity, Claude API integration, CDC-NCM (USB Ethernet).

**Target hardware:** Waveshare ESP32-S3-Zero (ESP32-S3, 4MB flash, 2MB PSRAM, WS2812 RGB on GPIO21).

## Build Commands

Builds run in Docker (`espressif/idf:v5.4`); flash/monitor run natively (USB passthrough on macOS is unreliable).

```bash
just build        # Compile in Docker
just flash        # Flash to /dev/cu.usbmodem101 (USB-Serial-JTAG)
just monitor      # Serial via minicom on GPIO43/44 USB-UART adapter @ 115200
just deploy       # build + flash
just menuconfig   # Interactive sdkconfig in Docker
just clean        # Remove build/, sdkconfig, managed_components (prompts confirm)
```

Override USB port: `just flash PORT=/dev/ttyUSB0`

**Credentials setup:**
```bash
cp main/credentials.h.example main/credentials.h
# Edit with WiFi SSID, password, Claude API key (gitignored)
```

## Architecture

### Initialization sequence (`main/main.c`)

1. NVS flash init
2. `status_led_init()` → set BOOT mode
3. `usb_composite_init()` → TinyUSB HID keyboard + CDC
4. Poll `usb_composite_is_mounted()` (30s timeout)
5. Type demo string via HID keyboard
6. Spawn CDC echo task on core 1

**Core assignment:** Core 0 — USB stack (TinyUSB interrupts). Core 1 — application tasks (CDC echo, future work).

### Component: `usb_composite`

TinyUSB composite device with two functional interfaces:

- **HID Boot Keyboard** (interface 0, endpoint 0x81): 8-byte boot protocol report, BIOS-compatible
- **CDC-ACM** (interfaces 1–2, endpoints 0x82/0x03/0x83): standard serial over USB

Key API: `usb_keyboard_type_string()`, `usb_keyboard_press_with_modifier()`, `usb_cdc_write()`, `usb_cdc_read()`.

ASCII → HID keycode mapping covers full US QWERTY including shifted characters. Timing: 20ms key hold, 10ms between chars.

### Component: `status_led`

WS2812 RMT driver (no background task — caller must invoke `status_led_update()` periodically at ≥50Hz).

| Mode | Color/Pattern | Meaning |
|------|--------------|---------|
| `BOOT` | Blue blink 250ms | Initializing |
| `USB_READY` | Cyan solid | USB mounted |
| `BIOS_MODE` | Yellow blink 250ms | Injecting keystrokes |
| `WIFI_CONNECTING` | Blue pulse 1000ms | WiFi connecting |
| `DIAGNOSTIC` | Green pulse 1000ms | Running diagnostics |
| `ERROR` | Red solid | Failure |
| `COMPLETE` | Green solid | Success |

## Hardware Constraints

**USB PHY sharing (critical):** USB-Serial-JTAG and USB-OTG share a single internal PHY on ESP32-S3. Once `tinyusb_driver_install()` is called, JTAG disconnects entirely — `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` does **not** redirect ESP_LOG.

**Only reliable monitoring:** USB-UART adapter on GPIO43 (TX) / GPIO44 (RX) at 115200 baud.

**RMT + TinyUSB DMA conflict:** `with_dma = true` on the RMT channel breaks after TinyUSB init. Status LED uses `with_dma = false` + `mem_block_symbols = 48`.

**TinyUSB `tinyusb_config_t` (esp_tinyusb v2.x):** The `.task` field (`size`, `priority`, `xCoreID`) must be non-zero or `tinyusb_driver_install()` returns `ESP_ERR_INVALID_ARG`.

## Dependencies

- `espressif/esp_tinyusb` ^1.0.0 — USB device stack
- `espressif/led_strip` ≥2.0.0 — WS2812 RMT driver
- ESP-IDF core: `nvs_flash`, `driver`, `esp_timer`, `freertos`
