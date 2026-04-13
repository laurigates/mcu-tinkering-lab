# robocar-unified

Single-board robocar firmware consolidating the dual-ESP32 design (`robocar-main` + `robocar-camera`) onto a **XIAO ESP32-S3 Sense**. Camera capture, AI inference, motor control, and peripherals all run on one module with core-affinity isolation.

## Hardware

| Component | Role | Connection |
|-----------|------|-----------|
| XIAO ESP32-S3 Sense | MCU + camera (OV2640) + 8MB PSRAM | USB-C (native USB-Serial-JTAG) |
| TCA9548A | I2C multiplexer | GPIO5 (SDA), GPIO6 (SCL) |
| PCA9685 | 16-ch PWM driver (LEDs, servos, motor control) | TCA9548A ch0 |
| SSD1306 OLED | 128x64 status display | TCA9548A ch1 |
| TB6612FNG | Dual motor driver | GPIO1 (STBY) + PCA9685 ch8-13 |
| 2x RGB LEDs | Status indicators | PCA9685 ch0-5 |
| 2x SG90 servos | Pan/tilt | PCA9685 ch6-7 |
| Piezo buzzer | Audio feedback | GPIO2 |

See [WIRING.md](WIRING.md) for full connection details.

## Architecture

- **Core 0**: motor control, peripheral I/O, serial commands
- **Core 1**: camera capture, AI analysis, WiFi / MQTT / OTA

AI backend is selected at compile time via sdkconfig (`CONFIG_AI_BACKEND_OLLAMA` by default; Claude and Gemini backends also available).

## Build & flash

This project builds in a container — no local ESP-IDF install required.

```bash
# From repo root
just robocar-unified::build
PORT=/dev/cu.usbmodem* just robocar-unified::flash
just robocar-unified::monitor
# or: just robocar-unified::flash-monitor
```

The XIAO ESP32-S3 uses native USB-Serial-JTAG — no external USB-serial adapter needed. Port auto-detection looks for VID `0x303a`.

## WiFi provisioning

No credentials are compiled in. First boot advertises an Improv WiFi BLE service — use a browser-based provisioner (Chrome on desktop/Android) to send WiFi credentials, which are stored in NVS. For local builds with hardcoded credentials, copy `main/credentials.h.example` to `main/credentials.h`.

After connection, the device is reachable as `robocar.local` via mDNS.

## OTA updates

OTA is enabled (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`) and configured to pull releases from `laurigates/mcu-tinkering-lab` GitHub. `version.txt` is the single source of truth (managed by release-please).

## Key files

- `main/main.c` — FreeRTOS task setup and command dispatch
- `main/pin_config.h` — all GPIO / PCA9685 channel assignments
- `main/ai_backend.c` — AI backend abstraction (Claude / Ollama / Gemini)
- `sdkconfig.defaults` — PSRAM, camera core pinning, OTA, mDNS config
- `partitions.csv` — OTA-capable partition table for 8MB flash
