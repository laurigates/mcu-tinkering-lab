# robocar-unified

Single-board robocar firmware consolidating the dual-ESP32 design (`robocar-main` + `robocar-camera`) onto a **XIAO ESP32-S3 Sense**. Camera capture, AI inference, motor control, and peripherals all run on one module with core-affinity isolation.

## Hardware

| Component | Role | Connection |
|-----------|------|-----------|
| XIAO ESP32-S3 Sense | MCU + camera (OV2640) + 8MB PSRAM | USB-C (native USB-Serial-JTAG) |
| Ultrasonic rangefinder | Distance reflex (obstacle avoidance) | GPIO3 (TRIG), GPIO4 (ECHO) |
| TCA9548A | I2C multiplexer | GPIO5 (SDA), GPIO6 (SCL) |
| PCA9685 | 16-ch PWM driver (LEDs, servos, motor control) | TCA9548A ch0 |
| SSD1306 OLED | 128x64 status display | TCA9548A ch1 |
| TB6612FNG | Dual motor driver | GPIO1 (STBY) + PCA9685 ch8-13 |
| 2x RGB LEDs | Status indicators | PCA9685 ch0-5 |
| 2x SG90 servos | Pan/tilt | PCA9685 ch6-7 |
| Piezo buzzer | Audio feedback | GPIO2 |

See [WIRING.md](WIRING.md) for full connection details.

## Architecture

Implements a hierarchical AI controller: a **slow planner** (~1 Hz, Core 1) that calls Gemini Robotics-ER to emit structured goals, and a **fast reactive executor** (~30 Hz, Core 0) that drives the robot smoothly toward those goals. See [ADR-016](../../docs/blueprint/adrs/ADR-016-hierarchical-ai-controller.md) for the detailed design.

- **Core 0**: reactive executor (visual servo, heading hold, motor PWM), motor control, peripheral I/O, obstacle reflex via ultrasonic sensor
- **Core 1**: planner task (Gemini calls), camera capture, WiFi / MQTT / OTA

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

- `main/main.c` — FreeRTOS task setup and core affinity
- `main/pin_config.h` — all GPIO / PCA9685 channel assignments
- `main/planner_task.c/.h` — Gemini Robotics-ER calls, goal state writes (~1 Hz)
- `main/reactive_controller.c/.h` — visual servo, heading hold, motor output (~30 Hz)
- `main/ultrasonic.c/.h` — distance measurement and reflex (~20 Hz sampling)
- `main/goal_state.c/.h` — shared planner-executor state (mutex-protected)
- `main/motor_controller.c/.h` — low-level motor PWM (called only by executor)
- `sdkconfig.defaults` — PSRAM, camera core pinning, OTA, mDNS config
- `partitions.csv` — OTA-capable partition table for 8MB flash
