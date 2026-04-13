# CLAUDE.md - robocar-unified

Project-specific guidance for Claude Code. See repo-root `CLAUDE.md` for monorepo-wide conventions.

## What this project is

Single-board consolidation of the dual-ESP32 robocar onto a **XIAO ESP32-S3 Sense**. Replaces the separate `robocar-main` (Heltec) + `robocar-camera` (ESP32-CAM) boards — camera, AI, motors, peripherals, WiFi, MQTT, and OTA all run on one module.

## Architecture

Core affinity is load-bearing — do not change without understanding the trade-offs:

- **Core 0** (motor-critical, timing-sensitive): `motor_task`, `peripheral_task`, `command_task`
- **Core 1** (bursty, I/O-bound): `camera_task`, `ai_task`, `network_task`, OTA
- Camera DMA is pinned to Core 1 via `CONFIG_CAMERA_CORE1=y` so motor PWM isn't jittered by frame captures

Tasks communicate via FreeRTOS queues (see `motor_cmd_t`, `peripheral_cmd_t` in `main.c`).

## I2C topology

All I2C devices hang off a **TCA9548A multiplexer** on GPIO5/6. Don't talk to devices directly on the primary bus — always select the channel first via `i2c_bus.c`. Current channel map in `pin_config.h`:

- ch0: PCA9685 (motors, servos, LEDs)
- ch1: SSD1306 OLED
- ch2-7: reserved

## PCA9685 channel layout

All motor direction, motor PWM, servo, and LED outputs go through the PCA9685 — the ESP32-S3 only drives STBY (GPIO1), the buzzer (GPIO2), and I2C. GPIO budget on XIAO headers is tight (11 pins). See the channel table in `main/pin_config.h` before reassigning.

Motor direction uses PCA9685 "full-on" (4096) / "full-off" (0) values on IN1/IN2 channels.

## AI backends

Selected at **compile time** via sdkconfig:

- `CONFIG_AI_BACKEND_OLLAMA` (default) — self-hosted, discovered via mDNS
- `CONFIG_AI_BACKEND_CLAUDE` — Anthropic API
- `CONFIG_AI_BACKEND_GEMINI` — Google Gemini Robotics-ER

`ai_backend.c` provides the abstraction; each backend has its own `.c/.h` pair. `ai_response_parser.c` extracts motor commands from the AI response.

## WiFi provisioning

Uses **Improv WiFi** BLE provisioning by default — no credentials compiled in. `CMakeLists.txt` auto-generates a stub `credentials.h` at configure time if one doesn't exist, so CI and the web flasher don't need credentials. For local dev with hardcoded creds, copy `main/credentials.h.example` to `main/credentials.h` (gitignored).

NVS stores runtime-provisioned credentials — don't wipe NVS unless you want to re-provision.

## OTA

`ota_manager.c` pulls releases from the `laurigates/mcu-tinkering-lab` GitHub repo. `version.txt` is the single source of truth for the running version (read at CMake-configure time into `PROJECT_VER`). Release-please manages version bumps — do not edit `version.txt` manually.

Partition layout is OTA-capable (see `partitions.csv`) with app rollback enabled.

## Build & flash

Containerized ESP-IDF v5.4 via `just robocar-unified::*`. XIAO uses native USB-Serial-JTAG — `just` auto-detects VID `0x303a`. If flashing fails, hold BOOT and tap RESET to enter download mode.

```bash
just robocar-unified::build
just robocar-unified::flash-monitor
```

Do not invoke `idf.py` directly on the host — there's no local ESP-IDF install. Use `just robocar-unified::shell` for an interactive container.

## sdkconfig rules

See repo `.claude/rules/esp-idf-sdkconfig.md`. If you change `sdkconfig.defaults`, delete the generated `sdkconfig` and run `just robocar-unified::clean` before rebuilding — ESP-IDF preserves existing `sdkconfig` values and silently ignores new defaults otherwise.

Key settings that matter:
- `CONFIG_SPIRAM_MODE_OCT=y` — XIAO ESP32-S3 Sense has **octal** PSRAM (not quad); wrong mode = boot loop
- `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` — bumped from default 3584 for WiFi + BLE + camera init
- `CONFIG_ESP_BROWNOUT_DET=n` — disabled; motor inrush was tripping it
- `CONFIG_MDNS_ENABLED=y` — required for Ollama discovery and `robocar.local`

## Don't

- Don't add direct GPIO motor control — everything goes through PCA9685 via `motor_controller.c`
- Don't bypass the TCA9548A — devices on different channels can share addresses (e.g. PCA9685 and OLED would conflict without it)
- Don't commit `main/credentials.h` — it's gitignored; use the `.example` as template
- Don't hand-edit `version.txt` — release-please owns it
- Don't change camera core pinning without re-verifying motor PWM jitter
