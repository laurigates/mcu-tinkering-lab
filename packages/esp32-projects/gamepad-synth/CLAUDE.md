# CLAUDE.md — Gamepad Synth

This file provides guidance to Claude Code when working in the `packages/esp32-projects/gamepad-synth/` subproject.

For monorepo-wide conventions, see the root [CLAUDE.md](../../../CLAUDE.md).

## Project Overview

Gamepad Synth turns a BLE Bluetooth controller (Xbox Series X/S, PS5 DualSense, Switch Pro) into a musical instrument. An ESP32-S3 reads gamepad input via Bluepad32 and drives a piezo buzzer through four sound modes: Theremin, Scale Player, Arpeggiator, and Retro SFX.

**Status**: v0.1.0 — LEDC PWM piezo output. I2S DAC upgrade planned (see PRD-008).

## Tech Stack

| Layer | Technology |
|-------|-----------|
| MCU | ESP32-S3 (no PSRAM, no WiFi) |
| Framework | ESP-IDF v5.4+ |
| Bluetooth | Bluepad32 + BTstack (BLE only) |
| Tone generation | LEDC PWM, 8-bit duty, square wave |
| Build | CMake (via ESP-IDF), containerized Docker builds |
| Task runner | justfile (imports `tools/esp32.just`) |

## Build Commands

All builds are containerized — no local ESP-IDF installation needed.

```bash
# From the gamepad-synth directory:
just build          # Build firmware in Docker
just flash          # Flash to device (needs PORT env or auto-detect)
just monitor        # Serial monitor
just develop        # Build + flash + monitor

# Setup (fetch Bluepad32 dependency):
just fetch-deps     # Symlinks from xbox-switch-bridge or clones

# Other:
just clean          # Clean build artifacts
just set-target     # Set chip target (esp32s3)
just menuconfig     # Open ESP-IDF menuconfig
just shell          # Interactive ESP-IDF container shell
just info           # Show project info
```

Port is auto-detected for ESP32-S3. Override with `PORT=/dev/ttyUSB0 just flash`.

## Architecture

### Dual-Core Design

- **Core 0**: Bluepad32 BTstack event loop — handles all Bluetooth communication. Blocks forever in `uni_esp32_enable()`.
- **Core 1**: Sound engine FreeRTOS task at 50 Hz (`SOUND_TASK_HZ`). Reads latest gamepad state and updates LEDC PWM tone.

### Key Constants

| Define | Value | Purpose |
|--------|-------|---------|
| `PIEZO_PIN` | GPIO4 | LEDC PWM output to passive piezo buzzer |
| `LED_PIN` | GPIO2 | Status LED (on during tone, blinks for mode switch) |
| `SOUND_TASK_HZ` | 50 | Sound engine update rate |
| `STICK_DEADZONE` | 50 | Analog stick dead zone (out of ±512) |
| `MIN_FREQ` / `MAX_FREQ` | 100 / 2000 Hz | Tone frequency range |

### Sound Modes (cycled via View/Share button)

1. **Theremin** — Left stick Y = pitch, left stick X = vibrato depth, right stick Y = vibrato speed, triggers = pitch bend
2. **Scale Player** — Face buttons + d-pad = C major scale notes, shoulders = octave shift (C4-C6)
3. **Arpeggiator** — Face buttons = chord type, RT = toggle, left stick Y = speed, left stick X = pattern
4. **Retro SFX** — Face buttons + d-pad = game sound effects, RT = speed multiplier

### Dependencies

- **Bluepad32**: External component in `external/bluepad32/`. Shared with xbox-switch-bridge via symlink. CI clones it directly.
- **ESP-IDF components**: `driver`, `nvs_flash`, `esp_timer`, `bluepad32` (private require)

## File Structure

```
gamepad-synth/
├── main/
│   ├── main.c              # All firmware code (single file)
│   └── CMakeLists.txt       # Component registration
├── CMakeLists.txt           # Project-level CMake (sets EXTRA_COMPONENT_DIRS for Bluepad32)
├── sdkconfig.defaults       # ESP-IDF config (BT controller-only, no WiFi, no PSRAM)
├── justfile                 # Build/flash/monitor recipes
├── dependencies.lock        # ESP-IDF component lock
├── version.txt              # Semantic version (used by CMake and release-please)
├── README.md                # User-facing documentation
└── WIRING.md                # Hardware wiring guide
```

## Code Conventions

- **Single-file firmware**: All code lives in `main/main.c`. Keep it that way until the I2S refactor (PRD-008 Phase A).
- **C style**: Google style, 4-space indent, 100-char lines (enforced by `.clang-format` at repo root).
- **ESP-IDF patterns**: Use `ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE` for logging with `TAG`. Use `ESP_ERROR_CHECK` for IDF API calls.
- **Bluepad32 callbacks**: Register via `uni_platform` struct. Callbacks run on Core 0 — keep them fast, copy data to shared state for Core 1.
- **Shared state**: Gamepad state is copied in the Bluepad32 callback and read by the sound task. Currently not mutex-protected (atomic-width fields on single reader/writer).

## sdkconfig Highlights

- `CONFIG_BT_CONTROLLER_ONLY=y` — Bluepad32 uses BTstack, not NimBLE or Bluedroid
- `CONFIG_ESP_WIFI_ENABLED=n` — No WiFi needed, saves RAM
- `CONFIG_SPIRAM=n` — No PSRAM
- `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` — Logging over USB-C
- `CONFIG_FREERTOS_HZ=1000` — 1 ms tick resolution

## Related Documents

- [PRD-008: Gamepad Synth](../../../docs/blueprint/prds/PRD-008-gamepad-synth.md) — I2S DAC upgrade roadmap
- [ADR-011: Gamepad Synth I2S Audio](../../../docs/blueprint/adrs/ADR-011-gamepad-synth-i2s-audio.md) — Architecture decision for I2S
- [WIRING.md](WIRING.md) — Hardware wiring guide
