# CLAUDE.md — Gamepad Synth

This file provides guidance to Claude Code when working in the `packages/audio/gamepad-synth/` subproject.

For monorepo-wide conventions, see the root [CLAUDE.md](../../../CLAUDE.md).

## Project Overview

Gamepad Synth turns a BLE Bluetooth controller (Xbox Series X/S, PS5 DualSense, Switch Pro) into a Korg Monotron-inspired synthesizer. An ESP32-S3 reads gamepad input via Bluepad32 and produces audio through an I2S DAC (MAX98357A) in seven sound modes.

**Status**: v1.0.0 — All PRD-008 phases complete. Mono Synth, Dual Osc, Delay Synth, Scale, Arpeggio, Retro SFX, Drone modes. Full signal chain: dual DDS oscillators → resonant SVF filter → LFO modulation → 0.5 s delay → I2S DAC.

## Tech Stack

| Layer | Technology |
|-------|-----------|
| MCU | ESP32-S3 (no PSRAM, no WiFi) |
| Framework | ESP-IDF v5.4+ |
| Bluetooth | Bluepad32 + BTstack (BLE only) |
| Audio output | I2S DAC (MAX98357A), 44.1 kHz, 16-bit, multi-waveform DDS |
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
- **Core 1**: Two tasks:
  - **Control task** (50 Hz, priority 5): reads gamepad state, updates `synth_state_t` (target frequency, waveform, active flag)
  - **Audio render task** (continuous, priority 10): DDS phase accumulator with selectable waveform (square/saw/tri/sine/noise) generates 256-sample blocks, writes to I2S DMA

### Key Constants

| Define | Value | Purpose |
|--------|-------|---------|
| `I2S_BCLK_PIN` | GPIO5 | I2S bit clock to MAX98357A |
| `I2S_WS_PIN` | GPIO6 | I2S word select (LRCLK) |
| `I2S_DOUT_PIN` | GPIO7 | I2S serial data out |
| `LED_PIN` | GPIO2 | Status LED (on during tone, blinks for mode switch) |
| `SAMPLE_RATE` | 44100 | Audio sample rate (Hz) |
| `BLOCK_SIZE` | 256 | Samples per render block (~5.8 ms) |
| `CONTROL_TASK_HZ` | 50 | Gamepad polling rate |
| `STICK_DEADZONE` | 50 | Analog stick dead zone (out of ±512) |
| `MIN_FREQ` / `MAX_FREQ` | 100 / 2000 Hz | Tone frequency range |

### Control Paradigm

Tweak sticks (RY, RX, plus LX/LY in Drone) use **rate control**: stick displacement = rate of change of the parameter, not absolute position. Holding off-center changes the value over time; releasing to center holds the last value. This makes it possible to find a sweet spot and release, rather than having to hold the stick still. A short "bump" blip fires when a parameter reaches its min/max limit.

Primary pitch sticks (LY in Mono/Dual/Delay) stay **absolute** for theremin-style muscle memory — stick up = high note.

Tweak parameter state (cutoff, resonance, delay time, feedback, detune, pitch offset) persists across ticks within a mode, resets on mode switch, and resets on LS-click.

### Global Buttons (every mode)

| Button | Action |
|---|---|
| D-pad ↑/↓ | Master volume nudge (±0.05 per step), auto-repeats at 4 Hz |
| D-pad ←/→ | Drum tempo nudge (±5 BPM), also drives the arp step rate |
| Share/View/− (`MISC_BACK`) | Cycle sound mode (1-7). Brief LED flash, then a per-mode signature gesture plays. |
| Home/PS/Xbox tap | Toggle drum engine on/off. Pattern/volume come from the settings page. |
| Home/PS/Xbox held + A/B/X/Y | Select drum pattern 1/2/3/4 directly (auto-starts drums if off) |
| Menu/Options/+ | Enter/exit settings-edit overlay. Inside, d-pad becomes field navigation (not volume/tempo). |
| LS click (left-stick press) | Reset current mode's tweak parameters to defaults |
| LT / RT triggers | ±7-semitone pitch bend (global in pitched modes) |

**Settings-edit overlay**: d-pad ←/→ moves cursor between fields (drum_pattern, drum_volume, lfo_rate_hz, lfo_depth, lfo_target); ↑/↓ adjusts the selected field. Each cursor move plays a short value ladder so the user can audit the current value without a screen. Master volume and tempo moved to the global d-pad.

### Sound Modes (cycled via View/Share/- button; each plays a signature gesture on entry)

1. **Mono Synth** — Monotron-style single osc. LY=pitch (absolute), LX=vibrato depth (absolute, fixed 5 Hz rate), RY=filter cutoff (integrating, log), RX=resonance (integrating). LFO target/rate/depth from settings.
2. **Dual Osc** — Two sawtooth oscillators. LY=pitch (absolute), face buttons pick interval (A=unison, B=fifth, X=octave, Y=2 octaves), RY=cutoff (integrating), RX=resonance (integrating). Detune persists in `s_tweak_detune_cents` (phase-2 entry point for a shoulder modifier).
3. **Delay Synth** — Single osc with dynamic delay. LY=pitch (absolute), RY=delay time (integrating, 20-500 ms, log), RX=feedback (integrating, 0-0.9). Filter fixed at 4 kHz / Q 1.2.
4. **Scale Player** (sine + slapback) — A/B/X/Y (no LB) = Do/Re/Mi/Fa; **LB held** + A/B/X/Y = Sol/La/Ti/Do-high. LY integrates pitch offset (±12 st); RY = fine bend (±50 Hz, absolute).
5. **Arpeggiator** (square + cosmic echo) — Face buttons = chord (A=major, B=minor, X=7th, Y=dim), LB/RB = octave, LY integrates root transpose (±12 st), LX-zones = pattern (left=down, right=up, center=up-down), RT toggles arp. Step rate is derived from the global tempo (16th notes).
6. **Retro SFX** (filter/delay bypassed) — A/B/X/Y (no LB) = Laser/Explosion/Power-up/Coin; **LB held** + A/B/X/Y = Siren/Engine/Jump/Warp. RT = speed multiplier (0.3-1.0x on tick count).
7. **Drone** — Two sustained oscillators. LY integrates osc A pitch, RY integrates osc B pitch (both drift slowly), LX integrates filter cutoff, RX integrates resonance. Osc B is also routed to the two piezos at a fixed detune ratio for acoustic beating. LFO from settings.

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

- **Single-file firmware**: All code lives in `main/main.c`. Will split into modules in Phase B (oscillators) or Phase C (filter/LFO).
- **C style**: Google style, 4-space indent, 100-char lines (enforced by `.clang-format` at repo root).
- **ESP-IDF patterns**: Use `ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE` for logging with `TAG`. Use `ESP_ERROR_CHECK` for IDF API calls.
- **Bluepad32 callbacks**: Register via `uni_platform` struct. Callbacks run on Core 0 — keep them fast, copy data to shared state for Core 1.
- **Shared state**: Gamepad state is copied in the Bluepad32 callback and read by the control task. Synth state (`synth_state_t`) is written by the control task and read by the audio render task. Both on Core 1 with the audio task at higher priority — no mutex needed.

## sdkconfig Highlights

- `CONFIG_BT_CONTROLLER_ONLY=y` — Bluepad32 uses BTstack, not NimBLE or Bluedroid
- `CONFIG_ESP_WIFI_ENABLED=n` — No WiFi needed, saves RAM
- `CONFIG_SPIRAM=n` — No PSRAM
- `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` — Logging over USB-C
- `CONFIG_FREERTOS_HZ=1000` — 1 ms tick resolution

## Related Documents

- [PRD-008: Gamepad Synth](../../../docs/requirements/PRD-008-gamepad-synth.md) — I2S DAC upgrade roadmap
- [ADR-011: Gamepad Synth I2S Audio](../../../docs/decisions/ADR-011-gamepad-synth-i2s-audio.md) — Architecture decision for I2S
- [WIRING.md](WIRING.md) — Hardware wiring guide
