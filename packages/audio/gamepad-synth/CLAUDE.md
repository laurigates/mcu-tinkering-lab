# CLAUDE.md — Gamepad Synth

This file provides guidance to Claude Code when working in the `packages/audio/gamepad-synth/` subproject.

For monorepo-wide conventions, see the root [CLAUDE.md](../../../CLAUDE.md).

## Project Overview

Gamepad Synth turns a BLE Bluetooth controller (Xbox Series X/S, PS5 DualSense, Switch Pro) into a Korg Monotron-inspired synthesizer. An ESP32-S3 reads gamepad input via Bluepad32 and produces audio through an I2S DAC (MAX98357A). Three top-level voicings (Theremin, Melody, Effects) with orthogonal toggles (DUAL_OSC, DRONE_HOLD, DELAY, ARP, WAVEFORM) set via RB-held + face button.

**Status**: Phase 3 — voicing-based control model with spoken voicing announcements. Full signal chain: dual DDS oscillators → resonant TPT SVF filter → LFO modulation → 0.5 s delay → I2S DAC. On voicing switch, a Gemini-generated TTS clip ("Theremin" / "Melody" / "Effects") plays as the single entry cue; the musical signature gesture is used only when `voice_announce` is off. Per-voicing config persists across voicing switches.

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

# Regenerate voicing-announcement TTS clips (needs GEMINI_API_KEY)
just tts-generate   # Writes data/tts_<clip>.pcm via Gemini TTS API

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

Tweak sticks (typically RY/RX) use **rate control**: stick displacement = rate of change of the parameter, not absolute position. Holding off-center changes the value over time; releasing to center holds the last value. This makes it possible to find a sweet spot and release, rather than having to hold the stick still. A short "bump" blip fires when a parameter reaches its min/max limit.

Primary pitch sticks (LY in Continuous/Discrete) stay **absolute** for theremin-style muscle memory — stick up = high note. Under `DRONE_HOLD`, LY becomes integrating too (slow drift).

Tweak parameter state (cutoff, resonance, delay time, feedback, detune, pitch offset) persists across ticks within a voicing, resets on voicing switch, and resets on LS-click.

Per-voicing configuration (`DUAL_OSC`, `DRONE_HOLD`, `DELAY`, `ARP`, `WAVEFORM`, interval semitones) persists **across** voicing switches, so you can tune up a Continuous setup, switch to Discrete for a chord bridge, and come back with the Continuous toggles still on.

### Global Buttons (every voicing)

| Button | Action |
|---|---|
| D-pad ↑/↓ | Master volume nudge (±0.05 per step), auto-repeats at 4 Hz |
| D-pad ←/→ | Drum tempo nudge (±5 BPM), also drives the arp step rate |
| Share/View/− (`MISC_BACK`) | Cycle voicing (3 choices). Brief LED flash, then a per-voicing signature gesture plays. |
| Home/PS/Xbox tap | Toggle drum engine on/off. Pattern/volume come from the settings page. |
| Home/PS/Xbox held + A/B/X/Y | Select drum pattern 1/2/3/4 directly (auto-starts drums if off) |
| Menu/Options/+ | Enter/exit settings-edit overlay. Inside, d-pad becomes field navigation (not volume/tempo). |
| LS click (left-stick press) | Reset current voicing's tweak parameters to defaults |
| LT / RT triggers | ±7-semitone pitch bend (global in pitched voicings) |
| **RB + A** | Toggle `DUAL_OSC` (Theremin) |
| **RB + B** | Toggle `DRONE_HOLD` (Theremin; forces DUAL_OSC on) |
| **RB + X** | Toggle `DELAY` (Theremin + Melody) |
| **RB + Y** | Cycle `WAVEFORM` (global: square → saw → triangle → sine) |
| **RB + LB** | Toggle `ARP` (Melody) |

While RB is held, face buttons are fully suppressed from normal voicing dispatch so the modifier-key metaphor stays clean. All toggles play a confirmation cue (two-blip up / single down / waveform-specific blip).

**Settings-edit overlay**: d-pad ←/→ moves cursor between fields (drum_pattern, drum_volume, lfo_rate_hz, lfo_depth, lfo_target, voice_announce); ↑/↓ adjusts the selected field (any direction toggles a bool). Each cursor move plays a short value ladder so the user can audit the current value without a screen — for `voice_announce`, 1 blip = off, 2 blips = on. Master volume and tempo live on the global d-pad.

### Voicings (cycled via View/Share/- button; each plays exactly one entry cue)

On voicing entry the flow is: LED flash → single entry cue → return to real-time dispatch. The cue is the spoken voicing name (TTS) when `voice_announce` is on, or the musical signature gesture when it's off — not both. Drums are ducked to 0 during the spoken clip so two-syllable words stay intelligible. The control task blocks for clip duration + 80 ms tail.

1. **Theremin** (Mono + Dual Osc + Delay Synth + Drone collapsed) — LY=pitch (absolute; integrating under DRONE_HOLD), LX=vibrato (or filter cutoff under DRONE_HOLD), RY=filter cutoff (integrating, log; osc B pitch under DRONE_HOLD), RX=filter resonance (integrating). Face buttons select dual-osc interval (unison/5th/8va/2×8va) when DUAL_OSC is on. LB+RY/RX fine-tunes delay time/feedback when DELAY is on. DRONE_HOLD routes osc B to the piezos at a fixed 1.02 detune ratio. Fallback signature (voice off): pitch-bend chirp on sawtooth.
2. **Melody** (Scale + Arpeggio collapsed) — LY integrates pitch offset (±12 st). Without ARP: face buttons play scale degrees (A=Do/B=Re/X=Mi/Y=Fa; LB held → Sol/La/Ti/Do); RY=fine bend (±50 Hz, absolute); RX=filter cutoff (integrating). With ARP: face buttons pick chord (major/minor/7th/dim); LX-zones pick pattern (left=down, right=up, center=up-down); RT toggles arp running; step rate from the global tempo (16ths). Fallback signature: Do-Mi-Sol on sine.
3. **Effects** (Retro SFX) — A/B/X/Y (no LB) trigger Laser/Explosion/Power-up/Coin; LB+face triggers Siren/Engine/Jump/Warp. RT = speed multiplier (0.3-1.0x). Filter and delay bypassed. Fallback signature: laser zap.

### TTS Voicing Announcements

A Python tool under `tools/tts/` calls the Gemini TTS API (`gemini-3.1-flash-tts-preview`) with the `Algenib` prebuilt voice ("gravelly") to generate short spoken clips for each voicing. Output is raw 16-bit LE PCM at 24 kHz mono, written to `data/tts_<clip>.pcm`. Clips are committed to git; `just tts-generate` is the only way they change, and it needs `GEMINI_API_KEY` in the environment (sourced from `~/.api_tokens`).

The firmware embeds the PCM blobs via ESP-IDF's `EMBED_FILES` (main/CMakeLists.txt). `tts_player.c` linearly upsamples 24 kHz → 44.1 kHz and saturating-adds into the shared stereo buffer each block — same overlay pattern as `drums_render_block()`. `main.c:tts_play_voicing()` starts a clip in `enter_voicing()` and blocks the control task for clip duration + 80 ms. No signature gesture follows — exactly one cue per switch.

Editing wording or voice: edit `tools/tts/voices.json` and run `just tts-generate`. Regeneration is intentionally an explicit developer action, not part of the firmware build, so CI never needs a Gemini API key.

### Dependencies

- **Bluepad32**: External component in `external/bluepad32/`. Shared with xbox-switch-bridge via symlink. CI clones it directly.
- **ESP-IDF components**: `driver`, `nvs_flash`, `esp_timer`, `bluepad32` (private require)

## File Structure

```
gamepad-synth/
├── main/
│   ├── main.c              # Synth, gamepad, voicings, settings, audio task
│   ├── drums.c / drums.h    # 16-step sequencer + 3 voices, overlay-mixes
│   ├── piezo_voice.c / .h   # LEDC-driven piezo accents for DRONE_HOLD
│   ├── tts_player.c / .h    # 24 kHz → 44.1 kHz PCM overlay player
│   └── CMakeLists.txt       # Component registration + EMBED_FILES
├── data/                    # Generated PCM clips (committed, embedded)
│   ├── tts_theremin.pcm
│   ├── tts_melody.pcm
│   └── tts_effects.pcm
├── tools/tts/               # uv workspace for Gemini TTS generation
│   ├── pyproject.toml
│   ├── voices.json          # {clip, text, voice} — edit to retune wording
│   └── generate.py          # `just tts-generate` entry point
├── CMakeLists.txt           # Project-level CMake (sets EXTRA_COMPONENT_DIRS for Bluepad32)
├── sdkconfig.defaults       # ESP-IDF config (BT controller-only, no WiFi, no PSRAM)
├── justfile                 # Build/flash/monitor recipes + tts-generate
├── dependencies.lock        # ESP-IDF component lock
├── version.txt              # Semantic version (used by CMake and release-please)
├── README.md                # User-facing documentation
└── WIRING.md                # Hardware wiring guide
```

## Code Conventions

- **Module split**: Drum engine, piezo voices, and TTS player live in their own translation units; the synth core, gamepad handling, voicings, and audio task stay in `main/main.c`. Further splits (oscillators / filter / LFO) are still TBD.
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
