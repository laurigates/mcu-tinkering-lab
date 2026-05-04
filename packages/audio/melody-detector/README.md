# melody-detector

A children's musical toy on a single XIAO ESP32-S3 Sense. Draw notes on a
printed staff sheet, press the button, hear the melody play through a small
speaker. All inference is on-device — no cloud, no WiFi at runtime.

This is also a TinyML showcase: the whole pipeline (synthetic data → PyTorch
training → INT8 quantization → ESP-DL deployment) is open source and
reproducible from `gen.py` + a seed.

See [PRD-011](../../../docs/requirements/PRD-011-melody-detector.md) and
[ADR-017](../../../docs/decisions/ADR-017-melody-detector-esp-dl.md) for the
system-level decisions, and the
[implementation plan](../../../docs/prompts/melody-detector.md) for tracks
and dependencies.

## Status

**Phase 1 — hardware bring-up** (this commit). Boots, accepts a debounced
button press, captures a JPEG frame from the OV2640, and plays a 1 kHz test
tone through the MAX98357A. No CV, inference, or melody synthesis yet.

| Phase | Track | Status |
|---|---|---|
| 1. Bring-up | A — scaffolding | this commit |
| 2. Classical CV | C — fiducial / homography / staff lines | not started |
| 3. ESP-DL inference | D — INT8 ROI classifier | not started |
| 4. Audio synthesis | E — DDS + ADSR + sequencer | not started |
| 5. Training pipeline | B — PIL + PyTorch + ESP-PPQ | not started |
| 6. Validation | G — Gemini ER 1.6 oracle benchmark | not started |

## Hardware

- **Seeed Studio XIAO ESP32-S3 Sense** (ESP32-S3, 8 MB octal PSRAM, OV2640)
- **MAX98357A** I2S Class-D mono amplifier
- 3 W 4 Ω or 8 Ω speaker
- Tactile push-button (capture trigger)
- USB-C power; no battery in v1

See [WIRING.md](WIRING.md) for the pin map.

## Build & flash

Containerized ESP-IDF v5.4 — no local toolchain needed.

```bash
just build
just flash-monitor
```

The XIAO uses native USB-Serial-JTAG; the justfile auto-detects VID `0x303a`.
If flashing fails, hold BOOT and tap RESET to enter download mode.

## Phase 1 acceptance

- `just build` succeeds in the container
- Binary fits the 1.75 MB CI guard (well under the 1.8 MB OTA partition)
- Pressing the button: status LED transitions idle → capturing → processing
  → playing, the serial log reports the captured frame size, and a 1 kHz
  test tone plays for 500 ms

## Layout

```
packages/audio/melody-detector/
├── CMakeLists.txt
├── partitions.csv          OTA-capable (3.5 MB ota_0 + ota_1 + 448 KB SPIFFS)
├── sdkconfig.defaults
├── justfile
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml   esp32-camera (ESP-DL added in Track D)
│   ├── main.c              app_main + capture loop
│   ├── pin_config.h        XIAO header GPIO assignments
│   ├── camera_pins.h       OV2640 internal pin map
│   ├── camera_capture.{c,h}  JPEG snapshot to PSRAM (FR-1.02)
│   ├── audio_test.{c,h}    I2S 1 kHz test tone (FR-1.03)
│   ├── button.{c,h}        debounced capture trigger
│   └── status_led.{c,h}    idle/capturing/processing/playing/error patterns (FR-1.04)
├── sheets/
│   └── blank-staff.tex     printable A4 staff sheet with corner fiducials
└── training/               (gitignored; populated by Track B)
```
