# CLAUDE.md - melody-detector

Project-specific guidance for Claude Code. See repo-root `CLAUDE.md` for
monorepo-wide conventions, and the canonical design documents:

- [PRD-011: Melody Detector](../../../docs/requirements/PRD-011-melody-detector.md)
- [ADR-017: ESP-DL for On-Device Vision](../../../docs/decisions/ADR-017-melody-detector-esp-dl.md)
- [Implementation plan / track dependencies](../../../docs/prompts/melody-detector.md)

## What this project is

A self-contained children's musical toy. Draw notes on a printed staff
sheet, press the button, hear the melody play through a speaker. All
inference runs on-device. WiFi is reserved for OTA only (deferred to v2).

## Architecture (target — built up over Phase 1 → 6)

```
button → camera capture → classical CV (fiducial + homography + ROI grid)
                       → ESP-DL CNN classifier (per-ROI)
                       → geometric pitch lookup
                       → I2S DDS oscillator + ADSR
                       → MAX98357A → speaker
```

Pitch is derived from ROI **row** geometrically; the model is a per-ROI
classifier (`empty / quarter / half / rest / other`), not a detector. This
keeps the model small (~50–200 KB INT8) and offloads spatial reasoning to
deterministic geometry anchored on the printed fiducials. See ADR-017.

## Phase 1 (current)

`main.c` runs a single-threaded capture loop. The status LED is driven by
a small task pinned to Core 0; the I2S driver runs on whichever core
emits the tone (currently `app_main`'s task). Phase 4+ will move audio
playback to a dedicated Core 0 task at high priority.

## Pin map

`main/pin_config.h` is the source of truth. Highlights:

- I2S to MAX98357A: BCLK=GPIO6, LRC=GPIO43, DIN=GPIO44
- Capture button: GPIO2 (active LOW, internal pull-up)
- Status LED: GPIO21 (onboard, active LOW)
- Camera: internal Sense pins (see `main/camera_pins.h`)

GPIO43/44 are only usable as I2S because we run the console on
USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`). Don't switch the
console to UART without re-picking the I2S pins.

## sdkconfig rules

See repo `.claude/rules/esp-idf-sdkconfig.md`. After modifying
`sdkconfig.defaults`, delete the generated `sdkconfig` and run
`just clean` before rebuilding — ESP-IDF preserves existing values and
silently ignores new defaults otherwise.

Settings that matter:

- `CONFIG_SPIRAM_MODE_OCT=y` — XIAO ESP32-S3 Sense has **octal** PSRAM
  (not quad); wrong mode = boot loop
- `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` — bumped from 3584 for camera +
  I2S + eventual ESP-DL warmup
- `CONFIG_CAMERA_CORE1=y` — DMA isolated to Core 1 so audio I2S DMA on
  Core 0 isn't disturbed by frame captures
- `CONFIG_ESP_BROWNOUT_DET=n` — camera + flash inrush on weak USB ports
  trips it

## Don't

- Don't add WiFi / mDNS to v1 — runtime is offline by spec (PRD-011).
  WiFi/OTA wiring is a v2 deliverable.
- Don't bypass `audio_test_init` ↔ `audio_test_tone` for Phase 1; the FR-4
  synth replaces this whole module in Track E and the I2S driver should
  be initialised once, not per-tone.
- Don't read `gpio_get_level(BUTTON_PIN)` directly outside `button.c` —
  the debounce logic owns the press semantics.
- Don't change camera pin assignments — the OV2640 is wired internally
  on the Sense expansion board.
- Don't pull in ESP-DL as a managed component until Track D — it bloats
  the build with unused code.
