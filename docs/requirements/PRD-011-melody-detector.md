# PRD-011: Melody Detector

**Status**: draft
**Created**: 2026-04-26
**Confidence**: 7/10

---

## Overview

Melody Detector is a self-contained children's musical toy. A child draws
notes on a pre-printed staff sheet, places it under the device's camera,
presses a button, and hears the melody play back through a built-in
speaker. All inference runs on-device — no cloud, no WiFi at runtime.

The device is a single XIAO ESP32-S3 Sense board paired with a MAX98357A
I2S amplifier and a small speaker. Printable A4 staff sheets with corner
fiducials are checked into the repository and provide the geometric
ground truth the on-device CV pipeline depends on.

This is also a TinyML showcase: the entire pipeline (synthetic data
generation, PyTorch training, INT8 quantization, ESP-DL deployment) is
open source and reproducible from a `gen.py` + seed.

## Problem

Children learning music have a chicken-and-egg problem: composing melodies
typically requires reading and writing standard notation, but reading and
writing notation is itself a multi-year skill. Drawing-based interfaces
collapse the gap — a child who can place a dot on a line can compose a
melody — and immediate audio feedback turns composition into play. No
existing affordable toy combines pre-printed staff paper with offline
on-device note recognition.

## Goals

- Children compose monophonic melodies by drawing on printed sheets
- Self-contained device: capture, recognize, and play locally with no
  runtime network dependency
- Robust to messy real-world drawings (varying note shapes, paper
  texture, lighting, hand-drawn imperfection)
- Deployment-ready firmware on a XIAO ESP32-S3 Sense (~$15 board)
- All-OSS pipeline suitable for a public showcase

## Non-Goals

- No melody composition assistance, autocomplete, or correction
- No multi-voice / polyphonic playback (monophonic only in v1)
- No rhythms beyond quarter / half / rest
- No internet connectivity at runtime (WiFi reserved for OTA only)
- No editing UI — each capture produces a fresh playback
- No companion app (v1 is fully standalone)

## User Stories

### Primary user — child (age 5–10)

- **US-01**: As a child, I want to draw notes on a pre-printed sheet and
  press a button to hear my song, so that I can experience my own
  composition.
- **US-02**: As a child, I want to draw approximately (close to but not
  perfectly on a line) and still have my notes recognized, so that the toy
  forgives my motor-skill limits.
- **US-03**: As a child, I want clear feedback (LED + audio chirp) when
  the toy is reading vs. playing, so that I know what's happening.
- **US-04**: As a child, I want to hear my song clearly even in a noisy
  room, so that I can share it with siblings or friends.
- **US-05**: As a child, I want short, hollow, and rest symbols to
  produce noticeably different sounds, so that I can hear the structure
  of my melody.

### Secondary user — parent / educator

- **US-06**: As a parent, I want to print fresh sheets on demand using a
  standard inkjet printer, so that we don't run out of paper.
- **US-07**: As a parent, I want the toy to work offline anywhere
  (camping, car), so that it doesn't depend on WiFi.
- **US-08**: As a parent, I want pitch-letter labels in the margin of the
  printable, so that my child can learn note-naming as a side effect of
  play.
- **US-09**: As a parent, I want to safely re-flash firmware updates
  without specialized tools, so that the device stays useful long-term.
- **US-10**: As an educator, I want a visible class boundary (filled =
  short, hollow = long, squiggle = rest), so that the recognizable
  vocabulary scales with the child.

### Tertiary user — developer / showcase audience

- **US-11**: As a developer, I want a reproducible end-to-end OSS
  pipeline (synthetic data → train → quantize → deploy), so that I can
  fork and adapt for similar projects.
- **US-12**: As a developer, I want documented inference benchmarks
  against a cloud baseline (Gemini Robotics-ER 1.6), so that I can evaluate
  the on-device accuracy / cost tradeoff.
- **US-13**: As a developer, I want the synthetic data generator and
  augmentation parameters documented and seedable, so that experimental
  results are reproducible.
- **US-14**: As a maintainer, I want host-based unit tests for the
  classical-CV stage (no hardware required), so that I can validate
  pipeline changes in CI.

## Hardware

| Component | Specification |
|-----------|--------------|
| MCU | Seeed Studio XIAO ESP32-S3 Sense (dual-core LX7, 8 MB Octal PSRAM, 8 MB flash) |
| Camera | OV2640 (built into Sense expansion) |
| DAC / Amp | MAX98357A I2S Class-D amplifier module |
| Speaker | 3 W 4 Ω or 8 Ω small enclosure speaker |
| Capture trigger | Tactile push-button on a free GPIO |
| Status indicator | On-board user LED + (optional) RGB indicator |
| Power | USB-C (no battery in v1) |
| Print medium | A4 inkjet paper, standard 80–100 g/m², printed at 100 % scale |

## Features

### Phase 1 — Hardware bring-up + scaffolding

**FR-1.01**: ESP-IDF project skeleton at
`packages/audio/melody-detector/` registered in the root justfile and
CI matrix.

**FR-1.02**: Camera capture on button press (single-shot JPEG to PSRAM
via the on-board OV2640 driver).

**FR-1.03**: I2S audio output stub: 1 kHz test tone confirms the
amplifier and speaker chain works.

**FR-1.04**: Status LED feedback: distinct patterns for idle / capturing
/ processing / playing.

### Phase 2 — Classical CV pipeline

**FR-2.01**: Fiducial detection. Locate the four corner markers on the
captured frame via threshold + contour detection. Identify the oriented
top-left marker by its inner-dot pattern.

**FR-2.02**: Perspective rectification. Compute a 3 × 3 homography from
the four fiducial centers to the canonical sheet rectangle and warp the
frame to a fixed-resolution staff strip.

**FR-2.03**: Staff line refinement. Sum darkness per row in the
rectified strip; locate the five staff lines as peaks in a known search
window.

**FR-2.04**: ROI grid extraction. Crop a 32 × 32 px patch at each beat
column × staff row intersection (default 8 columns × 9 rows). Optionally
skip patches whose darkness falls below an empty-threshold to save
inference time.

### Phase 3 — ESP-DL inference

**FR-3.01**: Embed the quantized `.espdl` model in firmware via
`EMBED_FILES`.

**FR-3.02**: Load the model at boot using the ESP-DL runtime; allocate
input / output tensors in PSRAM.

**FR-3.03**: Batch ROI inference. Feed the ROI grid through the model;
return an array of class labels (`empty / quarter / half / rest /
other`).

### Phase 4 — Audio synthesis + playback

**FR-4.01**: Pitch / frequency lookup table for treble-clef rows (E4
through F5 inclusive, plus optional ledger-line extensions).

**FR-4.02**: I2S DDS oscillator with sine + square waveforms at 44.1
kHz, 16-bit.

**FR-4.03**: ADSR envelope per note (attack, decay, sustain, release)
to avoid clicks at note boundaries.

**FR-4.04**: Sequence playback. Convert the ROI class grid to a
`[(pitch, duration)]` list (geometric lookup from row + class) and play
sequentially with configurable tempo.

### Phase 5 — Training pipeline

**FR-5.01**: Synthetic ROI generator (PIL + albumentations). Renders 32
× 32 px patches for each class with parametrized augmentations: paper
texture, rotation, scale, blur, noise, ink-bleed, brightness.

**FR-5.02**: PyTorch model definition. Small CNN: 3–4 conv layers,
~50–200 KB INT8 weights, 5-class softmax output.

**FR-5.03**: Training script with reproducibility (random seed,
parameter dump, dataset hash on `sha256sum`).

**FR-5.04**: ONNX export with opset compatible with ESP-PPQ.

**FR-5.05**: ESP-PPQ quantization config + calibration data extraction
(~1 K samples drawn from the synthetic generator).

### Phase 6 — Validation

**FR-6.01**: Real-photo holdout collection (~50–100 photographed kid
drawings).

**FR-6.02**: Oracle labeling via Gemini Robotics-ER 1.6 with manual
spot-checking on ~10 % of the holdout.

**FR-6.03**: Per-class accuracy benchmark vs. the cloud baseline.

**FR-6.04**: Inference latency measurement on the XIAO ESP32-S3 Sense.

## Constraints

- **Memory**: 8 MB PSRAM, 8 MB flash. Model + activations must fit in
  ≤ 1 MB PSRAM.
- **Latency**: < 1 s from button press to first note.
- **Power**: USB-C powered, no battery in v1.
- **Cost**: target BOM under $30 retail (board + amp + speaker +
  enclosure).
- **Print fidelity**: A4 sheet on standard inkjet at 100 % scale, no
  professional printing required.
- **Offline runtime**: WiFi optional and used only for OTA updates.

## Acceptance Criteria

- [ ] On-device per-ROI classification ≥ 90 % accuracy on the real-photo
  holdout.
- [ ] End-to-end latency from button press to first audio sample
  < 1 second.
- [ ] Firmware fits the 1.8 MB OTA partition (CI-enforced).
- [ ] Synthetic dataset reproducible from a single seed +
  `python gen.py`.
- [ ] External developer can build, train, and flash the system in
  under 1 day from a clean checkout.
- [ ] Documented benchmark vs. Gemini Robotics-ER 1.6 oracle published
  in the repo README.

## Implementation Order

1. Track A — Project scaffolding (Phase 1)
2. Track B — Training pipeline (Phase 5) — runs in parallel with A
3. Track C — Classical CV pipeline (Phase 2) — depends on A
4. Track D — ESP-DL inference (Phase 3) — depends on A + first model
   from B
5. Track E — Audio synthesis (Phase 4) — depends on A
6. Track F — Main loop integration — depends on A–E
7. Track G — Validation (Phase 6) — depends on F

Tracks A and B are independent (different languages, different domains)
and can run in parallel. Tracks C–E share the firmware scaffolding from
A. Track G closes the loop with hardware-on-device measurements.

## Related Documents

- [ADR-017: ESP-DL for On-Device Vision](../decisions/ADR-017-melody-detector-esp-dl.md)
- [PRP: Melody Detector Implementation Plan](../prompts/melody-detector.md)
- [TinyML frameworks reference](../reference/tinyml-esp32-frameworks.md)
- [Vision labeling services reference](../reference/vision-labeling-services.md)
- [Synthetic image generation reference](../reference/synthetic-image-generation.md)
- [Printable staff sheet (LaTeX)](../../packages/audio/melody-detector/sheets/blank-staff.tex)
