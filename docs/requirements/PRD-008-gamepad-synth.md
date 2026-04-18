# PRD-008: Gamepad Synth

**Status**: active
**Created**: 2026-04-07
**Confidence**: 9/10

---

## Overview

Gamepad Synth turns a Bluetooth gamepad (Xbox Series X/S) into a Korg Monotron-inspired synthesizer. An ESP32-S3 reads BLE gamepad input via Bluepad32 and produces real-time audio through an I2S DAC (MAX98357A). The analog sticks, triggers, buttons, and d-pad map to oscillator pitch, filter cutoff, resonance, LFO, delay, and mode selection.

## Problem

The current implementation (`v0.1.0`, formerly `esp32-ps4-speaker`) uses LEDC PWM to drive a passive piezo buzzer. This produces only square waves with harsh, limited tonal range and no ability to shape the sound. The Korg Monotron lineup demonstrates that a single oscillator with a resonant filter, LFO, and delay can produce remarkably expressive sounds from minimal hardware.

## Goals

- Replace LEDC PWM piezo output with I2S DAC for real waveform synthesis
- Implement a software synthesizer with oscillator, filter, LFO, and delay modules
- Map all synthesis parameters to Xbox controller inputs (no external controls)
- Provide 7 distinct sound modes covering different musical use cases
- Maintain the dual-core architecture: Bluepad32 on Core 0, audio on Core 1

## Non-Goals

- No WiFi, OLED display, or physical controls (potentiometers, encoders)
- No MIDI output or external sync
- No polyphony (monophonic only, matching the Monotron)
- No audio recording or streaming

## Hardware

| Component | Specification |
|-----------|--------------|
| MCU | ESP32-S3 (any dev board with USB-Serial-JTAG) |
| DAC | MAX98357A I2S amplifier module |
| Speaker | 3W 4-ohm (or 8-ohm) small speaker |
| LED | Status LED on GPIO2 (existing) |
| Controller | Xbox Series X/S (BLE, firmware v5.15+) |

## Features

### Phase A: I2S DAC Output + DDS Oscillator

**FR-A01: I2S streaming to MAX98357A**
Replace LEDC PWM with continuous I2S PCM output at 44.1 kHz, 16-bit. Audio render task on Core 1 fills 256-sample blocks via `synth_render()` callback.

**FR-A02: DDS oscillator with multiple waveforms**
Phase-accumulator oscillator supporting: sawtooth, triangle, sine, square, and noise. Waveform selectable per mode.

**FR-A03: Re-implement existing modes on I2S engine**
Theremin, Scale, Arpeggiator, and Retro SFX modes continue working identically but with richer waveforms instead of square-only.

### Phase B: Resonant Low-Pass Filter

**FR-B01: State-variable filter (SVF)**
Software low-pass filter with cutoff frequency and resonance parameters. Provides the characteristic Korg Monotron squelch when resonance is high.

**FR-B02: Controller mapping**
Right stick Y controls filter cutoff (100 Hz to 10 kHz). Right stick X controls resonance (0 to self-oscillation threshold).

### Phase C: LFO Modulation

**FR-C01: Low-frequency oscillator**
Sub-audio oscillator (0.1-20 Hz) with triangle and square waveforms. Modulates oscillator pitch (vibrato), filter cutoff (wah), or both.

**FR-C02: Controller mapping**
LT (brake) controls LFO rate. RT (throttle) controls LFO depth. LFO target depends on active mode.

### Phase D: Delay Effect

**FR-D01: Circular buffer delay with feedback**
0.5-second delay buffer (~44 KB RAM) with configurable delay time and feedback amount. Short delay + high feedback produces Karplus-Strong string-like tones. Long delay + moderate feedback creates the Monotron Delay's cosmic echo.

**FR-D02: Controller mapping**
Mapped per-mode. In Delay Synth mode: right stick Y = delay time, right stick X = feedback.

### Phase E: New Modes

**FR-E01: Mono Synth mode**
Single oscillator + filter + LFO. Classic Monotron emulation. Left stick Y = pitch, right stick Y = filter cutoff, right stick X = resonance, triggers = LFO rate/depth.

**FR-E02: Dual Osc mode**
Two oscillators with controllable interval and detune. Face buttons select interval (unison, fifth, octave, two octaves). Right stick X = detune amount for fat analog sound.

**FR-E03: Delay Synth mode**
Oscillator + delay with feedback. The delay acts as a second sound source when feedback is high (Monotron Delay behavior). Left stick Y = pitch, right stick Y = delay time, right stick X = feedback.

**FR-E04: Drone mode**
Two sustained oscillators, LFO modulates everything. Left stick Y = osc A pitch, right stick Y = osc B pitch, triggers = filter parameters. No note-off — continuous evolving texture.

## Mode Map

| # | Mode | Osc | Filter | LFO | Delay | Controller Summary |
|---|------|-----|--------|-----|-------|--------------------|
| 1 | Mono Synth | 1 | Yes | Yes | No | LY=pitch, RY=cutoff, RX=resonance, LT/RT=LFO |
| 2 | Dual Osc | 2 | Yes | No | No | LY=pitch, A/B/X/Y=interval, RX=detune, RY=cutoff |
| 3 | Delay Synth | 1 | Yes | No | Yes | LY=pitch, RY=delay time, RX=feedback, LT/RT=filter |
| 4 | Scale | 1 | Yes | No | No | A/B/X/Y/dpad=notes, LB/RB=octave, RY=cutoff |
| 5 | Arpeggio | 1 | Yes | No | No | A/B/X/Y=chord, RT=toggle, LY=speed, RY=cutoff |
| 6 | Retro SFX | 1 | No | No | No | A/B/X/Y/dpad=effects, RT=speed |
| 7 | Drone | 2 | Yes | Yes | No | LY=oscA, RY=oscB, LT/RT=filter/LFO |

View/Share button cycles modes (same as current). LED blinks 1-7 times to indicate mode number.

## Constraints

- **Memory**: ~512 KB SRAM, no PSRAM. Delay buffer (44 KB) + DMA (4 KB) + Bluepad32/BTstack (~60 KB) must fit.
- **Real-time**: Audio render must complete 256 samples in <5.8 ms (44100 Hz). ESP32-S3 at 240 MHz handles this easily.
- **Core affinity**: Bluepad32 BTstack event loop owns Core 0 entirely. Audio engine runs on Core 1.
- **Bluepad32 compatibility**: ESP32-S3 BLE only. Compatible with Xbox Series, PS5 DualSense, Switch Pro, 8BitDo.

## Implementation Order

1. Rename project (this PR)
2. Phase A: I2S + DDS oscillator
3. Phase B: Filter
4. Phase C: LFO
5. Phase D: Delay
6. Phase E: New modes

Each phase produces a working, flashable firmware.

## Related Documents

- [ADR-011: Gamepad Synth I2S Audio Architecture](../decisions/ADR-011-gamepad-synth-i2s-audio.md)
