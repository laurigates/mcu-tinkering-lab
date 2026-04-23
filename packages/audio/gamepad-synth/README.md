# Gamepad Synth

Turn a Bluetooth controller into a Korg Monotron-inspired synthesizer. An ESP32-S3 reads gamepad input via [Bluepad32](https://github.com/ricardoquesada/bluepad32) and produces audio through an I2S DAC (MAX98357A) in seven sound modes. Features dual DDS oscillators, resonant SVF filter, LFO modulation, and a 0.5-second delay line.

## Hardware

- ESP32-S3 development board (any with USB-Serial-JTAG)
- MAX98357A I2S DAC breakout (BCLK=GPIO5, WS=GPIO6, DIN=GPIO7)
- 4-8 ohm speaker (2-3W)
- Status LED on GPIO2
- Optional: two piezo discs on GPIO8/GPIO9 for Drone-mode accent voices

See [WIRING.md](WIRING.md) for the full wiring guide.

## Compatible Controllers

The ESP32-S3 only supports **BLE** (no Bluetooth Classic). Compatible controllers include:

| Controller | BLE Support | Notes |
|------------|-------------|-------|
| Xbox Series X/S | Firmware v5.15+ | Update via Xbox Accessories app on Windows |
| PS5 DualSense | Yes | |
| Nintendo Switch Pro | Yes | |
| Nintendo Joy-Cons | Yes | |
| 8BitDo controllers | Most models | Check BLE mode in manual |

**Not compatible** (requires Bluetooth Classic / BR/EDR):
- PS4 DualShock 4
- Xbox One (older models)
- Wii remotes

## Pairing

1. Flash and power on the ESP32-S3
2. Put the controller in pairing mode:
   - **Xbox Series**: Hold the small pair button on top until the Xbox button flashes rapidly
   - **PS5 DualSense**: Hold Share + PS button until the light bar flashes
   - **Switch Pro**: Hold the sync button on top of the controller
3. The LED blinks and the startup jingle plays when the board boots
4. Once paired, the controller auto-reconnects on subsequent power-ups (just press the home button)

## Sound Modes

Press **View** (Xbox) / **Share** (PS) / **-** (Switch) to cycle through modes. The LED blinks 1-7 times to indicate the current mode.

### Mode 1: Mono Synth

Monotron-style single oscillator with resonant filter and LFO wah.

| Control | Function |
|---------|----------|
| Left stick Y | Base pitch (100-2000 Hz) |
| Left stick X | Vibrato depth (fixed 5 Hz speed) |
| Right stick Y | Filter cutoff (40 Hz to 18 kHz, logarithmic) |
| Right stick X | Filter resonance (self-oscillation at max) |
| LT | LFO rate (0.1 - 20 Hz) |
| RT | LFO depth (0 = off, max = ±2 octaves of cutoff modulation) |

Sweep right stick Y upward while holding RX for classic Monotron squelch. Hold RT while varying LT for filter "wah".

### Mode 2: Dual Osc

Two sawtooth oscillators with selectable interval and detune for fat analog sound.

| Control | Function |
|---------|----------|
| Left stick Y | Base pitch |
| A / B / X / Y | Interval: unison / fifth / octave / two octaves |
| Right stick X | Detune (±50 cents) |
| Right stick Y | Filter cutoff |

### Mode 3: Delay Synth

Single oscillator with dynamic delay-as-secondary-voice (Monotron Delay behavior).

| Control | Function |
|---------|----------|
| Left stick Y | Base pitch |
| Right stick Y | Delay time (20 - 500 ms) |
| Right stick X | Feedback (up to 0.9 — Karplus-Strong territory at max) |
| LT | Filter cutoff down |
| RT | Filter cutoff up |

Short delay + high feedback produces string-like tones. Long delay + moderate feedback creates cosmic echoes.

### Mode 4: Scale Player

Play a C major scale with buttons and d-pad. Each button maps to a scale degree.

| Control | Note |
|---------|------|
| A | Do (C) |
| B | Re (D) |
| X | Mi (E) |
| Y | Fa (F) |
| D-pad Up | Sol (G) |
| D-pad Right | La (A) |
| D-pad Down | Ti (B) |
| D-pad Left | Do (C, high) |
| LB | Octave down |
| RB | Octave up |
| Right stick Y | Pitch bend |

Three octave range: C4, C5 (default), C6.

### Mode 5: Arpeggiator

Automatically cycles through chord notes. Select a chord, then toggle the arpeggio on.

| Control | Function |
|---------|----------|
| A | Major chord |
| B | Minor chord |
| X | Dominant 7th chord |
| Y | Diminished chord |
| RT | Toggle arpeggio on/off |
| Left stick Y | Speed (50-500 ms per note) |
| Left stick X | Pattern: left=down, right=up, center=up-down |
| D-pad Up/Down | Transpose root note up/down |
| LB | Octave down |
| RB | Octave up |

### Mode 6: Retro SFX

Trigger classic game sound effects with button presses (filter and delay bypassed for raw effects).

| Control | Sound Effect |
|---------|-------------|
| A | Laser (descending sweep) |
| B | Explosion (low rumble) |
| X | Power-up (ascending sweep) |
| Y | Coin (high chirp) |
| D-pad Up | Siren (oscillating) |
| D-pad Down | Engine (low rumble) |
| D-pad Left | Jump (ascending chirp) |
| D-pad Right | Warp (sweep up) |
| RT | Speed multiplier (0.5x-2x) |

### Mode 7: Drone

Two continuously-sustained oscillators with LFO modulation for evolving textures. Oscillator A plays through the DAC; oscillator B plays through the optional piezo pair (if wired) with a fixed 1.02 detune ratio between the two discs, so the beating happens acoustically in air.

| Control | Function |
|---------|----------|
| Left stick Y | Oscillator A pitch (sawtooth, DAC) |
| Right stick Y | Oscillator B pitch (square wave on piezos) |
| LT | LFO rate |
| RT | LFO depth (modulates both pitch and filter cutoff on the DAC voice) |

No note-off — the drone plays continuously while in this mode. Dial in a static interval with the sticks, then add slow LFO modulation for a shifting ambient pad. Without the piezos wired, oscillator B is silent and only the DAC drone is heard.

## Building

Requires Docker (builds are containerized, no local ESP-IDF needed):

```bash
just build
just flash
just monitor
```

## Architecture

- **Core 0**: Bluepad32 BTstack event loop (Bluetooth handling)
- **Core 1**: Two tasks:
  - **Control task** (50 Hz): reads gamepad state, updates synth parameters
  - **Audio render task** (continuous, priority 10): DDS phase accumulator with selectable waveform, writes 256-sample blocks to I2S DMA
- **Audio output**: 44.1 kHz, 16-bit stereo (mono duplicated) via MAX98357A I2S DAC
- **Piezo accents** (Drone mode only): two GPIO-driven LEDC square-wave voices on GPIO8/GPIO9 with a fixed 1.02 detune ratio, running in parallel to the DAC path
- **Waveforms**: Square, sawtooth, triangle, sine (256-entry lookup table), noise. Per-mode defaults: Mono/Dual Osc/Delay Synth/Drone=sawtooth, Scale=sine, Arpeggio=square, SFX=square
- **Filter**: State-variable low-pass with cutoff (40 Hz - 18 kHz) and resonance (Q 0.5 - 6.0). Chamberlin topology, coefficients recomputed once per 256-sample block. Self-oscillates at high resonance
- **LFO**: Block-rate triangle LFO (0.1 - 20 Hz) modulating filter cutoff or oscillator pitch. Up to ±2 octaves cutoff mod or ±1 octave pitch mod at full depth
- **Delay**: Circular-buffer delay line (0.5 s / 22050 samples, ~44 KB static RAM) with configurable delay time, feedback (up to 0.95), and wet/dry mix. Scale = slapback, Arpeggio = cosmic echo
