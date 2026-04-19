# Gamepad Synth

Turn a Bluetooth controller into a musical instrument. An ESP32-S3 reads gamepad input via [Bluepad32](https://github.com/ricardoquesada/bluepad32) and produces audio through an I2S DAC (MAX98357A) in four sound modes.

## Hardware

- ESP32-S3 development board (any with USB-Serial-JTAG)
- MAX98357A I2S DAC breakout (BCLK=GPIO5, WS=GPIO6, DIN=GPIO7)
- 4-8 ohm speaker (2-3W)
- Status LED on GPIO2

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

Press **View** (Xbox) / **Share** (PS) / **-** (Switch) to cycle through modes. The LED blinks 1-4 times to indicate the current mode.

### Mode 1: Theremin

Continuous pitch control with Monotron-style resonant filter and LFO wah.

| Control | Function |
|---------|----------|
| Left stick Y | Base pitch (100-2000 Hz) |
| Left stick X | Vibrato depth (fixed 5 Hz speed) |
| Right stick Y | Filter cutoff (40 Hz to 18 kHz, logarithmic) |
| Right stick X | Filter resonance (self-oscillation at max) |
| LT | LFO rate (0.1 - 20 Hz) |
| RT | LFO depth (0 = off, max = ±2 octaves of cutoff modulation) |

Release left stick to silence. Sweep right stick Y upward while holding right stick X for classic Monotron squelch. Hold RT while varying LT for filter "wah".

### Mode 2: Scale Player

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

### Mode 3: Arpeggiator

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

### Mode 4: Retro SFX

Trigger classic game sound effects with button presses.

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
- **Waveforms**: Square, sawtooth, triangle, sine (256-entry lookup table), noise. Each mode uses a default waveform: Theremin=sawtooth, Scale=sine, Arpeggio=square, SFX=square
- **Filter**: State-variable low-pass with cutoff (40 Hz - 18 kHz) and resonance (Q 0.5 - 6.0). Chamberlin topology, coefficients recomputed once per 256-sample block. Self-oscillates at high resonance
- **LFO**: Block-rate triangle LFO (0.1 - 20 Hz) modulating filter cutoff or oscillator pitch. Up to ±2 octaves cutoff mod or ±1 octave pitch mod at full depth
- **Delay**: Circular-buffer delay line (0.5 s / 22050 samples, ~44 KB static RAM) with configurable delay time, feedback (up to 0.95), and wet/dry mix. Scale = slapback, Arpeggio = cosmic echo
