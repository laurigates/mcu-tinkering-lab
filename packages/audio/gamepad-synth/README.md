# Gamepad Synth

Turn a Bluetooth controller into a Korg Monotron-inspired synthesizer. An ESP32-S3 reads gamepad input via [Bluepad32](https://github.com/ricardoquesada/bluepad32) and produces audio through an I2S DAC (MAX98357A). Three top-level voicings (Continuous, Discrete, One-shot) with orthogonal toggles for dual-osc, drone-hold, delay, arpeggiator, and waveform. Features dual DDS oscillators, resonant SVF filter, LFO modulation, and a 0.5-second delay line.

## Hardware

- ESP32-S3 development board (any with USB-Serial-JTAG)
- MAX98357A I2S DAC breakout (BCLK=GPIO5, WS=GPIO6, DIN=GPIO7)
- 4-8 ohm speaker (2-3W)
- Status LED on GPIO2
- Optional: two piezo discs on GPIO8/GPIO9 for Drone-hold accent voices

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

## Global Controls

| Control | Function |
|---------|----------|
| D-pad ↑/↓ | Master volume (±0.05, auto-repeats at 4 Hz) |
| D-pad ←/→ | Drum tempo (±5 BPM, also drives arp step rate) |
| Share / View / − | Cycle voicing (Continuous → Discrete → One-shot) |
| Home / PS / Xbox tap | Toggle drum engine on/off |
| Home held + A/B/X/Y | Select drum pattern 1-4 |
| Menu / Options / + | Enter/exit settings-edit overlay (d-pad navigates fields) |
| LS click | Reset tweak parameters to defaults |
| LT / RT triggers | ±7-semitone pitch bend in pitched voicings |

## Modifier Toggles (RB-held + face button)

Each toggle plays a confirmation cue and persists per-voicing across voicing switches.

| Combo | Toggle | Scope | Effect |
|-------|--------|-------|--------|
| RB + A | DUAL_OSC | Continuous | Enables osc B; face buttons select interval (unison/5th/8va/2×8va) |
| RB + B | DRONE_HOLD | Continuous | LY+RY integrate pitch; osc B plays on piezos; forces DUAL_OSC on |
| RB + X | DELAY | Continuous + Discrete | Enables delay tail; LB held + RY/RX fine-tunes time/feedback |
| RB + Y | WAVEFORM | All | Cycles square → saw → triangle → sine |
| RB + LB | ARP | Discrete | Overlays arpeggiator; face button picks chord, steps fire at 16ths |

## Voicings

Press **Share / View / −** to cycle. Each voicing plays a signature gesture on entry so you can identify the voicing without a screen.

### Continuous

Monotron-style instrument: LY is the pitch stick, the right stick shapes the tone.

| Control | Function |
|---------|----------|
| Left stick Y | Pitch (100-2000 Hz, absolute; integrating under DRONE_HOLD) |
| Left stick X | Vibrato depth at 5 Hz (absolute; filter cutoff integrator under DRONE_HOLD) |
| Right stick Y | Filter cutoff (40 Hz - 18 kHz, log, integrating; osc B pitch under DRONE_HOLD) |
| Right stick X | Filter resonance (0.5 - 6.0, integrating) |
| A/B/X/Y (no RB) | Interval: unison/5th/8va/2×8va (only when DUAL_OSC is on) |
| LB + Right stick Y | Delay time adjust (when DELAY is on) |
| LB + Right stick X | Delay feedback adjust (when DELAY is on) |
| LT / RT | ±7-semitone pitch bend |

Sweep RY upward while nudging RX for classic Monotron squelch. Toggle DRONE_HOLD (RB+B) for sustained drones that drift when sticks are released.

### Discrete

Face buttons pick scalar notes. Optional arpeggiator overlay.

Without ARP:

| Control | Function |
|---------|----------|
| A / B / X / Y (no LB) | Do / Re / Mi / Fa (lower tetrachord) |
| LB + A / B / X / Y | Sol / La / Ti / Do (upper tetrachord) |
| Left stick Y | Pitch offset, ±12 st (integrating) |
| Right stick Y | Fine bend, ±50 Hz (absolute) |
| Right stick X | Filter cutoff (integrating) |

With ARP (RB+LB toggle):

| Control | Function |
|---------|----------|
| A / B / X / Y | Chord: major / minor / 7th / diminished |
| Left stick X | Pattern: left=down, right=up, center=up-down |
| Left stick Y | Root transpose, ±12 st (integrating) |
| RT trigger rising edge | Start/stop arp running |
| Right stick X | Filter cutoff (integrating) |

Step rate follows the global tempo (16th notes).

### One-shot

Face buttons trigger retro SFX envelopes. Filter and delay bypassed.

| Control | Sound Effect |
|---------|--------------|
| A (no LB) | Laser (descending sweep) |
| B (no LB) | Explosion (low rumble) |
| X (no LB) | Power-up (ascending sweep) |
| Y (no LB) | Coin (high chirp) |
| LB + A | Siren |
| LB + B | Engine |
| LB + X | Jump |
| LB + Y | Warp |
| RT | Speed multiplier (0.3×-1.0×) |

## Building

Requires Docker (builds are containerized, no local ESP-IDF needed):

```bash
just build
just flash
just monitor
```

To enable verbose axis/dispatch diagnostic logging for debugging input-mapping issues:

```bash
just menuconfig
# Navigate to "Gamepad Synth" → enable "Enable verbose axis..."
just build && just flash && just monitor
```

## Architecture

- **Core 0**: Bluepad32 BTstack event loop (Bluetooth handling)
- **Core 1**: Two tasks:
  - **Control task** (50 Hz): reads gamepad state, updates synth parameters
  - **Audio render task** (continuous, priority 10): DDS phase accumulator with selectable waveform, writes 256-sample blocks to I2S DMA
- **Audio output**: 44.1 kHz, 16-bit stereo (mono duplicated) via MAX98357A I2S DAC
- **Piezo accents** (DRONE_HOLD only): two GPIO-driven LEDC square-wave voices on GPIO8/GPIO9 with a fixed 1.02 detune ratio, running in parallel to the DAC path
- **Waveforms**: Square, sawtooth, triangle, sine (256-entry lookup table), noise. Cycle via RB+Y
- **Filter**: Topology-preserving SVF (low-pass) with cutoff (40 Hz - 18 kHz) and resonance (Q 0.5 - 6.0). Unconditionally stable. Coefficients recomputed once per 256-sample block. Defensive NaN recovery guards against corner cases
- **LFO**: Block-rate triangle LFO (0.1 - 20 Hz) modulating filter cutoff or oscillator pitch. Up to ±2 octaves cutoff mod or ±1 octave pitch mod at full depth
- **Delay**: Circular-buffer delay line (0.5 s / 22050 samples, ~44 KB static RAM) with configurable delay time, feedback (up to 0.9), and wet/dry mix
