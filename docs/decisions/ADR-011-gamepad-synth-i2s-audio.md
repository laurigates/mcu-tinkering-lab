# ADR-011: Gamepad Synth I2S Audio Architecture

**Status**: accepted
**Date**: 2026-04-07
**Confidence**: 9/10

---

## Context

The gamepad-synth project (formerly esp32-ps4-speaker) produces audio via LEDC PWM driving a passive piezo buzzer. This approach only generates square waves at a single frequency — no waveform variety, no filtering, no effects. To implement a Korg Monotron-inspired synthesizer, we need real-time waveform synthesis with support for multiple waveforms, a resonant low-pass filter, LFO modulation, and delay effects.

The monorepo already has an I2S audio module in `nfc-scavenger-hunt/main/i2s_audio.c` that plays pre-recorded PCM buffers via a queue. However, the synth requires a fundamentally different model: continuous real-time sample generation where the audio task pulls samples from a render callback rather than playing queued buffers.

## Decision

Replace LEDC PWM with I2S streaming to a MAX98357A DAC. Implement a new audio engine with a pull-based render model and modular signal chain.

### Audio Engine Architecture

```
Core 0: Bluepad32 BTstack (blocks forever)
         │
         ▼ writes volatile gamepad_state_t
Core 1: Two tasks:
  ┌─────────────────────────────────────────┐
  │ Control Task (50 Hz, priority 5)        │
  │  reads gamepad → writes synth_params_t  │
  └─────────────────────────────────────────┘
  ┌─────────────────────────────────────────┐
  │ Audio Render Task (continuous, prio 10) │
  │  synth_render(block, 256) per DMA fill  │
  │  i2s_channel_write() blocks on DMA      │
  └─────────────────────────────────────────┘
```

### Signal Chain

```
Oscillator A ──┐
               ├──► Mixer ──► Filter (SVF) ──► Delay ──► I2S Output
Oscillator B ──┘         ▲              ▲
                         │              │
                        LFO ────────────┘
```

### Module Decomposition

| File | Responsibility |
|------|---------------|
| `i2s_output.c` | I2S channel init, DMA config, audio render task loop |
| `oscillator.c` | DDS phase-accumulator oscillator (saw, tri, sine, square, noise) |
| `filter.c` | State-variable filter with cutoff and resonance |
| `lfo.c` | Low-frequency oscillator for modulation |
| `delay.c` | Circular buffer delay with feedback |
| `synth_engine.c` | Owns all modules, exposes `synth_render()` and `synth_params_t` |

### Key Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Sample rate | 44100 Hz | Standard audio rate, good frequency range |
| Bit depth | 16-bit signed PCM | Sufficient for synthesis, matches MAX98357A |
| Block size | 256 samples | ~5.8 ms latency, good balance of CPU efficiency and responsiveness |
| DMA buffers | 4 x 256 frames | ~23 ms total pipeline latency |
| I2S mode | Stereo (Philips standard) | MAX98357A requires stereo I2S; mono duplicated to both channels |
| Delay max | 0.5 seconds (22050 samples) | ~44 KB RAM; fits comfortably in 512 KB SRAM |

### GPIO Pin Assignment

| GPIO | Function | Previous Use |
|------|----------|-------------|
| 5 | I2S BCLK | (unused) |
| 6 | I2S LRCLK (WS) | (unused) |
| 7 | I2S DOUT (DIN on DAC) | (unused) |
| 4 | (freed) | Was LEDC piezo output |
| 2 | Status LED | Unchanged |

### Memory Budget

| Component | RAM Usage |
|-----------|-----------|
| Sine lookup table (256 entries) | 512 bytes |
| Delay buffer (0.5s mono) | ~44 KB |
| I2S DMA buffers (4 x 256 x 2ch x 2B) | ~4 KB |
| Audio render task stack | 4 KB |
| Control task stack | 4 KB |
| Bluepad32 + BTstack | ~60 KB |
| ESP-IDF base | ~180 KB |
| **Total estimated** | **~300 KB of 512 KB** |

## Consequences

**Positive:**
- Real waveform synthesis: sawtooth, triangle, sine, square, noise
- Software filter produces the characteristic Monotron resonant sweep sound
- Delay effect with feedback enables Monotron Delay-style cosmic echoes
- Modular architecture — each DSP module is independent and testable
- MAX98357A + speaker provides actual audio output vs. tinny piezo

**Negative:**
- Requires additional hardware (MAX98357A + speaker) vs. simple piezo
- Real-time audio rendering is latency-sensitive — must complete 256 samples per ~5.8 ms block
- Delay buffer consumes ~44 KB of limited SRAM (no PSRAM configured)
- LEDC piezo fallback is dropped entirely

## Alternatives Considered

1. **Keep LEDC PWM alongside I2S** — Rejected. Fundamentally different code paths (`ledc_set_freq()` vs. sample-by-sample rendering). Would require `#ifdef` branches throughout, doubling the audio API surface for a vastly inferior fallback.

2. **Reuse nfc-scavenger-hunt's i2s_audio module** — Rejected. That module uses a queue-based push model designed for pre-recorded PCM playback. The synth needs a pull model where the audio task continuously requests samples from a render callback. Different enough to warrant a new module, though the I2S init code (same `driver/i2s_std.h` API) can be referenced.

3. **Internal DAC (ESP32 only, not S3)** — Not applicable. ESP32-S3 has no internal DAC. External I2S DAC is the standard approach.

4. **PDM output** — Rejected. Lower quality than I2S PCM, and MAX98357A supports standard I2S natively.

## Related Documents

- [PRD-008: Gamepad Synth](../requirements/PRD-008-gamepad-synth.md)
- NFC Scavenger Hunt I2S reference: `packages/games/nfc-scavenger-hunt/main/i2s_audio.c`
