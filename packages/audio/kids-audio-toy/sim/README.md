# Host simulator

Run the toy's logic on your Mac — **webcam in, speakers out** — without
flashing an ESP32. Great for quick iteration on the mapping/modulation logic.

```sh
just sim          # builds the shared lib, then runs the simulator
just sim --camera 1
```

ESC or `q` to quit.

## What's real vs faked

The simulator does **not** reimplement the firmware logic. It loads
`main/audio_core.c` — the exact C the firmware runs — as a shared library and
calls it via `ctypes`. Only the genuinely hardware-bound pieces are substituted:

| Firmware | Simulator |
|---|---|
| 3 pots (ADC) | brightness of the **left / middle / right** thirds of the frame → pitch / duration / interval |
| 555 timer (mod ADC) | **frame-to-frame motion** → vibrato (wave fast for more) |
| piezo (hardware PWM) | software-synthesized **square wave → speakers** |
| LED | on-screen indicator (green while a beep sounds) |

Because `audio_core` is shared, the pitch/duration/interval/modulation
behaviour is identical to the board. The one thing that can't carry over is
the *analog waveform shape* — hardware PWM vs a software square wave — which
doesn't matter for logic work.

## How it maps to the firmware

`audio_core` is the hardware-agnostic logic extracted from `main/main.c`:

- `audio_params_from_adc()` — the pot → pitch/duration/interval mapping
- `audio_update_modulation()` — the 555 EMA smoothing + clamp
- `audio_modulated_freq()` — folds modulation into the played pitch

The simulator's scheduler thread mirrors the firmware `audio_task` loop
exactly: read inputs → play one beep for `duration` → silence for `interval`
→ repeat. The same `audio_core` also backs the host unit tests
(`../test/test_audio_core.c`, run with `just test`).

## Notes & limits

- **macOS permissions**: first run prompts for **Camera** and **Microphone/
  speaker** access — grant them to your terminal in System Settings > Privacy
  & Security.
- The `555 idles mid-rail` assumption is baked into the firmware (a unipolar
  ADC reading maps to bipolar modulation), so the sim feeds mid-scale at rest
  → zero modulation, and motion pushes it positive. See `MOD_REST_ADC` /
  `MOTION_GAIN` in `toy_sim.py` to tune.
- `libaudio_core.dylib` is a build artifact (gitignored); `just sim` rebuilds
  it each run so edits to `audio_core.c` take effect immediately.
- This is a logic harness, not an emulator — it does not model FreeRTOS
  timing jitter, ADC noise, or the analog front-end.
