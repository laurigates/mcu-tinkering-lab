# Bench tests

Hardware-in-the-loop tests for the Kids Audio Toy. These need the board
flashed and running, with the piezo and pots connected — they verify real
acoustic output, not source logic, so they cannot run in CI.

## `bench_audio.py` — mic acceptance test

Records the piezo output from the default input device (the MacBook's
built-in mic works fine), detects each beep, and reports its **fundamental
pitch**, **duration**, and the **gap** to the next beep.

```sh
just bench-audio              # record 5 s
just bench-audio -d 10        # record 10 s
uv run test/bench_audio.py --list-devices   # pick a mic
```

The firmware plays a PWM square wave, so the FFT shows a fundamental plus odd
harmonics; the script picks the strongest in-band component, which is the
fundamental.

### Acceptance procedure

1. Flash and run the toy (`just flash-monitor`), piezo near the laptop mic.
2. Set the **pitch** pot to min / mid / max and run the test at each — confirm
   the reported pitch lands near **100 / ~1050 / 2000 Hz**.
3. Repeat the min/mid/max sweep on the **duration** (50-1000 ms) and
   **interval** (100-2000 ms) pots, checking the `duration (ms)` and
   `gap (ms)` columns.

### Notes & limits

- **macOS mic permission**: first run prompts for Microphone access — grant
  it to your terminal in System Settings > Privacy & Security > Microphone.
- Knob positions must be **known** to have an expected value: the firmware
  reads the pots live, so the test verifies measured-vs-intended for whatever
  you've dialed in.
- Very short beeps (≤30 ms) are filtered as noise; the envelope threshold and
  filters are constants at the top of the script if you need to tune them.
- The parameter ranges mirror `main/main.c` — keep them in sync if the
  firmware constants change.
