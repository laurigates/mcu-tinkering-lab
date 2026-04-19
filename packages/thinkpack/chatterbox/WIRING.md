# Chatterbox Wiring

Target board: **ESP32-S3 SuperMini** (4 MB flash, octal PSRAM).

All pins listed below match `main/audio_engine.c` and `main/touch_driver.c`.
Update both files when moving a pin — the driver sources are the source of
truth.

## I2S TX — MAX98357A speaker amplifier

| Signal | ESP32-S3 GPIO | MAX98357A pin | Notes |
|--------|---------------|---------------|-------|
| BCLK   | 4             | BCLK          | Bit clock |
| LRCLK  | 5             | LRC           | Word-select (LRC) |
| DOUT   | 6             | DIN           | PCM data out |
| GND    | GND           | GND           | Common ground |
| Vcc    | 3V3 or 5V     | Vin           | 3V3 is fine for low volume; use 5V for full output |

The driver writes mono samples duplicated into stereo frames (standard for
the MAX98357A breakout).

## I2S RX — INMP441 MEMS microphone

| Signal | ESP32-S3 GPIO | INMP441 pin | Notes |
|--------|---------------|-------------|-------|
| BCLK   | 10            | SCK         | Bit clock |
| LRCLK  | 11            | WS          | Word-select |
| DIN    | 12            | SD          | Serial data (mic → ESP32) |
| GND    | GND           | GND         | Common ground |
| Vcc    | 3V3           | VDD         | 3V3 only — do not tie to 5V |
| L/R    | GND           | L/R         | Select left channel (mono capture) |

RX runs on I2S peripheral `I2S_NUM_1` to keep it independent from the TX
channel on `I2S_NUM_0`.

## Touch pad

| Signal   | ESP32-S3 GPIO | Notes |
|----------|---------------|-------|
| Touch    | 14            | Touch channel 14.  Connect to a small conductive pad; no external components required. |

The driver calibrates against the untouched baseline on boot; avoid
touching the pad during the first 250 ms after power-on.

## Status LED (optional)

| Signal | ESP32-S3 GPIO | Notes |
|--------|---------------|-------|
| LED    | 15            | Reserved for a single status LED.  Not yet wired in firmware. |

## Pin conflicts to avoid

- GPIO19/20 are the native USB D+/D- lines.  Do not repurpose them.
- GPIO35/36/37 are internal (octal PSRAM) on the SuperMini and unusable.
- GPIO0 is the strapping / boot-mode pin — leave floating or weakly
  pulled up.

## Power

The SuperMini draws < 200 mA during record/playback at moderate volume;
USB power is sufficient for bench testing.  For battery operation, use a
3.7 V Li-Po through the VBAT input and keep Vcc on the MAX98357A at 3V3 to
stay within the amp's headroom.
