# Gamepad Synth — Wiring Guide

## Board

Any ESP32-S3 development board with USB-Serial-JTAG. No PSRAM, no WiFi, no SD card required. Verified with:

- Waveshare ESP32-S3-Zero (recommended — compact, castellated pads)
- Espressif ESP32-S3-DevKitC-1

The firmware runs at 240 MHz on one core (audio) while Bluepad32 uses the other (Bluetooth), so any ESP32-S3 variant with standard specs works.

## Pin Assignments

| GPIO | Function | Direction | Component |
|------|----------|-----------|-----------|
| 5 | I2S BCLK | Output | MAX98357A BCLK |
| 6 | I2S LRCLK (WS) | Output | MAX98357A LRC |
| 7 | I2S DOUT | Output | MAX98357A DIN |
| 2 | Status LED | Output | LED anode (via 220-330 Ω resistor) |

All three I2S pins are plain GPIOs on the ESP32-S3 (no special strapping roles), so any other trio works too — just update `I2S_BCLK_PIN` / `I2S_WS_PIN` / `I2S_DOUT_PIN` in `main/main.c`.

## Wiring Diagram

```
ESP32-S3                MAX98357A
 ________               ________
|        |              |        |
|  GPIO5 |──── BCLK ────| BCLK   |
|  GPIO6 |──── LRCLK ───| LRC    |
|  GPIO7 |──── DOUT ────| DIN    |
|        |              |        |
|   3V3  |──── VIN ─────| VIN    |    Speaker
|   GND  |──┬─── GND ───| GND    |   ┌──────┐
|        |  │           | + OUT ─|───│ (+)  │
|        |  │           | - OUT ─|───│ (-)  │
|        |  │           | GAIN   |   └──────┘
|        |  │           |        |
|        |  │           | SD  ───| (leave floating or tie to VIN for mono sum)
|        |  │           |________|
|        |  │
|  GPIO2 |──│── 220Ω ── LED (+)
|        |  │            │
|    GND |──┴─────────── LED (-)
|        |
|    USB |──── USB-C cable (power + serial monitor)
|________|
```

## Parts List

| Qty | Component | Notes |
|-----|-----------|-------|
| 1 | ESP32-S3 dev board | Waveshare ESP32-S3-Zero or ESP32-S3-DevKitC-1 |
| 1 | MAX98357A I2S amplifier breakout | Adafruit #3006 or generic — mono 3.2 W Class D |
| 1 | Speaker, 4-8 Ω, 2-3 W | 40 mm is a good breadboard size; panel mount for enclosures |
| 1 | LED (3mm or 5mm) | Any color |
| 1 | Resistor, 220-330 Ω | LED current limiter |
| — | Jumper wires, breadboard | Standard |

Optional:

- 100 µF electrolytic between MAX98357A VIN and GND if you hear USB-related whine or the amp resets under peak load
- 4.7 kΩ pull-up on the MAX98357A `SD` pin if you want to use the left-channel-only mode (tying SD directly to VIN gives mono sum; leaving it floating also gives mono sum on most breakouts)

## MAX98357A Gain Selection

The `GAIN` pin on the MAX98357A sets amplifier gain:

| GAIN connection | Gain | Use when |
|-----------------|------|----------|
| Tied to GND | 9 dB | Quiet listening, small speaker |
| Floating (default) | 12 dB | General use (start here) |
| 100 kΩ to GND | 6 dB | Very quiet — avoid if possible |
| Tied to VIN | 15 dB | Loud / outdoor |
| 100 kΩ to VIN | 3 dB | — |

Start with `GAIN` floating; adjust only if it's too quiet or clipping.

## Controllers

The ESP32-S3 only supports **BLE** (no Bluetooth Classic). Compatible controllers:

- Xbox Series X/S (firmware v5.15+; update via Xbox Accessories app on Windows)
- PS5 DualSense
- Nintendo Switch Pro
- Nintendo Joy-Cons (individually)
- 8BitDo controllers in BLE mode

**Not compatible** — these need Bluetooth Classic / BR/EDR:

- PS4 DualShock 4
- Xbox One (older models, pre-Series)
- Wii remotes

## Build Notes

1. **Power**: USB-C provides both 5 V (to VBUS/VIN on the board) and a serial console via USB-Serial-JTAG. No external PSU needed for low volumes; for a loud speaker, consider powering VIN from 5 V directly to avoid brown-outs.
2. **Bluetooth pairing**: Put the controller in pairing mode:
   - Xbox: hold the small pair button on top of the controller until the Xbox button flashes rapidly
   - PS5: hold Share + PS button until the light bar flashes
   - Switch Pro: hold the sync button on top of the controller
   After initial pairing, the controller auto-reconnects on boot (just press Home / Xbox / PS).
3. **Startup jingle**: The board plays a C5-E5-G5-C6 arpeggio on boot to confirm audio is working.
4. **Mode cycle**: View (Xbox) / Share (PS) / `-` (Switch) cycles through the **seven** sound modes.
5. **LED behavior**: Lights during active notes; blinks 1-7 times when switching modes to indicate the new mode number.

## Signal Chain

For reference, the audio path inside the ESP32-S3 is:

```
Oscillator A ──┐
               ├──► 50/50 mix ──► SVF filter ──► LFO mod ──► Delay line ──► I2S ──► MAX98357A ──► Speaker
Oscillator B ──┘
```

All DSP runs at 44.1 kHz, 16-bit, mono (duplicated to both I2S slots to satisfy the MAX98357A's stereo I2S expectation).
