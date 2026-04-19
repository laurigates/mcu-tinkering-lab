# Gamepad Synth — Wiring Guide

## Board

Any ESP32-S3 development board with USB-Serial-JTAG (e.g., Waveshare ESP32-S3-Zero, ESP32-S3-DevKitC-1). No PSRAM or WiFi required.

## Pin Assignments

| GPIO | Function | Direction | Component |
|------|----------|-----------|-----------|
| 5 | I2S BCLK | Output | MAX98357A BCLK |
| 6 | I2S LRCLK (WS) | Output | MAX98357A LRC |
| 7 | I2S DOUT | Output | MAX98357A DIN |
| 2 | Status LED | Output | LED anode (via resistor) |

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
|   GND  |──── GND ─────| GND    |   ┌──────┐
|        |              | + OUT ──|───| (+)  |
|        |              | - OUT ──|───| (-)  |
|        |              |________|   └──────┘
|        |
|  GPIO2 |──── 220R ──── LED (+)
|    GND |────────────── LED (-)
|        |
|    USB |──── USB-C cable (power + serial monitor)
|________|
```

## Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| MAX98357A | I2S 3W Class D amplifier breakout | Adafruit or generic; mono output. Connect GAIN to GND for 9 dB or leave floating for 12 dB. |
| Speaker | 4-8 ohm, 2-3W | Any small speaker; 40 mm recommended for breadboard projects |
| LED | Standard 3mm/5mm | Any color; 220-330 ohm series resistor |
| BLE controller | Xbox Series X/S (fw v5.15+), PS5 DualSense, Switch Pro, Joy-Cons, 8BitDo (BLE mode) | Connects via BLE (no wiring). PS4 DualShock 4, Xbox One, and Wii remotes are **not** compatible (require Bluetooth Classic / BR/EDR). |

## Notes

- **I2S audio**: The ESP32-S3 drives a MAX98357A I2S DAC at 44.1 kHz, 16-bit stereo (mono samples duplicated to both channels). The DAC handles amplification — no external amp needed.
- **GAIN pin**: The MAX98357A GAIN pin sets amplifier gain. Leave floating for 12 dB (default), connect to GND for 9 dB (quieter), or connect to VIN for 15 dB (louder).
- **Power**: USB-C provides both power and serial console via USB-Serial-JTAG. The MAX98357A draws power from 3V3.
- **Bluetooth pairing**: Put your BLE controller in pairing mode — Xbox: hold the pair button on top; PS5: hold Share + PS; Switch Pro: hold the sync button on top. No physical connection needed.
- **Mode cycle button**: View (Xbox) / Share (PS) / `-` (Switch) cycles through the four sound modes.
- **LED behavior**: Lights when a tone is playing; blinks N times when switching sound modes (1-4 blinks).
