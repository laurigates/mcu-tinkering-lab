# Gamepad Synth — Wiring Guide

## Board

Any ESP32-S3 development board with USB-Serial-JTAG (e.g., Waveshare ESP32-S3-Zero, ESP32-S3-DevKitC-1). No PSRAM or WiFi required.

## Pin Assignments

| GPIO | Function | Direction | Component |
|------|----------|-----------|-----------|
| 4 | LEDC PWM (piezo) | Output | Piezo buzzer (+) |
| 2 | Status LED | Output | LED anode (via resistor) |

## Wiring Diagram

```
ESP32-S3
 ________
|        |
|  GPIO4 |──────── Piezo (+)
|        |              |
|        |         Piezo (-)
|        |              |
|    GND |──────────────┘
|        |
|  GPIO2 |──── 220R ──── LED (+)
|        |                    |
|    GND |────────────── LED (-)
|        |
|    USB |──── USB-C cable (power + serial monitor)
|________|
```

## Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| Piezo buzzer | Passive, 3.3V compatible | Must be passive (not active) for variable frequency |
| LED | Standard 3mm/5mm | Any color; 220-330 ohm series resistor |
| BLE controller | Xbox Series X/S (fw v5.15+), PS5 DualSense, etc. | Connects via BLE (no wiring) |

## Notes

- **Passive vs active buzzer**: This project drives the piezo with PWM at variable frequencies (100-2000 Hz). An active buzzer has a built-in oscillator and will not produce different tones — use a passive buzzer.
- **Power**: USB-C provides both power and serial console via USB-Serial-JTAG.
- **Bluetooth pairing**: Put your BLE controller in pairing mode (Xbox: hold pair button on top; PS5: hold Share + PS). No physical connection needed.
- **LED behavior**: Lights when a tone is playing; blinks N times when switching sound modes (1-4 blinks).
