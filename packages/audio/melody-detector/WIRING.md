# Wiring — melody-detector

Target: **Seeed Studio XIAO ESP32-S3 Sense**.

The OV2640 camera lives on the Sense expansion board and uses internal GPIOs
that are not routed to the user headers, so the D0..D10 pins are all free
for the rest of the project.

USB-Serial-JTAG (USB-C) is the console, which frees `U0TX` (GPIO43) and
`U0RX` (GPIO44) for use as I2S.

## Pin map

| Function | GPIO | XIAO label | Direction | Notes |
|---|---|---|---|---|
| Capture button | 2 | D1 | input, internal pull-up | active LOW; 30 ms debounce |
| Status LED | 21 | onboard user LED | output | active LOW |
| I2S BCLK | 6 | D5 | output | → MAX98357A `BCLK` |
| I2S LRC (WS) | 43 | D6 (U0TX-pad) | output | → MAX98357A `LRC` |
| I2S DIN | 44 | D7 (U0RX-pad) | output | → MAX98357A `DIN` |

Free for future use (Phase 2+): GPIO1 (D0), GPIO3 (D2), GPIO4 (D3), GPIO5
(D4), GPIO7 (D8), GPIO8 (D9), GPIO9 (D10).

## MAX98357A connections

| MAX98357A pin | Wire to |
|---|---|
| `VIN` | 5V (XIAO `5V` pin if powered via USB, or external 5V) |
| `GND` | GND |
| `DIN` | GPIO44 (D7) |
| `BCLK` | GPIO6 (D5) |
| `LRC` | GPIO43 (D6) |
| `GAIN` | leave floating (9 dB) or tie to GND for 12 dB |
| `SD` | leave floating (left + right channel mix; mono output) |
| Speaker `+` / `-` | 3 W 4–8 Ω speaker |

## Capture button

Standard tactile button between **GPIO2 (D1)** and **GND**. The internal
pull-up is enabled in software; no external resistor required.

## Status LED

The onboard yellow user LED on **GPIO21** is reused as the status indicator.
No external wiring required.

State patterns:

| State | Pattern |
|---|---|
| Idle | slow heartbeat (~1 Hz) |
| Capturing | fast blink (~5 Hz) |
| Processing | solid on |
| Playing | double-pulse heartbeat |
| Error | rapid blink (~8 Hz) |

## Power

USB-C from the XIAO. The MAX98357A draws up to ~700 mA at peak speaker
output, which is comfortably within USB-C bus power. A USB power supply
rated for at least 1 A is recommended.

## Reference

- Seeed Studio XIAO ESP32-S3 Sense schematic / pinout
- MAX98357A datasheet (Adafruit / Analog Devices)
- `nfc-scavenger-hunt` for the validated XIAO-S3 + MAX98357A pinout
