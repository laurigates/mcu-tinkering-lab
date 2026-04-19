# thinkpack-glowbug

Light + motion box for the ThinkPack modular toy mesh.  Runs on an ESP32-S3
SuperMini and drives a WS2812B LED ring with sensor-reactive animations.
Part of ThinkPack Phase 2 — see [issue #195](https://github.com/laurigates/mcu-tinkering-lab/issues/195).

## Hardware

| Component      | Part             | Notes                                        |
|----------------|------------------|----------------------------------------------|
| MCU            | ESP32-S3 SuperMini | USB-C, USB-Serial-JTAG built-in            |
| LED ring       | WS2812B × 12     | GPIO4 data; 5V external power               |
| IMU            | MPU6050          | I2C: SDA GPIO6, SCL GPIO7                   |
| Light sensor   | LDR              | GPIO1 (ADC1_CH0), 10kΩ pulldown to GND      |
| Button         | Momentary N.O.   | GPIO9 → GND; internal pull-up enabled       |

## Pinout

| GPIO | Signal        | Direction |
|------|---------------|-----------|
| 1    | LDR (ADC)     | Input     |
| 4    | WS2812B Data  | Output    |
| 6    | I2C SDA       | Bidir     |
| 7    | I2C SCL       | Bidir     |
| 9    | Button        | Input     |

See [WIRING.md](WIRING.md) for the full ASCII schematic.

## Quick Start

```bash
# Build (Docker required — no local ESP-IDF needed)
just thinkpack-glowbug::build

# Flash (auto-detects ESP32-S3 USB-Serial-JTAG port)
just thinkpack-glowbug::flash

# Monitor serial output
just thinkpack-glowbug::monitor

# Flash then monitor in one step
just thinkpack-glowbug::flash-monitor

# Override port manually
PORT=/dev/ttyUSB0 just thinkpack-glowbug::flash
```

## Expected Standalone Behaviour

When no peers are present the glowbug runs sensor-driven animations:

| Condition                      | Animation         |
|-------------------------------|-------------------|
| Dark (LDR < 800)              | Amber nightlight  |
| Bright (LDR > 3200)           | Rainbow chase     |
| Shake detected (Δ > 1.5 g)    | White sparkle burst (500 ms) |
| Otherwise                     | Breathe — hue follows tilt angle |

Press the button to cycle a forced-mode override:
**BREATHE → RAINBOW → SPARKLE → AUTO** → repeat.

All LEDs are capped at ≤ 60% brightness and animations change no faster than
every 100 ms (toddler safety, PRD FR-T21).

## Expected Group Behaviour

When one or more peers are discovered via the ThinkPack mesh:

| Event              | Response                                        |
|--------------------|-------------------------------------------------|
| Peer discovered    | 150 ms full-white hello flash                   |
| SYNC_PULSE         | 200 ms white flash phase-locked to leader clock |
| CMD 0x10 (LED_PATTERN) | Switch to commanded colour / mode          |
| Leader lost        | Reverts to standalone sensor-driven mode        |

## Troubleshooting

**LED ring not lighting up**
- Confirm 5V external supply connected to ring power rail.
- Check GPIO4 data line (3.3V logic is accepted by WS2812B).

**IMU not detected**
- Verify SDA/SCL wiring and pull-up resistors (internal pull-ups are enabled
  in firmware but external 4.7kΩ resistors improve reliability).
- Check I2C address — AD0 pin must be tied LOW for address 0x68.

**Build fails with "managed_components not found"**
- Run `just thinkpack-glowbug::build` (not bare `idf.py build`); the
  container fetches managed components on first build.

**No peers visible**
- Ensure all boxes are on the same WiFi channel (channel 1 by default).
- Confirm ESP-NOW is not blocked by a nearby AP on the same channel.
