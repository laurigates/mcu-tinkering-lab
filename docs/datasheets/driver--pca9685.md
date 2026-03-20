# PCA9685 — 16-Channel 12-bit PWM/Servo Driver

**Manufacturer:** NXP Semiconductors
**Used in:** Robocar main controller (RGB LED control on ch 0–5, pan/tilt servos on ch 6–7)

## Key Specs

| Parameter | Value |
|-----------|-------|
| Channels | 16 independent PWM outputs |
| Resolution | 12-bit (4096 steps) |
| PWM frequency | 24 Hz – 1526 Hz (adjustable) |
| Interface | I2C (Fm+ up to 1 MHz) |
| I2C address | 0x40 (default), up to 62 devices via A0–A5 |
| Supply | 2.3–5.5V, I/O 5.5V tolerant |
| Output drive | 25 mA sink, 10 mA source (totem pole) |
| Oscillator | Internal 25 MHz, external clock input up to 50 MHz |

## Datasheets & References

- **Datasheet PDF (NXP, Rev. 4):** <https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf>
- **Adafruit tutorial:** <https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf>
