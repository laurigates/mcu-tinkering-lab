# SG90 — 9g Micro Servo

**Manufacturer:** TowerPro
**Used in:** Robocar pan/tilt camera mount (via PCA9685 ch 6–7)

## Key Specs

Verified 2026-09 against both datasheet PDFs below (text and timing diagram)
and TowerPro's product pages.

| Parameter | Value |
|-----------|-------|
| Size class | Micro (9 g) |
| Weight | 9 g |
| Dimensions | 23 x 12.2 x 29 mm (TowerPro) |
| Across mounting tabs | 32.3 mm (TowerPro dimension E) |
| Rotation | ~180° per the datasheets; TowerPro support states 0–150° for its normal servos |
| Torque (4.8V) | 1.8 kg-cm |
| Speed (4.8V) | 0.12 sec/60° (analog), 0.1 sec/60° (TowerPro digital variant) |
| Voltage | 4.8–6.0V |
| Modulation | Analogue per the Handsontec guide; TowerPro also sells a **digital** SG90 with identical dimensions |
| PWM frequency | 50 Hz (20 ms period), from the datasheet timing diagram |
| Pulse | 1.5 ms centre, ~1–2 ms nominal; 500–2400 µs range |
| Dead band | 1 µs |
| Gear type | Plastic (POM) |

## Robocar note

`packages/robocar/unified` drives these from a PCA9685 shipped at **200 Hz**,
four times the specified frame rate, and the firmware's 2500 µs pan maximum
exceeds the 2400 µs range. See [servo selection](../servo-selection.md) for
the test and the options.

## Wiring

| Wire color | Function |
|------------|----------|
| Red | VCC (5V) |
| Brown/Black | GND |
| Orange | PWM signal |

## Datasheets & References

- **Datasheet PDF:** <https://handsontec.com/dataspecs/motor_fan/SG90-Servo.pdf>
- **Datasheet PDF (alt):** <http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf>
