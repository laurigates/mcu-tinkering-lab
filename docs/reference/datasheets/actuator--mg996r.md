# MG996R — Standard-Size Metal Gear Servo

**Manufacturer:** TowerPro (widely cloned)
**Used in:** Not used. Evaluated 2026-09 as an SG90 replacement for the robocar
pan/tilt head and rejected on size. See [servo selection](../servo-selection.md).

## Key Specs

From the Handsontec user guide (EMH-1056):

| Parameter | Value |
|-----------|-------|
| Size class | Standard (not micro) |
| Weight | 55 g |
| Dimensions | 40.7 x 19.7 x 42.9 mm approx. |
| Modulation | Digital (per the guide's description) |
| Rotation | ~120° (60° each way) per the guide's intro; its Arduino section says 180°, so treat range as unverified |
| Stall torque | 9.4 kgf·cm (4.8 V), 11 kgf·cm (6 V) |
| Speed | 0.17 s/60° (4.8 V), 0.14 s/60° (6 V) |
| Voltage | 4.8–7.2 V |
| Running current | 500 mA |
| Stall current | 2.5 A (6 V) |
| Dead band | 5 µs |
| Pulse | 1.5 ms centre, ~1 ms and ~2 ms at the extremes |
| PWM frame rate | **Not stated in the datasheet** |
| Gears | Metal, double ball bearing |

## Mechanical

From the dimensioned drawing on page 2 of the guide (mm):

| Measurement | Value |
|-------------|-------|
| Case length | 40.3 |
| Length across mounting tabs | 53.6 |
| Case width | 20 |
| Case height | 36.6 |
| Base to mounting tabs | 26.6 |
| Overall height including horn | 47.6 |

Ships with 4 fixing screws with rubber grommets and an M4 horn screw.

## Datasheets & References

- **Datasheet PDF (Handsontec):** <https://www.handsontec.com/dataspecs/motor_fan/MG996R.pdf>
- **Datasheet PDF (alt):** <https://www.electronicoscaldas.com/datasheet/MG996R_Tower-Pro.pdf>
