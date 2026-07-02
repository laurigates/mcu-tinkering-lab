# Balancebot Wiring

Board: Seeed XIAO RP2350. Pin-map source of truth: `src/pin_config.h`.

## Overview

```
                 2–3S LiPo (7.4–11.1 V)
                     │
        ┌────────────┼───────────────┐
        │            │               │
   VMOT (left    VMOT (right    5 V buck converter
   DRV8825)      DRV8825)           │
   + 100 µF      + 100 µF           └─→ XIAO 5V pin
                                        (NOT the battery pads — the
                                         onboard charger is ~370 mA
                                         and must not power motors)

   All grounds common: battery −, both DRV8825 GND, buck GND, XIAO GND.
```

## XIAO RP2350 connections

| XIAO pin | GPIO | Connects to |
|----------|------|-------------|
| D4 | GPIO6 | MPU6050 SDA |
| D5 | GPIO7 | MPU6050 SCL |
| D3 | GPIO5 | MPU6050 INT |
| D8 | GPIO2 | left DRV8825 STEP |
| D10 | GPIO3 | right DRV8825 STEP |
| D9 | GPIO4 | left DRV8825 DIR |
| D6 | GPIO0 | right DRV8825 DIR |
| D7 | GPIO1 | both DRV8825 nENABLE **+ 10 kΩ pull-up to 3V3** |
| 3V3 | — | MPU6050 VCC, DRV8825 logic (see below), pull-up |
| GND | — | common ground |
| 5V | — | from buck converter output |
| D0–D2 | GPIO26–28 | spare (ADC-capable) |

The GY-521 MPU6050 breakout has an onboard 3.3 V regulator and works fed
from 3V3 directly (bypassing the regulator) — INT is push-pull 3.3 V,
safe for the RP2350.

## DRV8825 carriers (each)

| DRV8825 pin | Connect to |
|-------------|-----------|
| VMOT / GND MOT | battery + / − (100 µF electrolytic across, close to the pins) |
| B2 B1 A1 A2 | stepper coils (one coil pair per letter; swap one pair to reverse a motor in hardware — firmware also mirrors the right motor) |
| GND LOGIC | common ground |
| STEP / DIR | XIAO, per table above |
| nENABLE | shared GPIO1 line with 10 kΩ pull-up to 3V3 |
| nSLEEP + nRESET | tie both to 3V3 (or bridge nSLEEP–nFAULT on carriers with that route) |
| M0 M1 M2 | 1/8 microstep strap — M0=HIGH, M1=HIGH, M2=LOW |

**Why the nENABLE pull-up matters**: the DRV8825 EN input has an internal
pulldown, i.e. a floating nENABLE means *enabled*. During boot, reset and
BOOTSEL the XIAO's GPIOs are hi-Z — without the pull-up the motors would
be live while nothing is in control. Verify strapping against the TI
DRV8825 datasheet table when assembling.

### Current limit (Vref)

Set per driver, motors connected, before first `arm`:

1. Power VMOT (logic too), drivers disabled (default).
2. Measure Vref between the trim pot wiper and GND.
3. DRV8825: `I_limit = 2 × Vref`. For a typical 1.5 A/phase NEMA17 at 1/8
   microstep, start conservative: Vref ≈ 0.4 V → 0.8 A/phase. Increase
   only if steps are skipped under load (with cooling as needed).

## MPU6050 mounting

- Rigidly mounted (no foam) at **axle height**, near the wheel axis.
- **X axis pointing forward**, Z up — the filter computes pitch as
  `atan2(accel_x, accel_z)` and integrates gyro Y.
- If the robot drives *into* the fall on first arm, flip `IMU_PITCH_SIGN`
  in `src/config.h` (or rotate the sensor 180°).

## Bring-up checklist

1. No motor power: flash, `just balancebot::monitor`, `stat` shows
   `imu: ok`; `stream on`, tilt → angle tracks, level ≈ 0°.
2. `cal` with the robot still; verify no drift over a minute.
3. Motor power, wheels off the ground: `rate l 400`, `rate r 400` — both
   spin the same direction (as the robot would drive); reverse cleanly
   with `rate l -400`.
4. `disarm`, robot upright on the ground, `arm` (or hold upright 3 s) —
   tune per the README procedure.
