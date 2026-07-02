# Balancebot

Two-wheel self-balancing robot on a **Seeed XIAO RP2350**: an MPU6050 IMU
feeds a 500 Hz complementary-filter + PID cascade on core 1, and RP2350
PIO state machines generate jitter-free step pulses for two DRV8825-driven
NEMA17 steppers. Tuning happens live over a USB CDC serial CLI; tuned
gains persist to flash.

The repo's first non-ESP32 project — built with the Raspberry Pi Pico SDK
(pinned 2.2.0) inside a container.

Design docs: [PRD-012](../../../docs/requirements/PRD-012-balancebot.md) ·
[ADR-018](../../../docs/decisions/ADR-018-pico-sdk-balancebot.md) ·
[WIRING.md](WIRING.md)

## Build

```bash
# From the repo root (containerized; no local toolchain needed)
just balancebot::build          # → build/balancebot.uf2

# Host-side unit tests (PID / filter / CRC, no hardware)
just balancebot::test
```

## Flash

```bash
just balancebot::flash          # picotool (reboots the board itself)
just balancebot::flash-uf2      # or: BOOTSEL drag-and-drop instructions
```

## Tune

```bash
just balancebot::monitor        # USB CDC serial console
```

```
help                 # command list
stat                 # state, angle, battery, loop time
stream on 50         # CSV telemetry at 50 Hz (plot with any serial plotter)
set kp 30            # live PID gain changes
arm | disarm         # start/stop balancing (or hold upright 3 s to auto-arm)
rate l 800           # open-loop motor bench test (disarmed only)
cal                  # gyro bias calibration (keep the robot still)
save                 # persist gains to flash (survives power cycle)
```

CSV columns: `t_ms,state,angle_deg,gyro_dps,setpoint_deg,rate_sps,loop_us`.

### Tuning procedure

1. Bench-check: `rate l 800` / `rate r 800` — both wheels spin smoothly,
   `rate l 0` stops. `stream on`, tilt the frame — angle tracks.
2. Set `ki`/`vkp`/`vki` to 0. Raise `kp` until the robot visibly fights
   the fall (fast oscillation = too high).
3. Raise `kd` to damp the oscillation.
4. Restore small `vkp`/`vki` so it holds station instead of wandering.
5. `save`.

## Safety model

- DRV8825 nENABLE has an external 10 kΩ pull-up: drivers are **disabled**
  while the MCU boots, resets, or sits in BOOTSEL.
- Tilt cutoff: |pitch| > 45° disables the drivers (FAULT, fast LED blink);
  clears after 2 s upright.
- Hardware watchdog (100 ms) fed only by the control loop, enabled at
  first arm — a wedged loop reboots into motors-off.
- Motors arm only via `arm` or the deliberate hold-upright-3 s gesture.

## Architecture

| Where | What |
|-------|------|
| Core 1 | 500 Hz loop: IMU burst read → complementary filter → angle PID (output = wheel **acceleration**) → integrate to step rate → PIO periods. Tilt cutoff, watchdog, flash saves. |
| Core 0 | USB CDC CLI, CSV telemetry, status LED, battery ADC. |
| PIO0 SM0/SM1 | STEP pulse trains, 30–20 000 steps/s, cycle-accurate, hold-last-rate between updates. |

Core 0 → core 1: lock-free command queue. Core 1 → core 0: seqlock
snapshot. The control loop never blocks and never prints.

`src/pin_config.h` is the pin-map source of truth; see
[WIRING.md](WIRING.md) for the full harness.
