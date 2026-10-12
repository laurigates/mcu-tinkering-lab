# Balancebot

Two-wheel self-balancing robot on a **Seeed XIAO RP2350**: an MPU6050 IMU
feeds a 500 Hz complementary filter and either a PID cascade or an LQR
(switchable live) on core 1, and RP2350
PIO state machines generate jitter-free step pulses for two DRV8825-driven
NEMA17 steppers. Tuning happens live over a USB CDC serial CLI; tuned
gains persist to flash.

The repo's first non-ESP32 project — built with the Raspberry Pi Pico SDK
(pinned 2.2.0) inside a container.

Design docs: [PRD-012](../../../docs/requirements/PRD-012-balancebot.md) ·
[ADR-018](../../../docs/decisions/ADR-018-pico-sdk-balancebot.md) ·
[ADR-026](../../../docs/decisions/ADR-026-balancebot-lqr-live-switch.md) ·
[WIRING.md](WIRING.md)

## Printable build guide

[`docs/build-guide.typ`](docs/build-guide.typ) → [PDF](docs/build-guide.pdf):
the whole build at the bench — bill of materials, wiring reference, power,
assembly, Vref, flashing, first boot and calibration, functional checkout,
tuning, troubleshooting.

Regenerate it after editing — GPIO numbers come from `src/pin_config.h` via
`docs/auto/pin_defs.typ`, so a pin moved in the header changes what it prints:

```bash
just balancebot::build-guide
```

The recipe pins the Typst version and the two determinism flags that
`.github/workflows/build-guide-check.yml` recompiles with, so a hand-run
`typst compile` with different flags will read as drift in CI.

## Build

```bash
# From the repo root (containerized; no local toolchain needed)
just balancebot::build          # → build/balancebot.uf2

# Host-side unit tests (PID, filter, CRC, LQR, param record; no hardware)
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
set l_eff_m 0.14     # LQR model/weight change: re-solves K, prints K, rho, residual
ctrl lqr             # switch to LQR live (bumpless); `ctrl pid` switches back
arm | disarm         # start/stop balancing (or hold upright 3 s to auto-arm)
rate l 800           # open-loop motor bench test (disarmed only)
cal                  # gyro bias calibration (keep the robot still)
save                 # persist gains to flash (survives power cycle)
```

CSV columns: `t_ms,state,angle_deg,gyro_dps,setpoint_deg,rate_sps,loop_us,ctrl,pos_m`
(`ctrl` 0 = PID, 1 = LQR; `pos_m` = commanded wheel travel since arming).
Columns are only ever appended, so plots that index by position keep working.

### Tuning procedure

1. Bench-check: `rate l 800` / `rate r 800` — both wheels spin smoothly,
   `rate l 0` stops. `stream on`, tilt the frame — angle tracks.
2. Set `ki`/`vkp`/`vki` to 0. Raise `kp` until the robot visibly fights
   the fall (fast oscillation = too high).
3. Raise `kd` to damp the oscillation.
4. Restore small `vkp`/`vki` so it holds station instead of wandering.
5. `save`.

### LQR

The LQR ([ADR-026](../../../docs/decisions/ADR-026-balancebot-lqr-live-switch.md))
uses a wheeled-inverted-pendulum model whose input is the same wheel
acceleration the PID commands. Core 0 solves the gains in double precision at
boot and whenever one of its parameters changes. Core 1 only applies them.

| Param | Meaning | Default |
|-------|---------|---------|
| `l_eff_m` | Equivalent pendulum length (m) | 0.12 (placeholder) |
| `wheel_diam_m` | Wheel diameter (m) | 0.09 (placeholder) |
| `steps_per_rev` | Microsteps per wheel revolution | 1600 |
| `q_theta` `q_dtheta` `q_x` `q_dx` | State weights | 400, 1, 25, 4 |
| `r` | Input weight | 0.04 |

1. Get the PID balancing first. LQR inherits `IMU_PITCH_SIGN` from it.
2. Measure `l_eff_m` with the swing test. Hang the robot by its axle, time
   10 small swings to get the period `T`, then
   `L_eff = 9.81 * (T / 2π)^2`. Measure `wheel_diam_m` too.
3. `set l_eff_m <value>`. Every model or weight change re-solves K and
   prints K, the closed-loop spectral radius `rho` (it must be below 1) and
   the residual. A change whose solve fails is rejected and leaves both the
   parameter and K unchanged.
4. While balancing on the PID, `ctrl lqr`. The robot holds the position it
   switched at. `ctrl pid` switches back. The switch back is bumpless only
   with `ki` ≠ 0.
5. `save` stores the mode and the model. K is not stored: it is re-solved at
   boot. If the solve fails at boot, the robot runs the PID and says so.

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
| Core 1 | 500 Hz loop: IMU burst read → complementary filter → angle PID or LQR (output = wheel **acceleration**) → integrate to step rate → PIO periods. Tilt cutoff, watchdog, flash saves. |
| Core 0 | USB CDC CLI, CSV telemetry, status LED, battery ADC, LQR gain solve. |
| PIO0 SM0/SM1 | STEP pulse trains, 30–20 000 steps/s, cycle-accurate, hold-last-rate between updates. |

Core 0 → core 1: lock-free command queue. Core 1 → core 0: seqlock
snapshot. The control loop never blocks and never prints.

`src/pin_config.h` is the pin-map source of truth; see
[WIRING.md](WIRING.md) for the full harness.
