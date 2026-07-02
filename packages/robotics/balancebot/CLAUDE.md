# CLAUDE.md - balancebot

Project-specific guidance for Claude Code. See repo-root `CLAUDE.md` for
monorepo-wide conventions, and the canonical design documents:

- [PRD-012: Balancebot](../../../docs/requirements/PRD-012-balancebot.md)
- [ADR-018: Pico SDK Toolchain and Balancebot Architecture](../../../docs/decisions/ADR-018-pico-sdk-balancebot.md)

## What this project is

A two-wheel self-balancing robot on a Seeed XIAO RP2350 with an MPU6050
IMU and two DRV8825/NEMA17 steppers. **This is a Pico SDK project, not
ESP-IDF** — no sdkconfig, no idf.py; plain CMake with
`PICO_BOARD=seeed_xiao_rp2350`, built in the `pico-sdk` compose service
(recipes from `tools/pico.just`).

## Architecture

```
core 1 (hard real-time, 500 Hz):  MPU6050 → complementary filter → angle PID
                                  (output = wheel ACCELERATION) → integrate →
                                  step rate → PIO0 SM0/SM1 step pulses
core 0 (soft):                    USB CDC CLI + CSV telemetry + LED + battery
```

- core0 → core1: `pico_util` queue of `balance_cmd_t` (see `src/ipc.h`)
- core1 → core0: seqlock'd `balance_snapshot_t`
- `src/pin_config.h` is the pin-map source of truth (mirror: `WIRING.md`)

## Build / test

```bash
just balancebot::build   # containerized, → build/balancebot.uf2
just balancebot::test    # host unit tests for pid/imu_filter/crc32
```

`pid.c`, `imu_filter.c`, `crc32.c` are **pure C with no SDK includes** so
the host tests can compile them — keep them that way.

## Don't

- Don't `printf` (or block on anything) in `balance_control.c`'s loop or
  anything it calls on core 1 — telemetry goes through the snapshot;
  printing belongs to core 0.
- Don't touch flash while state is RUN — a sector erase stalls XIP on
  both cores mid-balance. `CMD_SAVE` is already gated to IDLE/FAULT.
- Don't drive the DRV8825s from CPU-timed GPIO — step pulses come from
  the PIO program (`src/stepper.pio`); rate changes go through
  `stepper_set_rate()`, which owns the accel slew and direction-reversal
  rules.
- Don't reorder the boot sequence in `main.c` — nENABLE must be driven
  high before anything else initializes.
- Don't talk to the MPU6050 outside `mpu6050.c`, and don't call its
  functions from core 0 after `multicore_launch_core1()` — core 1 owns
  the I2C bus.
- Don't add `sdkconfig` / ESP-IDF patterns here; shared Pico recipes live
  in `tools/pico.just`, and the SDK pin lives in `Dockerfile.pico` +
  `.github/workflows/_ci-build-pico.yml` (bump both together).
- Don't rely on internal GPIO pull-downs — broken on the A2 stepping
  (RP2350-E9); use external pulls or actively driven lines.
- Don't move the param store to the last flash sector — UF2 downloads
  erase it (picotool RP2350-E10 workaround block).
