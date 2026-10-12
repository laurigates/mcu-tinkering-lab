# ADR-026: Balancebot LQR With a Live, Bumpless PID/LQR Switch

**Status**: Accepted
**Date**: 2026-10-12
**Confidence**: 6/10 (host-tested against scipy; not yet run on hardware, and the default model constants are placeholders)
**Extends**: [ADR-018](ADR-018-pico-sdk-balancebot.md)

---

## Context

The balancebot ([PRD-012](../requirements/PRD-012-balancebot.md)) balances with
the controller [ADR-018](ADR-018-pico-sdk-balancebot.md) chose. An angle PID
on core 1 outputs wheel **acceleration** at 500 Hz, the loop integrates that
into a step rate, and a 100 Hz velocity PI nudges the angle setpoint so the
robot holds station. That cascade has five gains with no model behind them,
and it controls wheel position only indirectly, through the velocity loop's
lean offset.

A linear-quadratic regulator on a wheeled-inverted-pendulum model computes all
four state gains (angle, angle rate, position, velocity) from one physical
constant and a set of weights. That gives tuning a model to reason about.
Wheel position becomes a state the controller holds, rather than drift that a
second loop corrects. The PID still has to stay usable. It is the controller
that bring-up settles `IMU_PITCH_SIGN` against, and the fallback when LQR
cannot be used. A comparison is also only fair when it is made on the same
robot within a few seconds.

Two constraints shape the design. Core 1 must stay hard real-time: no
`printf`, no blocking, and float math only, because the Cortex-M33 FPU is single
precision. And there are no encoders. Wheel position is known only by
integrating the commanded step rate.

## Decision

### Model and solve

- **Model**: a wheeled inverted pendulum whose input is the wheel acceleration
  `u = x_ddot` (m/s²), the quantity ADR-018's PID already commands, so the
  stepper path does not change. The state is `[theta, theta_dot, x, x_dot]`
  in SI units, with `theta_ddot = (g·theta − u) / L_eff`. The physical
  parameters are `l_eff_m` (equivalent pendulum length), `wheel_diam_m` and
  `steps_per_rev`.
- **Discretisation**: exact zero-order hold at dt = 1/500 s. It is closed form,
  using `cosh`/`sinh` of `w = sqrt(g / L_eff)`.
- **Riccati solve**: the structure-preserving doubling algorithm in double
  precision, with a bounded iteration count and 4×4 inverses. It returns
  `K[4]`, the closed-loop spectral radius and the Riccati residual. It fails,
  and leaves any previous K in place, on invalid input, non-convergence, a
  non-finite result, or a closed loop that is not strictly stable
  (`rho >= 1`, for example `q_x = 0`).
- **Where it runs**: core 0 only, at boot and on `set` of any model or weight
  parameter. The solve never runs in the 500 Hz loop. It needs about 2.1 KB
  of stack (measured with `-fstack-usage`), so core 0's stack grows to the
  whole 4 KB SCRATCH_Y bank (`PICO_STACK_SIZE=0x1000`). Core 1 keeps 2 KB.
- **K is not persisted.** The record stores the model and weights, and K is
  re-solved from them, so a stored K can never disagree with its model.

### Loop integration

- Core 1 integrates `wheel_pos_steps` from the commanded `wheel_rate_sps`. It
  is reset when the robot enters RUN.
- The LQR term is `u = -K·[theta, theta_dot, x − x_ref, x_dot]`, scaled to
  steps/s² by `steps_per_rev / (π·d)` and clamped to `ANGLE_PID_OUT_MAX`. It
  is then integrated exactly like the PID output. The core-1 side is float
  only, and `-Wdouble-promotion` is on for the loop's sources.
- **Sign**: `theta` is the negated pitch in radians, so that for a small
  positive pitch the LQR and the PID command an acceleration of the same sign.
  LQR therefore inherits whichever `IMU_PITCH_SIGN` bring-up settles on, and a
  host test pins the invariant.

### Handoff and switching

- `CMD_SET_LQR_GAINS` carries all four gains in one queue item, so core 1
  never runs a tick on a half-updated K.
- `CMD_SET_CONTROLLER` switches the law, applied at a tick boundary:
  - **PID → LQR**: `x_ref` is set to the current position, so the position
    error starts at 0. `wheel_rate_sps` carries over.
  - **LQR → PID**: the velocity-PI integral restarts from 0. The angle-PID
    integral is back-calculated (`pid_bumpless_integral()`, loaded through
    the clamping `pid_set_integral()`) so that the first PID output equals the
    last LQR output.
- **Fail closed**:
  - Core 0 refuses `ctrl lqr`, with the reason, while it holds no valid K.
  - Core 1 refuses the switch while it holds no K.
  - Core 1 always boots on the PID.
  - A saved LQR mode whose boot solve fails falls back to PID, and both the
    boot banner and `stat` report it.
- A model or weight `set` whose solve fails is **rejected**: neither the
  parameter nor K changes. The new K and the parameter are queued
  all-or-nothing, so a full queue also changes neither.

### Parameters and telemetry

- `PARAM_VERSION` 2 adds `ctrl_mode`, `l_eff_m`, `wheel_diam_m`,
  `steps_per_rev`, `q_theta`, `q_dtheta`, `q_x`, `q_dx` and `r`. A v1 record
  migrates: its PID gains are kept and every new field takes its default.
  The record framing and the migration moved into the pure `param_record.c`
  so both can be host-tested against a hand-built v1 blob.
- The CSV only appends columns: `…,loop_us,ctrl,pos_m`.

## Consequences

**Positive**

- One measurable constant (`L_eff`, from a swing test: hang the robot by its
  axle, time small swings, `L_eff = g·(T/2π)²`) plus weights in physical units
  replace five model-free gains.
- PID and LQR can be compared live, on the same robot within seconds, without
  a reflash.
- Wheel position is regulated directly, not only through a lean offset.
- The solver, the sign mapping, the bumpless transfer and the record migration
  are host-tested. The solver is checked against scipy's `solve_discrete_are`.

**Negative**

- Without encoders, `x` is the *commanded* travel. Missed steps and wheel slip
  never show up in it, so the position the LQR holds can drift from the real
  one. The stepper's acceleration slew in `stepper_set_rate()` also sits
  between the command and the wheel, and the model does not include it.
- The LQR → PID transfer is bumpless only when the angle PID has `ki ≠ 0`. With
  the default `ki = 0` the integral cannot absorb the difference. The output
  then steps to the PID's proportional and derivative value, while the wheel
  rate (the integral of that output) stays continuous. The transfer also falls
  short when the required integral exceeds `ANGLE_INTEGRAL_MAX`.
- The model treats `u` as the wheel's ground acceleration and ignores the
  motor's reaction torque on the body. The stepper command is relative to the
  body, so the true ground acceleration is `u + r·θ̈`. A verification
  simulation with assumed masses put the real lean-growth term at about half
  the model's and the input gain at 0.7–0.8 of it. The default K still
  stabilised all three cases (closed-loop ρ ≈ 0.9985). Only the robot can
  confirm the margin. If it falls short, use a full two-body model instead of
  the pendulum one.
- The default `L_eff` (0.12 m) and wheel diameter (0.09 m) are placeholders
  until the robot is built and measured.
- The command queue grows from 16 to 32 items so that `load` can send every
  parameter, K and the controller in one burst.

## Alternatives Considered

- **Solve on core 1 at a low rate**: rejected. The solve needs double precision
  (the M33 FPU is single precision, so double goes through SDK routines)
  and about 2 KB of stack, which would cost core 1 its
  deadline.
- **Persist K**: rejected. A stored K can disagree with the stored model after
  a migration or a hand edit. Re-solving at boot costs milliseconds.
- **Send K one gain per command**: rejected. A tick between two items would run
  on a mixed K.
- **Accept `q_x = 0` (no position hold)**: rejected for now. The closed-loop
  spectral radius is then exactly 1, which the solver treats as not stable.
  This is an open question, not a settled one.

## Related

- [ADR-018](ADR-018-pico-sdk-balancebot.md): the acceleration-output framing
  this extends
- [PRD-012](../requirements/PRD-012-balancebot.md): the balancebot requirements
- `packages/robotics/balancebot/src/lqr.{c,h}`, `tests/host/test_lqr.c`,
  `tests/host/lqr_reference.py`: the solver and its scipy reference
