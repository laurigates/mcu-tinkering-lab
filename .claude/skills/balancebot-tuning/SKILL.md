---
name: balancebot-tuning
description: "Bring up and tune the balancebot (packages/robotics/balancebot) from telemetry: sign checks, capture, `analyze`, one live change, repeat. Use when the robot will not balance, jitters, rocks, drifts, runs away, falls after a while, when `stat` looks wrong (tick 0, inverted angle), or when choosing r, alpha or trim."
user-invocable: true
argument-hint: "[symptom | capture.csv]"
allowed-tools: Bash(just:*), Bash(python3:*), Bash(uv:*), Bash(ls:*), Read, Grep, Glob
---

# Balancebot Tuning: Measure, Change One Thing, Measure Again

The method that took the rev1 robot from falling over in 15 s to holding its
spot within about 1 cm (2026-11-22/23). The procedure and the worked example
are in the project README, "Tuning loop" and "Choosing r"; the hardware checks
are in `BRINGUP.md`. This skill is the decision table and the traps.

## Before any tuning

1. `just balancebot::stat`. Read `fw:`, `core 1:`, `imu:`, `gyro cal:`,
   `trim:`, `maxrate:`, `model:` and `loop:`. A rising tick is the only proof
   core 1 is running.
2. The signs, `BRINGUP.md` §4: tilting toward FWD reads positive, and
   `rate -400` on both wheels rolls toward FWD. Both, or it drives away from
   every fall.
3. `just balancebot::cli get` for the saved values. A `set` that was never
   saved is gone after a reboot: on 2026-11-23 an `r` change "had no effect"
   because the robot had rebooted since.

## The loop

1. The robot balances on its own on the floor (never with the wheels in the
   air; see the traps). Run `just balancebot::capture 20`.
2. `just balancebot::analyze <csv> [l_eff_m] [maxrate]`.
3. Change **one** setting live (`just balancebot::cli "set <name> <v>"`):
   LQR settings, `alpha` and `trim` all apply while armed.
4. Capture again and compare the same numbers.
   For a push, a fall or anything the cable would spoil, use the run log
   instead: unplug USB, run, then `just balancebot::log-dump` and `analyze`
   the CSV it writes (README "Untethered run log", ADR-037). `log` says
   whether the last run saved. Flash keeps the newest six runs (`log list`,
   `log-dump n=<run>`, #1277); before that it kept one, and runs 5 and 6
   were lost on 2026-11-24. For a fall, `just balancebot::timeline <csv>`
   prints the rows around it; `terms`, `accel` and `push-sim` go further
   (README "Untethered run log").
5. `just balancebot::cli disarm save` when it holds; `save` reports `OK saved`.

| `analyze` shows | Cause | Change |
|---|---|---|
| a large share at the acceleration clamp, hard jitter | the LQR asks more than MAX_ACCEL_SPS2 allows | raise `r`; `just balancebot::lqr-sweep <l_eff> <wheel>` shows the lean at which K hits the clamp |
| a rock near √(g/L) (about 1.1 Hz for 0.21 m) | alpha too low: the accelerometer reads the robot's own acceleration as tilt | raise `alpha` (τ = dt·α/(1−α); 0.996 is 0.5 s at 500 Hz) |
| position held, mean angle away from 0 | trim | add the mean angle to `trim` |
| position wanders | position hold too weak (high r) | raise `q_x` / `q_dx` |
| at maxrate | speed limit | raise `maxrate` only if the motors keep up: 5000 and 6000 ran clean on 3S, 8000 stalled the rev1 steppers (buzz) |

Judge the robot by wheel speed and position range as well as the angle. The
angle passes through the filter, so a filter change alters what it reads; the
speed and position do not, and they show whether the robot really moved less.

## Reading a fall

Every push-test fall so far (#1262: `maxrate` 4000, 5000, 6000) had the same
cause: the wheels reached `maxrate` and the lean kept growing. Read a fall in
this order:

1. **Sign.** The LQR uses theta = −pitch (`lqr.h`), so a positive logged
   `angle_deg` is caught by a negative `rate_sps`. Rate and angle of opposite
   sign at the cap means the wheels were driving the right way.
2. **The point of no return is when `rate_sps` first sits at the cap**, not the
   45° cutoff. Note the lean and the time from there to FAULT (0.6° and 0.47 s
   at 4000; 4.4° and 1.1 s at 5000). The cutoff is not the limit: with the
   40 000 steps/s² clamp and 65 mm wheels (5.1 m/s²) no lean past about
   atan(5.1/9.81) ≈ 27° can be held at all, so raising `TILT_CUTOFF_DEG` cannot
   save a fall.
3. **Did the motors keep up?** `rate_sps` and `pos_m` are commanded, not
   measured. Compare the ground acceleration the IMU implies,
   u = g·θ − L·θ̈ (θ̈ from a 50 ms difference of the gyro), with the change in
   `rate_sps`. At the cap the commanded change is 0; an implied value near 0
   (it scatters about ±2 m/s² on these logs) means the wheels held speed. A
   sustained positive gap would mean skipped steps or wheel slip.
4. **What asked for the speed.** Rebuild u = −K·s term by term from the log
   (θ = −pitch in rad, θ̇ = −gyro, x = `pos_m`, ẋ = `rate_sps` / steps-per-metre,
   K from `stat`). After a push is released, the angle term is what asks for
   speed; the position term brakes the return.

**Steady pushes.** Under a held push the lean is set by the push (about
push/weight), not by the gains; `q_x` decides how far the robot rolls before it
gets there. A lower `q_x` gets pushed further at the same lean, so it does not
make the release easier. A point-mass simulation with a 2 s push ramp
(2026-11-24) put the largest survived push at 12 % of weight for `q_x` 400 and
15 % for 1600 at `maxrate` 5000, failing on release, as the robot did. The
simulation is pessimistic in absolute terms: the robot held a 12° lean at 5000.
Change one setting per run, or the logs cannot say which helped.

## Traps from the bench

- **Wheels in the air run away by design.** The LQR's position and speed terms
  are positive feedback; only the body tipping back closes the loop. A 20 s run
  at maxrate in `stat` (`pos` ≈ 10 m) means the wheels were not moving the body.
- **Find frequencies with a spectrum, not zero crossings.** Counting crossings
  read 9.4 Hz for a 0.9 Hz swing: sensor noise adds crossings. `analyze` uses a
  DFT.
- **`fit-leff` did not work on real captures** (R² 0.06–0.13). Estimate
  `l_eff_m` from the geometry (README "Tuning loop"); the LQR tolerates a rough
  value.
- **Holding the robot at its balance point is the auto-arm gesture.**
  `disarm` and `trim capture` hold the gesture off (#1243). Without that
  firmware, put `disarm`, `"trim capture"` and `save` in one `cli` call.
- **The trim from a balancing capture beats a hand-held one.** With the
  position held within a few cm, the mean angle over 20 s is the balance point.
- **Tick 0 with `imu: NOT RESPONDING` 10 s after boot** is a held I²C bus:
  power-cycle with USB and battery both off (`BRINGUP.md` §5). In the first
  seconds after boot, `core 1: starting` is normal (#1244).
- **Connect USB before the battery** or no serial port appears (#1230).
- **A dead DRV8825 looks like a firmware fault.** On 2026-11-22 one wheel did
  not turn; A1 drives the left wheel (J4), A2 the right (J5). Swap the motor to
  the other header to split motor from driver.
- **Verify a sign against the hardware once.** The filter's accelerometer sign
  was wrong, and the host test's physics model had the same error, so the
  suite passed (#1242).

## Where the numbers live

- Captures: `tmp/balancebot/capture-<time>.csv`, run-log dumps
  `tmp/balancebot/log-<time>.csv`, both at the repo root (git-ignored).
- Compile-time limits: `src/config.h` (`MAX_ACCEL_SPS2`, `DEFAULT_*`,
  `ARM_GESTURE_*`). Runtime settings: `get` / `set` / `save`.
- Follow-ups: #742 (PID defaults), #1235 and #1237 (hardware checks).
