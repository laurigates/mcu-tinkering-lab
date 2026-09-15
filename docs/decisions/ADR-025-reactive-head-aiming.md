# ADR-025: Reactive Head Aiming — The Executor Owns the Pan/Tilt Head

**Status**: accepted
**Date**: 2026-09-15
**Source**: issue #511, decision comment 2026-09-14
**Confidence**: 6/10 (no hardware verification; two unmeasured constants)
**Extends**: [ADR-016](ADR-016-hierarchical-ai-controller.md)

---

## Context

`robocar-unified` has a pan/tilt head on PCA9685 channels 6 and 7, and nothing under autonomous control moves it. Gemini declares five tools (`drive`, `track`, `rotate`, `stop`, `speak`); `track(box_2d)` visually servos toward a target by steering the whole robot. Turning the body to look at something costs a heading change, motor current and floor space, where turning the head costs none of those. Following a target that crosses the frame while the robot holds position cannot be expressed at all.

Issue #511 set out three channels for head aiming:

- **A. A new `goal_kind_t`** (`look(pan, tilt)`). It would compete with `drive`/`track` for the single last-write-wins goal slot, so the robot could not aim and move at once.
- **B. A separate queue**, the way `speak` travels beside `goal_state` (ADR-019).
- **C. No planner change.** The reactive executor satisfies `track` with the head, the wheels, or both.

The planner calls at most every 15 s and sleeps when nothing changes (ADR-022). A head aimed by the planner holds a fixed angle for up to 15 s, so it cannot follow a moving target under either A or B.

## Decision

**Option C, with the head leading** (Lauri, #511, 2026-09-14). The planner gains no tool, and the prompt, parser and `goal_kind_t` are unchanged. The 30 Hz executor owns the head as it owns the wheels.

### Aiming a track goal

1. `planner_task` records the head's pan/tilt when it captures the frame, in `goal_t.head_pan_deg`/`head_tilt_deg`.
2. When a new goal arrives, the executor converts the box once into a body-relative bearing: the pan at capture plus the box centre's offset times the camera FOV. Tilt is handled the same way. "New" means a different `goal_state_read_seq()` write sequence.
3. The head slews toward the bearing, clamped to the live travel limits on every step. The wheels creep straight at `max_speed_pct`.
4. Past 80 % of the pan limit on that side, the body turns in place toward the bearing. Each tick of turning subtracts an estimated yaw from the bearing, so the head re-centres as the heading catches up. The turn stops within 5° of straight ahead.

### Rulings on the points #511 left open

| Point | Ruling | Why |
|---|---|---|
| Stale or non-track goal | The head slews back to centre. | Mirrors the motors stopping on a stale goal. A head left turned would put the next frame, and every heading derived from it, off-axis for no reason. |
| `drive(heading_deg)` with the head turned | Body heading = `heading_deg + head_pan_deg`, wrapped to ±180. The pan is recorded **with the goal, at capture**. | The heading is stated in the frame the planner saw. Reading the pan when the executor acts would use a pose seconds later, after the head has already moved during the request. The same field converts `track` boxes. |
| Console `servo` commands | `servo pan|tilt` takes a 30 s head lease (`reactive_controller_head_hold()`). `servo exercise` holds `reactive_controller_head_external()` for its duration. `servo on|off`, `servo limit` and `servo freq` are unchanged, because the executor reads the enable flags and limits live. | This is the counterpart of `reactive_controller_manual()`. A second writer would be overwritten within one refresh interval. The lease is long because a held head is a bench diagnostic, not a hazard. |
| Limits and write rate | Every command is clamped to `servo_get_limits()` after slewing, so a narrowed limit applies on the next tick. An unchanged command is re-sent only every 1000 ms. | The clamp after slewing matters: clamping only the target would slew back into a narrowed range over several out-of-range ticks. The refresh re-asserts a PCA9685 that browned out, as `MOTOR_REFRESH_INTERVAL_MS` does. |
| No servos | `servo_is_initialized()` false, `servo off`, or a live head lease: `track` runs the original wheel-only P controller, and the executor makes no servo call. | A board without a working head behaves exactly as before. |

### Why the bearing is latched instead of servoed from the box

The box describes one frame, and the device has no newer frame until the next plan. Treating `cx - 500` as a live error and servoing on it every tick would add the same offset again from an already-turned head, winding the head into its limit. The executor therefore latches the box once per plan and advances it only by what it knows changed, which is its own estimated yaw.

## Consequences

**Positive**
- Aiming and driving are no longer mutually exclusive. The head covers a target within its travel at no heading or floor-space cost.
- The planner surface is unchanged, so no prompt tokens are spent and no parser or fixture work is needed.
- The decision logic is pure C (`head_aim.c`) and runs in host tests, including the 49-day millisecond wrap and a limit narrowed mid-track.
- The drive/track/rotate/manual helpers in `reactive_controller.c`, previously duplicated between the target and host builds, are compiled once, so the host tests exercise the shipped code.

**Negative**
- **Two constants are unmeasured.** The first is the camera FOV (60° × 45°); an error scales where the head points. The second is the in-place yaw rate (180°/s at speed 153). With no IMU or odometry, it is the only thing that ends a body turn. It is set high so that a wrong value ends the turn early; the next plan's frame corrects the residual.
- Driving straight while the head covers a target off to one side does not approach that target until the bearing passes the engage edge. This follows from the ruling that the wheels act only near the pan limit.
- Re-centring after a goal expires changes the view, which ADR-022's dormancy gate reads as evidence. That costs one extra planner request per expiry, which is bounded.
- `servo_controller.c` has no lock. A `servo off` landing inside another task's `servo_set_angle()` can have its release overwritten by that single write. `servo exercise` already had this exposure before this change; the executor adds a second writer that writes on a change or once a second.

**Unverified**
- Nothing has run on hardware. When #511 was decided, the servos were reported not following commands at 200 Hz, and that diagnosis is separate. The executor's added stack use (servo writes from the 4096-byte `reactive_ctrl` task) is visible in its existing high-water-mark log and has not been read on a board.

## Alternatives Considered

1. **A `look` goal kind (A)** was rejected. It competes with motion for the goal slot and runs at the planner's 15 s cadence.
2. **A head queue beside `goal_state` (B)** was rejected. It avoids the slot conflict, but a head aimed from the planner still cannot follow a moving target, and it costs a prompt and parser change.
3. **Servoing on the box every tick** was rejected; see "Why the bearing is latched".
4. **Steering the wheels proportionally while the head aims** was rejected. Every wheel turn changes the bearing, and without odometry that change can only be estimated. Turning only in place at one fixed speed keeps the estimate to a single constant.

## Related

- **Issue #511**: the design question and decision
- **PR #574**: servo travel limits, `servo on|off`, and the frequency change this builds on
- **ADR-016**: planner/executor split, extended here so the executor also owns the head
- **ADR-019**: the precedent for keeping `speak` out of `goal_kind_t`
- **ADR-022**: the dormancy gate that reads head motion as a view change
