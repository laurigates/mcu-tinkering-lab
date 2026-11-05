# robocar-unified: Pan/tilt head aiming

Moved word for word from the project `CLAUDE.md`, which keeps a
one-line form of each rule below and links here for the reasoning.

## Pan/tilt head: the head leads, the wheels follow

The planner has **no head tool** ([ADR-025](../../../../../docs/decisions/ADR-025-reactive-head-aiming.md), issue #511). `track(box_2d)` stays its only aiming primitive, and the 30 Hz executor decides whether to satisfy it with the head, the wheels, or both. The logic is pure C in `head_aim.c`; `reactive_controller.c` feeds it the live servo pose and limits and writes what it returns.

- **A track box becomes a bearing once per plan.** `planner_task` records the head's pan/tilt when it captures the frame (`goal_t.head_pan_deg`/`head_tilt_deg`), and the executor adds the box centre's offset in that frame (`HEAD_CAMERA_HFOV_DEG`/`VFOV_DEG`). The bearing is latched against `goal_state_read_seq()`'s write sequence, not recomputed from the box every tick: the box describes one frame, and re-reading it from a turned head would add the same offset again forever.
- **The head leads.** It slews toward the bearing at `HEAD_SLEW_DEG_PER_TICK`, clamped to the live `servo limit` values on every step, and the wheels creep straight at `max_speed_pct` (0 holds position and only looks).
- **The wheels follow near the limit.** Past `HEAD_PAN_ENGAGE_PCT` of the pan limit on that side the body turns in place at `HEAD_BODY_TURN_SPEED`. Each tick of turning subtracts `HEAD_BODY_YAW_DEG_PER_S` worth of yaw from the bearing, so the head re-centres as the heading catches up and the turn ends inside `HEAD_RECENTRE_DEADBAND_DEG`. While the reflex or a manual lease holds the wheels, the estimate does not advance.
- **Anything else re-centres.** A stale goal, `stop`, `drive` and `rotate` slew the head back to 0/0.
- **The pose at capture also keys the scene gates.** `planner_task` classifies the same capture pose into a dwell pose (`scene_pose_from_head()`), and `plan_activity` and `scene_change` keep one reference per dwell pose ([ADR-029](../../../../../docs/decisions/ADR-029-idle-gaze-and-wander.md) §2; [planner.md](planner.md)). While a `track` goal or a lease holds the head off a dwell pose, both gates read the frame as "cannot measure": the planner is not woken by the view, and `speak` opens only on audio evidence. A head held at centre is measured as before.
- **`drive(heading_deg)` folds in the pan at capture.** The heading is stated in the frame the planner saw, so the body heading is `heading + head_pan_deg` (`head_aim_body_heading()`).
- **No servos, no aiming.** With `servo_is_initialized()` false, or after `servo off`, a track goal runs the original wheel-only P controller and nothing touches the servo driver. The same fallback applies while a head lease is live.
- **Writes are suppressed when nothing changed**, and re-sent every `HEAD_REFRESH_INTERVAL_MS`, for the same reason as the motors: a PCA9685 that browned out holds its power-on defaults while the shadow says otherwise.

**Two constants are unmeasured**, and both say so where they are defined: the camera FOV (an error scales where the head points) and the body yaw rate (the only thing that says when a turn has caught up — there is no IMU or odometry). The yaw rate is deliberately set high, so a wrong value ends the turn early rather than spinning past the target; the next plan's frame corrects the residual. Measure both on the bench and pin them.

**Console commands take a head lease**, the counterpart of `reactive_controller_manual()`. `servo pan|tilt` calls `reactive_controller_head_hold()` (30 s, `REACTIVE_HEAD_HOLD_TTL_MS`); `servo exercise` takes `reactive_controller_head_external()` on `peripheral_task` for its duration and releases it after. Only when the executor is not running does `servo pan|tilt` fall back to writing through `peripheral_task`. `servo on|off`, `servo limit` and `servo freq` are unchanged: the executor reads the enable flags and limits live every tick. `servo` alone prints the executor's head mode (`aim`, `turn`, `centre`, `lease`, `unavailable`).

A known race carried over from PR #574: `servo_controller.c` has no lock, so a `servo off` landing inside another task's `servo_set_angle()` can have its release overwritten by that one write. `servo exercise` already had this exposure; the executor adds a second writer, though it writes only on a change or once a second.

**The executor's aiming has not run on hardware.** The servos themselves are now confirmed good: the 2026-09-18 bench found one mechanically binding (not dead) and traced the non-following at 200 Hz to the PWM frame rate, which `PCA9685_FREQ_HZ` = 100 fixes. What remains unexercised is `head_aim` driving them.


## Don't

- Don't write the pan/tilt servos from anywhere except `reactive_controller.c` and the console lease path (`reactive_controller_head_hold()` for `servo pan|tilt`, `reactive_controller_head_external()` around `servo exercise`). The executor re-asserts the head every refresh interval and aims it at 30 Hz, so a second writer is overwritten within a second and fights the aiming loop meanwhile — the same failure the motors had before `reactive_controller_manual()`
- Don't add a head-aiming planner tool (`look`, `aim`, a `GOAL_KIND_LOOK`), as was ruled for `speak`. The planner calls every 15 s and sleeps when idle, so it cannot follow a moving target; the 30 Hz executor can. `track` stays the only aiming primitive and the executor chooses head, wheels or both — see ADR-025
- Don't recompute a track goal's bearing from its box every tick. The box describes one frame and there is no newer frame on the device between plans, so re-reading it from a turned head adds the same offset again and winds the head into its limit. `head_aim` latches the bearing per `goal_state_read_seq()` write
- Don't drop `goal_t.head_pan_deg`/`head_tilt_deg`, or fill them when the executor acts instead of when `planner_task` captures the frame. The head moves during the seconds a request takes; only the pose at capture puts a box or a drive heading into body coordinates, and only the pose at capture says which scene reference a frame belongs to (ADR-029 §2)
