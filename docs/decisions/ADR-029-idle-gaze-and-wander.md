# ADR-029: Idle Gaze and Wander in the Executor

**Status**: proposed
**Date**: 2026-11-05
**Source**: conversation 2026-11-05 (spectator review of Teuvo on the bench)
**Confidence**: 6/10 (the pose-keyed scene reference is untested; the gaze and
dwell constants are guesses until the bench sets them)
**Extends**: [ADR-016](ADR-016-hierarchical-ai-controller.md),
[ADR-025](ADR-025-reactive-head-aiming.md)
**Constrains**: [ADR-020](ADR-020-ambient-audio-speech-gate.md),
[ADR-022](ADR-022-planner-dormancy-and-spend-ceiling.md)

---

## Context

Watched on the bench on 2026-11-05, Teuvo mostly sits still with his head
centred, talking. A read-only review of that day's logs and the code found
that this follows from the design rather than from a fault:

- **The head moves only for a `track` goal.** `execute_head()`
  (`reactive_controller.c`) re-centres it for every other goal and for a stale
  one, as ADR-025 ruled. The planner sent no `track` in most captures; in one
  session with a ball in view it sent nine of eleven goals.
- **The planner mostly says `stop`.** 20 `stop` and 4 `rotate` goals across
  the 2026-11-04/05 captures before the ball appeared. The prompt asks for
  `stop` when the scene is ambiguous, and a cluttered table always is.
- **When nothing changes, the planner sleeps** (ADR-022), so it is consulted
  every 30–300 s and has nothing to send between calls.

People watching read a still head as an inattentive robot. Lauri's proposal
(2026-11-05) is behaviour the robot does without the planner, the way a person
looks around without deciding to:

- an **idle gaze**: the head looks around when the planner gives no direction
  for it;
- a **wander mode**: bounded body motion when nothing else is going on;
- **planner control**: the planner can turn both off, so they don't interfere
  with a longer plan, and back on.

Three existing rulings shape how:

1. **The executor is the only writer of the servos and motors** (ADR-025, and
   the Motors and Pan/tilt rules in `packages/robocar/unified/CLAUDE.md`).
2. **The planner gets no aiming primitive.** The `head-aiming.md` rule forbids
   a `look`/`aim` tool or a `GOAL_KIND_LOOK`, because a planner consulted every
   15–300 s cannot follow anything. `track` stays the only aiming primitive.
3. **The scene gates compare frames, not poses.** `plan_activity` (ADR-022)
   and `scene_change` (ADR-020) fingerprint each planner frame (8×6 blocks of
   luma, relative to the frame mean) against one reference frame. A head that
   points somewhere new makes every frame "new". The dormancy gate would wake
   the planner on every capture and `speak` would open on every cycle. That is
   chattering and API spend, which are what those ADRs exist to prevent.

`drive` and `rotate` also have no end point today: `execute_drive` ignores
`distance_cm`, and `execute_rotate` turns at 60 % until the next goal or the
22.5 s TTL. A wander built on them could carry the robot off a table edge,
which the front ultrasonic cannot see.

## Decision

Idle behaviour lives in the **executor**, as two layers below the planner's
goal. A **mode flag** governs it, which the console and the planner can set.

### 1. Idle gaze (head only)

When no fresh `track` goal holds the head, no head lease is live, and gaze is
enabled, the executor runs a slow scan instead of re-centring:

- **Dwell poses, not sweeps.** The head moves between a small fixed set of
  poses (initially pan −40°, 0°, +40° at the current tilt, clamped to
  `servo_get_limits()`), slews at a capped rate, and **dwells** 3–8 s at each,
  with the order and dwell drawn from `esp_random()` (the executor has no
  PRNG of its own; a host test injects one). Centre is visited
  every other move, so the view the planner knows comes back often.
- **A `track` goal or a lease pre-empts gaze immediately**, as now. Gaze
  resumes only after the goal goes stale.
- The executor writes the dwell pose into the state it already publishes, so
  `planner_task` keeps recording the pose **at capture**
  (`goal_t.head_pan_deg`, unchanged).

### 2. Pose-keyed scene references

`plan_activity` and `scene_change` keep **one reference per dwell pose**
instead of one in total. A frame captured at pan −40° is compared with the
last frame captured at −40°. A frame captured mid-slew (pose not one of the
dwell poses) is fingerprinted but neither compared nor adopted: it is "cannot
measure", which both modules already distinguish from "measured zero"
(`stateless-model-gating.md` §3).

This keeps both gates' meaning, "has *this view* changed since I last
planned on / spoke about it", while the head moves. Three poses × 48 blocks is
a few hundred bytes per module.

### 3. Local attention (what makes the gaze useful)

At each dwell, the comparison in §2 is also an **attention signal**. When the
view at the current pose scores over the scene threshold, the executor
**holds the gaze there**, extending the dwell, and the existing evidence path
(`plan_activity`'s scene score) wakes the planner on its next tick. A front
range reading that drops sharply does the same at pan 0°. This is the "looking
for things to act on" part: the robot notices locally at frame rate, and the
planner decides what it means. No new model call is made.

### 4. Bounded motion, then wander (body)

Wander needs motion that ends by itself, so this comes first:

- `drive` and `rotate` gain an **executor-side time bound** derived from
  `distance_cm` and `angle_deg` at a calibrated speed (open loop; there is no
  odometry), capped so that no single motion runs longer than a few seconds.
  This changes motion for every goal, not only wander, and gets its own host
  tests and bench calibration.
- **Wander** then issues short bounded motions itself while enabled and idle:
  a turn of 20–60°, then at most a short forward step if the front range is
  clear by a margin above `STOP_THRESHOLD_CM`. It never reverses (the rear
  sensor is not in the reflex). The obstacle reflex governs it like every
  other motion, and it runs through `reactive_controller_manual()`'s path.
- **Wander is off at every boot** and is enabled only by the console
  (`wander on`) or the planner. There is no edge sensor. Enabling it is a
  decision about where the robot stands, and that decision is Lauri's.

### 5. Control: one mode, three writers

`idle_mode` holds two booleans, `gaze` and `wander`, plus an optional expiry:

| Writer | How | Default |
|---|---|---|
| Boot | — | `gaze` on, `wander` off |
| Console | `idle`, `idle gaze on/off`, `idle wander on/off` | — |
| Planner | a `set_idle(gaze, wander, minutes)` function declaration | — |
| MQTT (full mode) | the same console line | — |

The planner tool is **not an aiming primitive**: it says whether to look
around, never where. The `head-aiming.md` rule against a `look`/`aim` tool
therefore holds. The tool is offered on every cycle, so the planner can pause
idle behaviour for a multi-step plan (`set_idle(false, false, 5)`) and the
expiry restores the previous mode if it never sends the re-enable. A fresh
non-`stop` motion goal pauses `wander` for its duration in any case, so a
plan's own motion never interleaves with a wander step.

## Consequences

- **Visible**: the head looks around, settles on what changed, and the planner
  is consulted about it. The robot reads as attentive without more API calls.
- **Spend**: unchanged at rest, since the pose-keyed reference stops gaze from
  waking the planner. It rises when the gaze finds something, which is the
  point.
- **Planner prompt**: grows by one function declaration. Check
  `GEMINI_SYSTEM_PROMPT_MAX` and `PLANNER_TASK_STACK_SIZE` per the voice rules.
- **Two ADR rulings change**: ADR-025's "stale or non-track goal → head
  re-centres" becomes "→ head re-centres, or gazes if gaze is on"; ADR-022's
  single scene reference becomes one per dwell pose. Both are recorded here,
  and the head-aiming and planner docs and the CLAUDE.md rules are updated in
  the implementing PRs.
- **Every `drive`/`rotate` becomes bounded** (§4). That is a behaviour change
  for planner goals too, and the safer default.
- **Voice turns** (#947) can later reuse the gaze machinery to look where they
  are asked, through a head lease. That is out of scope here.

## Implementation order

Each step is a PR, testable before the next:

1. Pose-keyed references in `plan_activity` and `scene_change`, with host tests
   for a moving head over a static scene (must score 0) and a change at one
   pose (must score at that pose only). No behaviour change while the head
   stays centred.
2. Idle gaze in `reactive_controller.c`/`head_aim`, with the `idle` console
   command. Bench: head on, wheels off; gaze must not raise the planner's
   request rate on a static scene.
3. Attention hold (§3).
4. Time-bounded `drive`/`rotate`, with calibration on the bench box.
5. Wander, and the planner's `set_idle` tool.

## Open questions

- Dwell poses and timings: three pan poses at the current tilt, or include a
  tilt-down pose for the floor in front?
- Should gaze stop while a voice turn is listening, so the robot looks
  "at" the speaker (pan 0°)? There is no direction-of-arrival from one mic.
- Speech while gazing: should a remark at pan ±40° say which way it is
  looking ("vasemmalla…", on the left)? The prompt can carry the pose.

## Related

- [`packages/robocar/unified/docs/firmware/head-aiming.md`](../../packages/robocar/unified/docs/firmware/head-aiming.md)
- [`packages/robocar/unified/docs/firmware/planner.md`](../../packages/robocar/unified/docs/firmware/planner.md)
- Issues #944 (rangefinders drop out while the motors run; matters for
  wander's clear-range check) and #947 (voice turns promise actions)
