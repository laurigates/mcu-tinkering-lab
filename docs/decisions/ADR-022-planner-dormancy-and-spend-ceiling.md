# ADR-022: Gate the Planner Request Itself — Dormancy Ladder plus a Hard Spend Ceiling

**Status**: accepted
**Date**: 2026-08-28
**Source**: conversation 2026-08-28
**Confidence**: 7/10

---

## Context

`planner_task.c` called `gemini_backend_plan()` on every one of its 15 s ticks,
unconditionally and forever. That is 4 requests/minute, 240/hour, and roughly
2 900 overnight, each carrying a QVGA JPEG through a vision model together with
a multi-kB prompt and however many thinking tokens the model spent.

The board was left plugged into a laptop overnight with no speaker attached. It
ran all night. Nothing on the device had any notion that it was spending
anything, and nothing would have stopped it; prepaid quota was the only backstop
that existed.

The firmware already computed almost exactly the evidence needed to know better.
ADR-020's three speech gates — `speech_budget` (ration), `scene_change` and
`ambient_audio` (two senses) — combine in `gemini_backend.c` to decide whether
the `speak` **tool declaration** goes into the request. They were applied one
layer too late. On a quiet cycle the request got *cheaper*, because the whole
speech half of the prompt was omitted. It still went out.

A second gap made the first invisible: `gemini_parse.c` parsed `usageMetadata`
into locals marked `GP_MAYBE_UNUSED` and logged them. The per-request token count
was on the wire and printed to the console, and nothing summed it, so there was
no number any policy could have been written against.

## Decision

Three layers, deliberately independent, in increasing order of how much they can
be trusted.

### 1. `plan_activity` — a dormancy gate on the request, with a backoff ladder

A new pure-C module decides, from on-device evidence, whether a cycle is worth a
request. Evidence is the OR of: a changed view, a latched audio event, a changed
rangefinder reading, the robot being in motion, and an external wake (any console
line). All of it is free — a fingerprint the frame decode already produced, a
telemetry snapshot the executor already maintains, a latch the microphone task
already fills.

**It is a ladder, not a switch.** While nothing changes the interval grows
15 s → 30 → 60 → 120 → 300, and only after the whole ladder is spent — five
requests over about nine minutes — does the planner stop calling entirely.

This is the crux of the decision, and it is a direct consequence of this
project's own history. `ambient_audio.c` failed **open** when the microphone was
absent and asserted novelty on every cycle for a whole boot. The mirror failure —
a wake detector that fails **closed** — is a robot that never wakes again, which
reads as bricked and which standing in front of it does not fix. A ladder bounds
both: a detector that has gone deaf costs a few minutes of reduced cadence and
five requests, and the symptom is visible in the log as a ladder climbing while a
person is plainly in the room.

**The evidence loop keeps running while dormant.** Dormancy stops the network
call, not the tick. The planner still captures and fingerprints a frame every
15 s, so wake latency is one tick regardless of how long it has been asleep.

**The reference is the view the robot last planned on**, not the previous frame.
Frame-to-frame comparison is blind to slow drift. This is a *second* reference
alongside `scene_change`'s "frame last spoken about"; the two ask different
questions of the same fingerprint and move at different moments, so neither can
serve for the other. The representation is shared, which is what makes the gate
immune to the AGC/AEC rewriting gain and exposure on a motionless scene.

**Audio evidence uses a new `ambient_audio_event()`, not `ambient_audio_novel()`.**
`novel()` reports a room never spoken about as novel — correct for speech, fatal
here, because a dormant robot never speaks and would therefore never see that
branch clear. The extraction leaves `novel()` byte-identical in behaviour.

### 2. The boot state is dormant

A board powered on in a static room makes no request at all. The first valid
frame is adopted as the reference rather than treated as a change, because a
first observation cannot be evidence of a change.

### 3. `plan_budget` — a hard ceiling that trips terminally

A counter, deliberately the dumbest thing in the chain: past a per-boot ceiling
on requests or tokens it refuses the planner regardless of what any detector
believes. Enforced at the `gemini_backend_plan()` choke point rather than in the
planner loop, for the same reason the `activity_trace` hooks live inside
`gemini_http_post()` — a future second caller inherits the fuse instead of
quietly spending outside it.

Charging fails **closed on the parse and open on the network**:

| Outcome | Requests | Tokens |
|---|---|---|
| `usageMetadata` present | +1 | reported `totalTokenCount` |
| body returned, no readable total | +1 | `PLAN_BUDGET_ASSUMED_TOKENS`, and `assumed` count +1 |
| no response at all (DNS, TLS, no WiFi) | +1 | 0 |

The middle row is the load-bearing one. Google renaming a field, or a parse
regression, would otherwise make every request cost zero — a fuse that silently
stops being a fuse while every log line still shows a healthy budget. The
`assumed` counter is reported unconditionally by `plan`, so "the token figure
beside this is a guess" is visible rather than inferred later from a bill.

The last row is why charging is not simply "always assume the worst": a robot
with no network is the one situation in which it is provably not spending, and
charging phantom tokens there would let an overnight outage trip the fuse.

**No rolling window and no auto-reset.** Recovery is `plan resume`, an MQTT
command, or a reboot. A fuse that resets itself is not a backstop against an
unattended board, which is the entire case this layer exists for.

## Consequences

- An idle board costs **zero** planner requests instead of 240/hour. A board
  whose wake detector has failed costs 12/hour, self-recovering.
- Wake latency is one planner period (15 s) from any of: motion in view, a sound
  over the ambient threshold, a rangefinder change, robot motion, or a console
  line.
- **In a genuinely static, silent, unattended room the robot stops planning and
  stays stopped.** That is the intent, not a fault. It is why `plan` prints every
  live score beside its threshold, why the ladder-climbing phase is logged at
  INFO while dormant cycles are logged at DEBUG, and why entering dormancy and
  waking each emit exactly one INFO line.
- Every threshold is a `plan` console knob, non-persistent, for the reason
  `cam gainceiling` and `voice scene` are: whether the robot is sensibly frugal
  or annoyingly asleep needs somebody standing in the room to judge, and a
  reflash per trial is far too slow a loop.
- `plan off` restores the original behaviour exactly — a request every tick. The
  budget fuse still applies; that switch disables the optimisation, never the
  backstop.
- Token accounting now exists at all, which is what makes
  `PLAN_BUDGET_ASSUMED_TOKENS` and both ceilings replaceable with measured
  numbers rather than the estimates they currently are.

### Not done here

- **The TTS, narrate and voice-turn paths are outside the fuse.** All three are
  event-driven and already rationed (`speech_budget`, a subsystem health change,
  an explicit `listen`), so none of them can run away on an unattended board the
  way the planner loop could. Extending the ceiling to cover them is a separate
  question, and audio output tokens are expensive enough that it is worth asking.
- **No dormancy indicator on the LEDs.** `activity_trace`'s colour table is
  documented and load-bearing; adding a state to it belongs in its own change.
  Off-tether, dormancy is currently only visible over MQTT.
- **MCU sleep is a different axis.** Deep sleep would kill WiFi, the reactive
  executor and every wake detector; light sleep is incompatible with polling the
  camera and microphone. "Dormant" here means *API dormancy* with the MCU fully
  awake. If battery life becomes the goal, that is a separate design.

## Alternatives considered

**Binary sleep with no ladder.** Simplest, and rejected for the fail-closed
reason above: this project has already shipped a gate that was wrong about its
own sensor, and the cost of being wrong in the sleeping direction is a robot
that never comes back.

**A rolling-window budget that auto-resets.** Self-healing, and exactly wrong for
the scenario that prompted this: an unattended board would resume spending on its
own the moment the window rolled.

**Detect the USB host and refuse to plan while tethered.**
`usb_serial_jtag_is_connected()` distinguishes a real host from a power bank via
SOF packets, which does target the literal scenario. Rejected as a *trigger*: a
bench session with the monitor attached is precisely when the planner should be
running. The console-line wake captures the useful half of the same signal
without the inversion.

**Cheaper requests instead of fewer.** Smaller frames, shorter prompts. Image
tokens are near-flat for a small image and the speech half of the prompt is
already omitted when muted. Not the lever.

## References

- [ADR-016](ADR-016-hierarchical-ai-controller.md) — the planner/executor split
  whose planner loop this gates
- [ADR-020](ADR-020-ambient-audio-speech-gate.md) — the three speech gates, one
  layer in from this one, and the source of `ambient_audio_event()`
- `.claude/rules/stateless-model-gating.md` §4 — a gate feeding a stateless
  prompt must fail closed; the audio-gate fail-open this decision is shaped by
- `packages/robocar/unified/main/plan_activity.h`, `plan_budget.h` — the design
  arguments in full, beside the code
