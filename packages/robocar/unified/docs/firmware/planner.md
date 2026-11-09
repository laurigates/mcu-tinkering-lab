# robocar-unified: AI planner, dormancy and spend ceiling

Moved word for word from the project `CLAUDE.md`, which keeps a
one-line form of each rule below and links here for the reasoning.

## AI planner

**Gemini Robotics-ER 2 only** (`gemini-robotics-er-2-preview`; ER 1.6 was shut down at the end of August 2026). The planner calls Gemini to emit function-call goals:

- `drive(heading_deg, distance_cm, speed_pct)` — absolute heading + distance
- `track(box_2d, max_speed_pct)` — visual servo toward a bounding box
- `rotate(angle_deg)` — spin in place
- `stop()` — hold position

- `speak(text)` — say one short sentence aloud, emitted *in addition to* a movement call
- `sound(name)` — play a prebuilt clip (`horn`, `aoogah`, `bell`, `boing`), offered only when `speak` is; the parser puts it at the front of the line as a `[name]` tag (see [voice.md](voice.md#sound-clips))
- `note(kind, text)` — a note to the developers, sent to GlitchTip and never spoken, offered only after a trigger (see [Planner notes](#planner-notes))

The planner runs every `PLANNER_LOOP_PERIOD_MS` (15 s default, set by the Gemini free-tier quota rather than by control preference) on Core 1; the executor drives the goal at ~30 Hz on Core 0. No on-demand inference or blocking on responses — the planner is a background task that constantly updates `goal_state`, and the executor always has something to do.

## The planner does not run unless something is happening

The loop above describes the cadence when the robot is *awake*. It is not awake
by default, and this is the part to understand before changing anything in
`planner_task.c`.

The planner used to call Gemini on every 15 s tick, unconditionally and forever —
240 requests an hour whether or not anything had changed. A board left plugged
into a laptop overnight ran all night; nothing on the device had any notion that
it was spending anything. ADR-020's three speech gates were already computing
almost exactly the evidence needed to know better, but they gate what goes *into*
a request, not whether one is made: on a quiet cycle the request got cheaper and
still went out, JPEG and all. [ADR-022](../../../../../docs/decisions/ADR-022-planner-dormancy-and-spend-ceiling.md)
carries the full argument.

Two mechanisms now sit in front of the call, and they are deliberately different
kinds of thing.

### `plan_activity` — evidence, with a ladder rather than a switch

A request is made when any of five things is true: the **view** has changed from
the frame last planned on, the **room** has made a noise
(`ambient_audio_event()`), the **rangefinder** reading has moved, the robot is
**in motion**, or somebody typed at the **console**. Otherwise the interval grows
15 s → 30 → 60 → 120 → 300 and only then stops entirely — five requests over
about nine minutes.

"In motion" is a console lease or a fresh goal that drives the robot
(`goal_moves_wheels()` in `goal_state.c`): `drive`, `rotate`, or `track` with
`max_speed_pct > 0`. A look-only `track` (speed 0) does not drive forward; it
turns in place only to re-centre the head when the pan runs out, and it does
not count (issue #958).

**The ladder is the design, not a softening of it.** A binary sleep is only as
good as its wake detector, and this project has already shipped a gate that was
wrong about its own sensor: `ambient_audio.c` failed *open* with the microphone
absent and claimed novelty every cycle for a whole boot. The mirror of that
failure is a robot that never wakes, which reads as bricked and which standing in
front of it does not fix. The ladder bounds both directions — a deaf detector
costs 12 requests an hour and recovers by itself, and the symptom is visible in
the log as a ladder climbing while a person is plainly in the room.

**Dormancy stops the request, not the loop.** Capturing and fingerprinting a
frame costs CPU and nothing else, so that keeps happening every 15 s while
dormant. Wake latency is one tick however long it has been asleep.

**It keeps its own scene reference, and must.** `scene_change` measures from the
frame last *spoken about*; this measures from the frame last *planned on*. The
two move at different moments, so one field cannot serve both — which is why the
planner log now carries `scene:` and `still:` side by side. Both share
`scene_change.c`'s block-mean-minus-frame-mean representation, so the AGC/AEC
rewriting exposure on a motionless scene does not read as a change.

**Both references are kept per dwell pose of the head** ([ADR-029](../../../../../docs/decisions/ADR-029-idle-gaze-and-wander.md) §2).
`planner_task` names the pose the head was at when it captured the frame
(`scene_pose_from_head()`: centre, pan ±`SCENE_POSE_SIDE_PAN_DEG` level, or the
floor pose at tilt `SCENE_POSE_FLOOR_TILT_DEG`, each within
`SCENE_POSE_TOLERANCE_DEG`), and a frame is compared only with the last one
planned on, or spoken about, at the same pose. A frame taken anywhere else —
mid-slew, while a `track` goal or a console lease holds the head — is "cannot
measure": neither compared nor adopted, so it neither wakes the planner nor
opens `speak`. So is a frame taken before the head settled: the executor's
`head_settled` telemetry says the commanded pose has been still for
`HEAD_SETTLE_MS`, and without it the frame gets no dwell pose. That is what
lets the idle gaze ([head-aiming.md](head-aiming.md)) look around a static room
without waking the planner. The first look at a pose is adopted silently, like the boot
frame. A head that stays centred, or a robot with no servos, uses only the
centre reference, which is the single reference these gates had before.

**Audio evidence is `ambient_audio_event()`, never `ambient_audio_novel()`.**
`novel()` additionally treats a room never spoken about as new. That is right for
speech and fatal here: a dormant robot never speaks, so it would never see that
branch clear and would never go quiet at all.

**The board boots dormant.** The first valid frame becomes the reference rather
than counting as a change — a first observation cannot be evidence that something
changed. A board powered on in a still room therefore makes no request at all.

### `plan_budget` — a counter, and the only layer that can be trusted absolutely

Past a per-boot ceiling on requests or tokens the planner is refused regardless
of what any detector believes. It lives at the `gemini_backend_plan()` choke
point rather than in the planner loop, for the same reason the `activity_trace`
hooks live inside `gemini_http_post()`: a future second caller inherits the fuse
instead of quietly spending outside it.

Charging **fails closed on the parse and open on the network**. A response whose
`totalTokenCount` cannot be read is charged `PLAN_BUDGET_ASSUMED_TOKENS`, and the
count of such requests is reported by `plan` — charging zero there is how a spend
ceiling silently stops being one while every log line still looks healthy. A
request that never produced a body is counted but charged no tokens, because a
robot with no network is the one situation where it is provably not spending, and
phantom tokens would let an outage trip the fuse.

**Tripping is terminal.** No rolling window, no auto-reset: `plan resume`, an
MQTT command, or a reboot. A fuse that resets itself is not a backstop against
the unattended board it exists for.

`gemini_parse.c` now keeps the token counts it had been logging into
`GP_MAYBE_UNUSED` locals, and reads them *before* the branching that can `goto
done` — so a response that parses as JSON but carries no recognisable
`functionCall` now reports its cost, which is exactly the response you most want
the cost of.

## Planner notes

The planner can tell its developers about its own behaviour and the
difficulties it runs into: `note(kind, text)`, where `kind` is `difficulty`,
`behaviour` or `suggestion` and `text` is at most 280 characters. The note goes
through `event_report` to `robocar/events` with `source: planner`, and the
GlitchTip bridge routes it to the `robocar-ai` project
([ADR-033](../../../../../docs/decisions/ADR-033-robocar-error-reporting.md)).

**Offered on evidence, like `speak`.** The tool and its prompt clause are in a
request only when a trigger has fired since the last offer, so a quiet robot
pays no prompt tokens for the feature. The triggers are raised in
`planner_cycle_core.c` (so the behaviour twin runs them) from facts the robot
already has:

| Trigger | Raised when |
|---|---|
| `reflex` | the executor's `reflex_trips` count changed since the last request (a brake that tripped and released between two plans still counts) |
| `blind` | a request is made while no camera frame has decoded; once per blind spell |
| `track_stuck` | `track` goals written on `NOTE_GATE_TRACK_STUCK_CYCLES` (4) requests in a row; the executor does not report arrival, so the streak stands in for "not converging" |
| `repeats` | `NOTE_GATE_REPEAT_STREAK` (3) spoken lines in a row withheld as near-repeats |
| `reflect` | every `NOTE_GATE_REFLECT_EVERY` (40) requests, about 10 min awake |

The prompt names the trigger through `note_trigger_reason()`, which states only
what was measured (the reflex reason says the front rangefinder read something
close, not what it was). The model writes only the prose. The firmware attaches
the trigger, the kind, the persona, the reflex count, the `scene:` and `still:`
scores, the request count and the last five goals (as `goal` breadcrumbs), and
fingerprints the event by `[trigger, kind]` so GlitchTip groups notes on facts.

**Rationed by `note_gate.c`.** One offer per 5 minutes and 20 per boot. Offers
are counted, not notes: a declined offer still carried the declaration and the
clause. Taking an offer clears every pending trigger; a trigger raised while
the window is shut waits for it. A request that never left the board (no
WiFi link or no key) gives its offer back (`note_gate_untake()`), so an offline
hour does not burn the triggers. The per-boot count stays spent until
`plan resume` or a reboot, like the spend fuse. `plan` prints
`notes: <offered>/20 offered, <n> written, next in <s> s, pending: <triggers>`.

A note in a reply whose request did not offer the tool is dropped by
`gemini_backend_plan()`, and the parser never takes a `note` call as the motion
goal, whichever order the calls come in. `test_note_gate.c`,
`test_planner_cycle_core.c` and `test_gemini_parse.c` pin those, and the twin
scenario `us-n1-note-on-reflex` checks that four reflexes in six minutes yield
exactly two offers.

### Reading it

```text
Goal: STOP | latency: 1188 ms | pose: C | scene: 3/8 | ... | gate: . | still: 1/6 | why: + | step: 2/5 | spent: 41/500 req, 149203 tok
Idle: no request | pose: C | still: 0/6 | range: 0/20 cm | loud: 2/12 dB | rung: 3/5 every 60000 ms | why: .
```

`pose:` is the dwell pose the frame was taken at (`C`, `L`, `R`, `F`), or `-`
between poses. With `-`, `scene:` and `still:` read 0 because the frame could not
be measured, not because nothing moved.

`why:` letters combine — `W` woken, `M` moving, `V` view, `A` audio, `R` range;
`+` a keep-alive fell due with no evidence, `.` holding between rungs, `Z`
dormant. On an empty desk the sequence should read `+ . . + . . . +` and end at
`Z`. `Z` while somebody is visibly moving means `plan scene` is too high; a
ladder that never leaves step 1/5 on an empty desk means it is too low, or the
camera is noisier than the threshold. Dormant cycles log at DEBUG so an overnight
run stays quiet; entering dormancy and waking each emit one INFO line.

Every threshold is a console knob and none of them persists to NVS, for the same
reason `cam gainceiling` and `voice scene` are: whether the robot is sensibly
frugal or annoyingly asleep is a judgement that needs somebody standing in the
room. `plan off` restores the original behaviour exactly — a request every tick —
while leaving the budget fuse in force.

Both modules are pure C with injected clocks, so the whole nine-minute ladder and
the uint32 millisecond wrap at day 49 run in microseconds:

```bash
just robocar-unified::test   # + test_plan_activity, test_plan_budget
```


## Don't

- Don't make the planner's request unconditional again, and don't "simplify" the ladder into an on/off switch. An idle board went from 240 requests an hour to zero because the request itself is gated; a binary sleep is only as good as its wake detector, and a detector that fails closed is a robot that never wakes. See ADR-022 and the planner-dormancy section above
- Don't feed `ambient_audio_novel()` to the dormancy gate. It reports a room never spoken about as novel — right for speech, fatal here, because a dormant robot never speaks and so would never see that branch clear. `ambient_audio_event()` exists for exactly this and is the same OR without the first-impression branch
- Don't re-point `plan_activity`'s scene reference at `scene_change`'s, or fold the two `scene:`/`still:` log fields into one. They measure from different frames — last spoken about, last planned on — and those move at different moments
- Don't collapse the per-pose references back to one, or compare an off-pose frame against any of them. A head looking around shows a new view at every pose, so one reference wakes the planner on every capture and opens `speak` on every cycle (ADR-029 §2). `test_a_moving_head_over_a_static_scene_never_calls` and its `test_scene_change.c` twin fail if either reverts
- Don't charge zero tokens for a response whose `usageMetadata` could not be read. That is how a spend ceiling silently stops being one: every log line keeps showing a healthy budget while the robot spends all night. `plan_budget_note()` charges the assumed cost and counts that it had to, and `plan` prints the count unconditionally
- Don't give the budget fuse a rolling window or an auto-reset. It exists for the unattended board, which is precisely the case that must not resume on its own. The same goes for the voice-turn ceiling in `voice_budget.c`
- Don't offer the `note` tool on every request, or word the clause to ask for restraint instead. The model cannot know what happened since its last note; `note_gate` decides on-device and the clause names the trigger (stateless-model-gating.md). Keep the trigger logic in `planner_cycle_core.c`, where the twin runs it
- Don't give the note gate's per-boot count an auto-reset, for the same reason as the spend fuse
- Don't add goal sources outside `planner_task.c` — structured goals keep the two layers decoupled. If a new goal source is needed, it should write `goal_state` the same way the planner does
- Don't resurrect `claude_*` or `ollama_*` source files. If a second backend becomes necessary, design an abstraction; old code paths are not a foundation
