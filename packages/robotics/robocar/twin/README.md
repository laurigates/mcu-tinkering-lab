# robocar-twin — behaviour twin for the robocar planner and executor

The robocar's planner loop and its 30 Hz executor compiled for the host and
run on a virtual clock, driven by scripted scenarios and checked against what
the robot should do.
Requirements are in [PRD-017](../../../../docs/requirements/PRD-017-robocar-behaviour-twin.md),
the design in [ADR-030](../../../../docs/decisions/ADR-030-robocar-behaviour-twin.md)
and [ADR-036](../../../../docs/decisions/ADR-036-robocar-twin-plant-port.md).
This README covers how to use it.

The twin runs the firmware's own C, unmodified: `planner_cycle_core.c` (the
per-cycle decisions extracted from `planner_task.c`), `plan_activity`,
`plan_budget`, `scene_change`, `ambient_audio`, `speech_budget`,
`dialogue_style`, `goal_state` and `gemini_parse` for the planner, and
`reactive_controller.c` (its host path), `motor_controller.c`,
`ultrasonic_core.c`, `head_aim.c`, `idle_gaze.c`, `cliff_core.c` and
`cliff_sensor.c` for the executor, built against the host shims in
`../test/include` (see [`CMakeLists.txt`](CMakeLists.txt)). Python never
reimplements gate or reflex logic: the harness supplies the world and the
model's replies and reads the trace.

```
harness (Python) ──JSONL stdin──▶ robocar-twin (C) ──JSONL stdout──▶ harness
 scenario world events            planner_cycle_core, the executor   assertions,
 model replies                    and firmware modules on a          scorecard
                                  virtual clock
```

## Quickstart

Needs CMake, a C compiler, `libcjson-dev` (`brew install cjson` on macOS) and
`uv`. From the repo root:

```sh
just robocar-twin::test    # build, smoke test, harness unit tests, every scenario
just robocar-twin::run     # every scenario, scorecard at build/scorecard.json
just robocar-twin::unit    # harness unit tests only; scenario files validated, not run
just robocar-twin::build   # build/robocar-twin
just robocar-twin::clean
```

CI runs `just robocar-twin::test` in the Python Suites job of
[`.github/workflows/test.yml`](../../../../.github/workflows/test.yml) on any
change here, under `../main/` or to the `../test/` files the twin compiles;
its entry and trigger paths are in
[`.github/python-suites.json`](../../../../.github/python-suites.json).

To keep each scenario's trace, run the harness directly:

```sh
cd harness && uv run python -m robocar_twin run --scenarios ../scenarios/*.yaml --traces ../build/traces
```

## Layout

| Path | Contents |
|---|---|
| `src/twin_main.c` | virtual clock, event loop, JSONL protocol, the glue that stands in for the FreeRTOS tasks (each piece cites the firmware lines it mirrors) |
| `src/twin_exec.c` | what sits under the executor: the rangefinder bus over `ultrasonic_core.c`, the PCA9685 that `motor_controller.c` writes, the floor sensors' sample, and the executor's bring-up and tick |
| `src/twin_signals.c` | deterministic synthetic camera luma (from the scene name) and microphone PCM (from the seed) |
| `src/twin_sha256.c` | SHA-256 for the request key |
| `tests/smoke.sh` | ctest smoke test: no link, no request, a `no_link` stop |
| `harness/` | uv project: scenario loader, model ports, runner, assertions, scorecard ([harness README](harness/README.md)) |
| `scenarios/*.yaml` | one file per user story |

## Protocol

One JSON object per line. The harness writes `config`, every world event in
time order, then `{"t":"run"}`.

| Direction | Records |
|---|---|
| harness → twin, before `run` | `config` (`seed`, `duration_ms`, `planner_period_ms`, optional `plan_budget_max_requests`, `plan_budget_max_tokens`, `head`, `cliff`); world events `frame`, `range`, `range_fail`, `floor`, `sound`, `voice_turn`, `console`, `link`, each with `at_ms` |
| twin → harness | `model_request` (`id`, `at_ms`, `kind`, `descriptor`, `key`), after which the twin blocks for one `model_response` (`id`, `status`, `latency_ms`, `body`) |
| twin → harness, trace | `cycle`, `goal`, `force_stop`, `utterance`, `suppressed`, `announce`, `backoff`, `speak_not_offered`, `note_offered`, `note`, `note_not_offered`, `wheels`, `head`, and a final `done` |

World events whose `at_ms` falls inside a reply's latency window are applied
before the reply is handled, which is what lets a scenario put a voice turn in
the middle of a planner request (US-A4). `cycle` records use the field names of
the planner log line in `planner_task.c` (`why`, `step`, `still`, `scene`,
`gate`, `spent_*`).

### The executor

The executor ticks every 33 ms from `t = 0`, after any world event, voice-turn
end or microphone frame at the same instant. Ticks inside a reply's latency
window run against the previous goal, as on the robot. There is no switch that
turns it off; two `config` keys add inputs to it:

| `config` key | Default | Effect |
|---|---|---|
| `head` | `false` | Servos present: the executor aims the head and `head` records are traced. Each planner tick samples the servo position and `head_settled`, as `planner_task.c` does at capture, for the frame's pose (`-` when unsettled or between dwell poses) and the goal's head angles. The idle gaze faces forward while a voice turn runs. `pose` on a `frame` is ignored |
| `cliff` | `false` | Cliff reflex armed. It starts as "no floor" under both sensors and holds the wheels until `floor` events say otherwise |

The world events that feed it:

| Event | Fields |
|---|---|
| `range` | `cm` (2..400, the bounds `ultrasonic_core_decode()` accepts), optional `sensor`: `front` (default) or `rear` |
| `range_fail` | optional `sensor`; the sensor's reads fail from then on |
| `floor` | `front`, `rear`: whether there is floor under each sensor |

A rangefinder ranges what is in front of it at its trigger and is read 132 ms
later; triggers are 165 ms apart (`ULTRASONIC_CONVERSION_MS` plus a tick to
read and a tick to re-trigger). A range held for less than 165 ms can fall
between two triggers and never be read. Until the first `range` event for a
sensor its reads fail. The obstacle reflex is the executor's reaction to a
front reading, so a scenario scripts the obstacle; there is no `reflex` event.

The planner's evidence comes from `reactive_controller_get_telemetry()`, as in
`planner_task.c`: range, range validity, `robot_moving` and reflex trips. Its
tick at `t = 0` runs before any read has completed and sees the executor's boot
value (400 cm, valid), which `plan_activity` adopts as its range reference. A
scenario that scripts a range at 0 s therefore wakes the planner on its next
tick with `R`, and request counts include that request.

Two trace records, written when the state changes:

```json
{"t":"wheels","at_ms":20262,"left":{"drive":"brake","duty":0},"right":{"drive":"brake","duty":0}}
{"t":"head","at_ms":4290,"pan_deg":-24,"tilt_deg":0,"mode":"gaze","settled":false}
```

`drive` is `forward`, `reverse`, `coast` (IN1 = IN2 = low) or `brake`
(IN1 = IN2 = high), decoded from the direction channels `motor_controller.c`
wrote; `duty` is the PCA9685 count, 0..4095, after the driver's 300 ms rise
ramp. Both wheels coast at duty 0 after `motor_controller_init()`; that state
is not traced. With `head` set the first `head` record is the state after
tick 0. `mode` is `reactive_head_mode_name()`.

The virtual clock is 64-bit milliseconds, like `esp_timer`; firmware modules
take their own `uint32` casts, so a run can cross the day-49 wrap. Microphone
frames are noted at the end of their 64 ms, as `ambient_listener.c` stamps
them, so a sound starting exactly on a tick is heard by the next tick, not
that one. A `voice_turn` holds the microphone only for its `record_ms` (the
whole turn when omitted); a second turn requested while one runs is queued
and starts when it ends, as the depth-1 queue does once the task has taken
the first. The field-level details are in the
[harness README](harness/README.md) and in `src/twin_main.c`.

## Adding a scenario

1. Pick the user story in PRD-017 and write `scenarios/us-<id>-<slug>.yaml`:
   a world timeline, a model binding and `expect:` assertions. The format and
   the assertion list are in the [harness README](harness/README.md#scenario-format).
2. Work out the expected timings from the firmware (`plan_activity.c` for the
   ladder, `planner_task.c` for announces, `reactive_controller.h`,
   `ultrasonic_core.h`, `cliff_core.h` and `head_aim.h` for the executor), and
   say in the file's header comment where each number comes from. The planner
   boots dormant, so a scenario needs an event before it plans at all; a
   `range` at 0 s is one (see [The executor](#the-executor)).
3. `cd harness && uv run python -m robocar_twin check --scenarios ../scenarios/<file>`
   validates it without running it; `just robocar-twin::test` runs it.
4. Check that it can fail: break the behaviour it targets under `../main/`
   (`planner_cycle_core.c` for a planner story, `reactive_controller.c`,
   `cliff_core.c` or `idle_gaze.c` for an executor one), rebuild, and confirm
   the scenario goes red, then restore.

## Model replies and cassettes

A scenario binds `models.planner` to `inline` (replies written in the
scenario, including a shorthand for Gemini `functionCall` parts) or to
`replay:<dir>`, which reads `<dir>/<key>.json` (`{status, latency_ms, body}`)
and fails, naming the key and descriptor, when a cassette is missing. Reply
bodies are raw `generateContent` JSON parsed by the firmware's
`gemini_parse_response()`. The phase-1 scenarios all use `inline`; there are
no recorded cassettes yet.

Recording (`RecordPort`, behind `GEMINI_API_KEY` and a spend guard) exists in
the harness but has no request body to send in phase 1: the firmware's request
builder still lives in `gemini_backend.c`, and its extraction to
`gemini_request.c` (ADR-030 §2) is what makes live recording possible.

## Limitations

- **The request key is a proxy.** It is the SHA-256 of a canonical
  descriptor the twin builds (`gate`, `kind`, `pose`, `scene`,
  `speak_allowed`; sorted keys, no whitespace), not of the request body the
  firmware would send. It becomes the body hash once `gemini_request.c` is
  extracted, and any cassettes are re-recorded then.
- **Voice turns are scripted world events.** The twin glue mirrors what
  `voice_turn.c` does to the counters the planner reads (turn count, busy flag,
  `speech_budget_notes()`), with line citations in `src/twin_main.c`;
  `voice_turn_core` is phase 2.
- **Not modelled:** TTS playback time (a planner line never quarantines the
  microphone), the start cue, manual leases, and the microphone's quarantine
  after a head movement (`ambient_head_quiet()`): with `head` set the twin
  hears sounds the robot's listener would drop while the head moves.
- **The head is modelled only with `head` set.** Without it the servos read as
  absent, the head as unavailable, and a frame's pose is the scripted one.
- **The world does not move with the robot.** Range and floor readings are
  what the scenario scripts, whatever the wheels do: a back-off from an edge
  does not bring the floor back, and a drive toward a wall does not shorten
  the range.
- **The boot range value reaches the planner.** Its first tick reads 400 cm
  before any rangefinder read has completed. Whether the robot's first cycle
  does the same depends on how long its first camera capture takes, which is
  unmeasured (ADR-036, open question 2).
- **A link coming up does not wake the planner**, matching `main.c`, so a
  scenario that expects a request on link-up depends on the dormancy state at
  that moment.
