# robocar_twin — scenario harness

Python orchestrator for the robocar behaviour twin ([PRD-017](../../../../../docs/requirements/PRD-017-robocar-behaviour-twin.md),
[ADR-030](../../../../../docs/decisions/ADR-030-robocar-behaviour-twin.md),
[ADR-036](../../../../../docs/decisions/ADR-036-robocar-twin-plant-port.md)). It loads
`../scenarios/*.yaml`, drives the `robocar-twin` binary over JSONL on
stdin/stdout, answers its model requests, and checks the trace.

```sh
uv sync
uv run pytest -m "not physics"                   # unit tests + every scripted scenario
uv run pytest -m physics                         # the scenarios that start the physics plant
ROBOCAR_TWIN_ALLOW_MISSING=1 uv run pytest       # skip scenarios when the binary is not built
uv run python -m robocar_twin run --scenarios ../scenarios/*.yaml --out scorecard.json
```

The binary is `$ROBOCAR_TWIN_BIN`, or `../build/robocar-twin` relative to the
twin directory. A scenario whose `plant:` is `physics` gets the pytest marker
`physics` on its run (`test_scenario_passes`), read from the file at
collection time; its validation (`test_scenario_validates`) is unmarked. The
recipes that use the marker are `just robocar-twin::test` and `::test-physics`.

## Model ports

| Binding in `models.planner` | Port | Behaviour |
|---|---|---|
| `replay:<dir>` | `ReplayPort` | `<dir>/<key>.json` holding `{status, latency_ms, body}`; a missing file is a hard error naming the key and the descriptor |
| `inline` | `InlinePort` | the scenario's `responses:` in order, then `default:` |
| (programmatic) | `RecordPort` | live Gemini behind `GEMINI_API_KEY`, writes cassettes |

**The cassette key is a proxy in phase 1.** It is `sha256` of the canonical
request descriptor the twin builds (kind, frame scene, pose, gate verdict,
`speak_allowed`), serialised with sorted keys, no whitespace, UTF-8. It becomes
the hash of the real request body once `gemini_request.c` is extracted
(ADR-030 §2), and every cassette will be re-recorded then.

## The plant port

Where the executor's range and floor readings come from, chosen by the
scenario's top-level `plant:` key:

| `plant:` | Port | Behaviour |
|---|---|---|
| `scripted` (the default; the key may be omitted) | `ScriptedPlant` | the scenario's `range`, `range_fail` and `floor` events are the world. The twin is not told to ask a plant, and a `plant_request` from it is an error |
| `{physics: {arena, start}}` | `PhysicsPlant` | `robocar_sim.plant` runs as a child process. `"plant":"external"` goes into the twin's `config`, each `plant_request` is relayed as one `step`, and the `reading` comes back as the `plant_response` ([twin README](../README.md#the-plant)) |

```yaml
plant:
  physics:
    arena: arenas/wall-1m.yaml                   # against the scenario's twin directory, like replay:
    start: {x_cm: 0, y_cm: 0, heading_deg: 0}    # the axle midpoint and the way it faces
```

The loader checks that the arena file exists and holds a mapping, and that
`start` has exactly those three numbers, each finite. The arena is forwarded as it is: the
schema and its validation are the plant's
([simulation README](../../simulation/README.md)), and an arena it refuses
fails the run with the plant's message. With `physics` the loader refuses
`range`, `range_fail` and `floor` events, because the plant produces them.

The plant's command is `uv run --project <simulation> --locked --quiet python
-m robocar_sim.plant`, where `<simulation>` is the directory next to this
harness's twin directory, wherever the scenario file is.
`ROBOCAR_PLANT_CMD` replaces the command (split as a shell splits a line), and
`run_scenario(..., plant=)` takes a port directly, which the unit tests use
with `tests/fake_plant.py`. The plant gets the scenario's `seed`. A plant that
exits, writes an `error` line or answers out of order fails the scenario with
its message and its stderr, and the child process is reaped whether the run
passes or fails. The plant's start and every step count against the run's
wall-clock timeout, so build its environment first
(`just robocar-twin::test-physics` does).

A physics run adds `pose` records to the trace. Every scorecard entry carries
`plant` (`scripted` or `physics`), and a physics one also `plant_steps`, the
number of exchanges.

## Scenario format

```yaml
id: us-a4-question-during-plan
stories: [US-A4]
seed: 7
duration: 60s
config: {planner_period_ms: 15000}           # optional overrides
world:
  - {at: 0s,  frame: {scene: kitchen-table, pose: C}}
  - {at: 0s,  range: 120}                               # front rangefinder, cm
  - {at: 5s,  console: wake}
  - {at: 16s, voice_turn: {dur: 2s, record: 1s, reply: "A cup."}}  # record: mic held, default whole turn
  - {at: 5s, every: 5s, until: 300s, console: status}   # repeated event
models:
  planner: inline
responses:
  - {status: 200, latency: 4s, speak: "Some strange noise.", call: stop}
default: {status: 200, latency: 1s, call: stop}
expect:
  - suppressed: {reason: person_spoke, between: [15s, 20s], count: 1}
  - utterance: {source: voice_turn, within: 4s, after: "voice_turn[0]"}
```

`seed` is a whole number in 0..4294967295: the twin reads it into 32 bits and
the plant refuses a negative one.

`config` keys: `planner_period_ms`, `plan_budget_max_requests` and
`plan_budget_max_tokens` (integers), and two executor inputs that are off
unless set, `head` and `cliff` (true/false; what each does is in the
[twin README](../README.md#the-executor)).

World events, each with `at:` and optionally `every:` + `until:`:

| Event | Value |
|---|---|
| `frame` | `{scene, pose}`; `pose` is `C`, `L`, `R` or `F`, default `C`. With `config.head` a `pose` is refused: the pose comes from the head position at the planner tick, and the frame is sent without one |
| `range` | whole centimetres in 2..400 for the front sensor, or `{cm, sensor: front \| rear}` |
| `range_fail` | `{}`, or `{sensor: front \| rear}`: that sensor's reads fail from then on |
| `floor` | `{front, rear}` booleans, floor present under each sensor; needs `config.cliff` |
| `sound` | `{peak_db, dur}` |
| `voice_turn` | `{dur, reply, record}`; `record` is how long the microphone is held, default the whole turn |
| `console` | a console line |
| `link` | `{wifi, key}` |

A `range` should be held for at least 165 ms, one rangefinder read cycle, or a
trigger may never see it. There is no `reflex` event: the loader refuses one
and says to script the obstacle. `range`, `range_fail` and `floor` are refused
when `plant:` is `physics` ([The plant port](#the-plant-port)).

An inline response is either `{status, latency, body}` with a raw Gemini
`generateContent` body, or the shorthand `call:` (`stop`, `drive`, `track`,
`rotate`, with `args:`), `speak:` and `tokens:`, which the harness expands to
the `functionCall` shape `gemini_parse.c` reads. A `speak:` line in a reply
to a request that did not offer `speak` (descriptor `speak_allowed: false`)
is one the robot could never receive: the twin drops it and emits a
`speak_not_offered` record, so check for that rather than relying on it.
`note: {kind, text}` adds a planner `note` call (ADR-033); the same rule
applies, with `note_not_offered` when the request offered no note (descriptor
`note: none`).

Assertions: `utterance`, `no_utterance`, `suppressed`, `requests`,
`request_intervals`, `request_ladder`, `announce`, `force_stop`, `backoff`,
`goal`, `goals`, `speak_not_offered`, `note_offered` (filter `trigger`),
`note` (filters `trigger`, `kind`, `text_contains`), `note_not_offered`,
`model_request` (filters `gate`, `speak_allowed`, `scene`, `pose`, `note` from
the descriptor) and `cycle` (filters
`cause`, `requested`, `gate`, `pose`, `spent_req`, `spent_tok`). Each failure
names the trace records it is about.

`wheels`, `head` and `pose` records report a state when it changes, so each
has two assertions:

| Assertion | Checks | Filters |
|---|---|---|
| `wheels` | how many `wheels` records in the window match (at least one unless `count`, `min` or `max` is given) | `left`, `right`: `forward`, `reverse`, `coast`, `brake`; `left_duty`, `right_duty`: PCA9685 counts |
| `wheels_at` | the wheels at `at:`, that is the last record at or before it; before the first record both wheels coast at duty 0 | the same; at least one |
| `head` | how many `head` records in the window match | `pan`, `tilt`: degrees; `mode`: `gaze`, `centre`, `aim`, `turn`, `lease`, `unavailable`; `settled`: true/false |
| `head_at` | the head at `at:`; fails when there is no `head` record at or before it | the same; at least one |
| `pose` | how many `pose` records in the window match | `x_cm`, `y_cm`: centimetres in the arena's frame; `heading_deg`: degrees, -180..180; `collided`, `off_floor`: true/false |
| `pose_at` | the pose at `at:`; fails before the first reading, at 33 ms | the same; at least one |

`pose` and `pose_at` need `plant: physics`: a scripted run has no pose
records, so the loader refuses them there. A `pose` record is written when the
pose or a flag changes, so `pose: {collided: true, count: 0}` means the
chassis never touched anything, and `pose: {between: [30s, 40s], count: 0}`
that the robot did not move in that window.

A numeric filter is `[lo, hi]`, inclusive, or one number. `at:` is a duration
or a `<kind>[i]` reference to a world event, like the window times.

```yaml
expect:
  - wheels: {left: brake, right: brake, after: "range[1]", within: 297ms, count: 1}
  - wheels_at: {at: 21.417s, left: forward, left_duty: [2400, 2500]}
  - head: {settled: true, pan: [1, 39], count: 0}
  - head_at: {at: 21.087s, pan: 0, tilt: 0, settled: true}
  - pose: {off_floor: true, count: 0}
  - pose_at: {at: 40s, x_cm: [70, 91], collided: false}
```
