# robocar_twin — scenario harness

Python orchestrator for the robocar behaviour twin ([PRD-017](../../../../../docs/requirements/PRD-017-robocar-behaviour-twin.md),
[ADR-030](../../../../../docs/decisions/ADR-030-robocar-behaviour-twin.md),
[ADR-036](../../../../../docs/decisions/ADR-036-robocar-twin-plant-port.md)). It loads
`../scenarios/*.yaml`, drives the `robocar-twin` binary over JSONL on
stdin/stdout, answers its model requests, and checks the trace.

```sh
uv sync
uv run pytest                                    # unit tests + every scenario
ROBOCAR_TWIN_ALLOW_MISSING=1 uv run pytest       # skip scenarios when the binary is not built
uv run python -m robocar_twin run --scenarios ../scenarios/*.yaml --out scorecard.json
```

The binary is `$ROBOCAR_TWIN_BIN`, or `../build/robocar-twin` relative to the
twin directory.

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
and says to script the obstacle.

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

`wheels` and `head` records report a state when it changes, so each has two
assertions:

| Assertion | Checks | Filters |
|---|---|---|
| `wheels` | how many `wheels` records in the window match (at least one unless `count`, `min` or `max` is given) | `left`, `right`: `forward`, `reverse`, `coast`, `brake`; `left_duty`, `right_duty`: PCA9685 counts |
| `wheels_at` | the wheels at `at:`, that is the last record at or before it; before the first record both wheels coast at duty 0 | the same; at least one |
| `head` | how many `head` records in the window match | `pan`, `tilt`: degrees; `mode`: `gaze`, `centre`, `aim`, `turn`, `lease`, `unavailable`; `settled`: true/false |
| `head_at` | the head at `at:`; fails when there is no `head` record at or before it | the same; at least one |

A numeric filter is `[lo, hi]`, inclusive, or one number. `at:` is a duration
or a `<kind>[i]` reference to a world event, like the window times.

```yaml
expect:
  - wheels: {left: brake, right: brake, after: "range[1]", within: 297ms, count: 1}
  - wheels_at: {at: 21.417s, left: forward, left_duty: [2400, 2500]}
  - head: {settled: true, pan: [1, 39], count: 0}
  - head_at: {at: 21.087s, pan: 0, tilt: 0, settled: true}
```
