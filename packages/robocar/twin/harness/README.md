# robocar_twin — scenario harness

Python orchestrator for the robocar behaviour twin ([PRD-017](../../../../docs/requirements/PRD-017-robocar-behaviour-twin.md),
[ADR-030](../../../../docs/decisions/ADR-030-robocar-behaviour-twin.md)). It loads
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

An inline response is either `{status, latency, body}` with a raw Gemini
`generateContent` body, or the shorthand `call:` (`stop`, `drive`, `track`,
`rotate`, with `args:`), `speak:` and `tokens:`, which the harness expands to
the `functionCall` shape `gemini_parse.c` reads. A `speak:` line in a reply
to a request that did not offer `speak` (descriptor `speak_allowed: false`)
is one the robot could never receive: the twin drops it and emits a
`speak_not_offered` record, so check for that rather than relying on it.

Assertions: `utterance`, `no_utterance`, `suppressed`, `requests`,
`request_intervals`, `request_ladder`, `announce`, `force_stop`, `backoff`,
`goal`, `goals`, `speak_not_offered`, `model_request` (filters `gate`,
`speak_allowed`, `scene`, `pose` from the descriptor) and `cycle` (filters
`cause`, `requested`, `gate`, `spent_req`, `spent_tok`). Each failure names the
trace records it is about.
