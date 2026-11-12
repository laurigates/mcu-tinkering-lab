# ADR-030: Robocar Behaviour Twin

**Status**: proposed
**Date**: 2026-11-05
**Confidence**: 5/10 (the firmware seams below are read from the code, not
yet extracted; scenario and trace formats are sketches until the tracer
bullet runs)
**Requirements**: [PRD-017](../requirements/PRD-017-robocar-behaviour-twin.md)
**Extends**: [ADR-016](ADR-016-hierarchical-ai-controller.md),
[ADR-020](ADR-020-ambient-audio-speech-gate.md),
[ADR-022](ADR-022-planner-dormancy-and-spend-ceiling.md),
[ADR-024](ADR-024-conversational-entity-and-multimodal-voice.md)

---

## Context

Teuvo's behaviour comes from the interaction of the planner loop
(`planner_task.c`), the speech gates (`scene_change`, `ambient_audio`,
`speech_budget`), the dormancy ladder (`plan_activity`), the voice turn
(`voice_turn.c`, `ambient_listener.c`, `speech_trigger`) and the executor
(`reactive_controller.c`, `idle_gaze`, `head_aim`). Most of these modules
already compile on the host: `packages/robotics/robocar/test/` builds over
forty suites against shims for FreeRTOS, `esp_timer`, `esp_random`, NVS and the
PCA9685 (`test/include/`). What does not compile on the host is the glue
that sequences them: the planner task body, the voice-turn task, and
`gemini_backend.c`, which mixes the request builder with `esp_http_client`.

PRD-017 asks for scenario-level evaluation of that behaviour, offline and
deterministic for CI, live and judged for model comparison.

## Decision

Build a **behaviour twin**: a host executable compiled from the firmware's own
sources, plus a Python orchestrator that feeds it scenarios, answers its model
calls and scores its trace.

```
scenario.yaml ──► orchestrator (Python, uv, pytest)
                    │  world events (JSONL, stdin)
                    ▼
                 robocar-twin (C, host build of main/*_core.c + gates)
                    │  virtual clock, shims from test/include
                    │  model_request {body}  ──►  model port
                    │  ◄── model_response {raw}    ├─ replay  (cassette by sha256(body))
                    │                              ├─ gemini  (live, spend-capped)
                    │                              └─ other   (translating adapter)
                    ▼
                 trace.jsonl ──► D-checks (pytest) ──► scorecard.json
                             └─► J-rubric (judge model, N repeats)
```

### 1. The twin runs firmware code, not a model of it

Every decision the twin makes is a call into a function that also ships in
the firmware: the shared-core pattern in `.claude/rules/testing.md`. A
Python reimplementation of the gates would drift from the C within weeks; the
removed dual-ESP32 simulation (`packages/robotics/robocar/simulation`) is an example
of a parallel model that outlived the design it described.

### 2. Firmware seams (behaviour-preserving extractions)

Each is a refactor of the kind already done for `oled_status_core`,
`net_supervisor_core` and `log_mirror_core`, landed with its own host tests
before the twin uses it.

| Seam | From | Contents | Unblocks |
|------|------|----------|----------|
| `planner_cycle_core.c` | `planner_task.c` steps 1c–4 | gather evidence, gate verdict, speech suppression (repeat, person-spoke-during-request), goal write vs force-stop, backoff arithmetic | US-A4, US-B*, US-E*, US-F* |
| `gemini_request.c` | `gemini_backend.c` | `build_tools()`, `build_request_json()`, `build_narrate_json()`; transport stays in `gemini_backend.c` | FR-3, model comparison |
| `voice_turn_core.c` | `voice_turn.c` `run_turn()` | budget checks, request body, `__IGNORE__` handling, history update, follow-up window | US-A1–A3, US-A6 |
| listener driver | `ambient_listener.c` | feed level/band-share frames instead of PDM reads, so `speech_trigger` and `voice_endpoint` run on scripted speech | US-A1, US-A5 |

The task bodies keep only FreeRTOS scheduling, camera buffers and HTTP. The
twin's main loop replaces them and advances a virtual clock in 33 ms executor
ticks and 15 s planner ticks.

### 3. World: a scripted timeline, not a physics engine

A scenario lists timed events. Frames are either a fingerprint (enough for
gate-only stories) or a JPEG fixture (needed when a model must see the frame;
`snap` dumps from the robot are the source). Speech is a duration plus
level and band share for the trigger, with a text and an optional WAV fixture
for the model. Motion is a kinematic stub (goal → pose → range) added in
phase 4 for the head and gaze stories. Pymunk is not adopted: the stories in
PRD-017 depend on timing and evidence, not on contact dynamics.

```yaml
id: us-a4-question-during-plan
stories: [US-A4]
persona: en-default
seed: 7
duration: 60s
world:
  - {at: 0s,    frame: {jpeg: fixtures/frames/kitchen-table.jpg, pose: C}}
  - {at: 14s,   sound: {kind: door, peak_db: 22}}          # wakes the planner
  - {at: 15.6s, speech: {text: "Teuvo, what's on the table?", dur: 1.8s,
                         level_db: 15, band_pct: 82, wav: fixtures/speech/table-q.wav}}
models:
  planner: replay:cassettes/us-a4
  voice:   replay:cassettes/us-a4
expect:
  - no_utterance: {source: planner, between: [15s, 30s]}
  - utterance:    {source: voice_turn, within: 6s, after: speech[0]}
rubric: [answers_question, in_persona]
```

### 4. The trace uses the firmware's own field names

One JSONL record per decision: `plan` (with `why`, `step`, `scene`, `still`,
`loud`, `gate`, `spent` exactly as in the planner log line), `goal`,
`utterance` (source, text, tags), `voice_turn`, `suppressed` (reason),
`reflex`, `head`. Because the fields match the log, a bench log parsed by the
same reader is a trace, which makes H-5 (capture to scenario) a converter
rather than a second format.

### 5. The model port is the only non-deterministic seam

- **replay**: cassette keyed by `sha256(request body)`. A prompt change
  changes the hash and fails loudly as "no cassette", never silently replaying
  an answer to a different prompt. Cassettes are recorded by a live run and
  committed; images in a body are hashed, not stored twice.
- **gemini**: live calls with a per-run ceiling on requests and tokens,
  read from `usageMetadata` the way `plan_budget` does; `--dry-run` prints the
  request count the scenario set will make.
- **other providers**: an adapter translates the Gemini body (contents,
  `tools`, function declarations) to the provider's format and the reply back
  to Gemini's `functionCall` shape so `gemini_parse.c` reads it unchanged. The
  translation is harness code and is lossy (Robotics-ER's box coordinates have
  no equivalent); scorecards name the adapter.

### 6. Evaluation tiers

| Tier | What | Where | Gate |
|------|------|-------|------|
| T0 | Host unit tests (existing) | CI | must pass |
| T1 | D-oracle scenarios, replay cassettes | CI, every PR touching the firmware (`packages/robotics/robocar/main/`) | must pass |
| T2 | Same scenarios, live model, records cassettes | local / manual workflow | advisory |
| T3 | J-rubrics over T2 traces; judge model pinned, rubric versioned, N=3 repeats, mean and spread reported | local / manual workflow | advisory |
| T4 | Bench (HIL) | Lauri | final |

A scorecard is the T1 pass list plus, when run, T2/T3 numbers per model:
pass rate, judge score ± spread, p50/p95 model latency, tokens, cost. The
agent loop in PRD-017 H-4 is: change → T1 green → T2/T3 against the same
scenarios → scorecard diff against `main`'s last scorecard → PR.

### 7. Layout

```
packages/robotics/robocar/twin/
├── CMakeLists.txt        # builds robocar-twin from ../main + ../test/include
├── src/twin_main.c       # virtual clock, event loop, JSONL in/out
├── harness/              # uv project: orchestrator, model port, checks, judge, scorecard
├── scenarios/*.yaml
├── rubrics/*.md
├── cassettes/
├── fixtures/{frames,speech}/
└── justfile              # test (T1), live (T2), judge (T3), matrix, capture-to-scenario
```

## Alternatives considered

- **Extend `packages/robotics/robocar/simulation`**: built for the removed dual-ESP32
  design, its AI path is Claude/Ollama over an I2C protocol that no longer
  exists. Reusing it means rewriting the parts that matter here.
- **Reimplement the behaviour in Python**: fastest to start, drifts from the
  firmware, and a passing scenario would say nothing about the shipped code.
- **Emulate the whole firmware in QEMU (ESP32-S3)**: runs the real binary,
  but the camera, PDM microphone, I2S and PCA9685 are not emulated, time is
  not virtual, and a 30 min scenario takes 30 min.
- **Bench only, with log analysis**: the current state; slow, spends the
  robot's own quota, and cannot compare models on identical inputs.

## Consequences

- Four extractions in `main/` before the twin is useful; each is reviewed and
  tested on its own, and the firmware build must stay byte-for-byte
  equivalent in behaviour (existing host tests plus a bench smoke test).
- A new rule for the firmware: decision logic added to a task body belongs in
  its `*_core.c`, or the twin cannot see it.
- Cassettes become reviewable artefacts: a prompt change shows up as a
  cassette diff in the PR.
- Judged scores are noisy; they inform, they do not gate. Promoting a J-check
  to a gate requires a recorded agreement rate against Lauri's own ratings.

## Open questions

1. Judge model and provider for T3 (Opus, Gemini Pro, or a PAL consensus of
   several), and whether Lauri rates a calibration set first.
2. Whether T2/T3 run on a schedule in CI with a Gemini key secret, or only
   locally.
3. Speech fixtures for live voice turns: record real utterances, or synthesise
   them once with Gemini TTS and commit the WAVs.
4. Whether `packages/robotics/robocar/simulation` is retired once the twin exists.
