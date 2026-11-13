---
id: PRD-017
title: Robocar Behaviour Twin — Scenario-Driven UX Evaluation
status: draft
created: 2026-11-05
---

# PRD-017: Robocar Behaviour Twin — Scenario-Driven UX Evaluation

**Status**: draft
**Created**: 2026-11-05
**Confidence**: 5/10, self-assessed. The user stories are drawn from the
firmware docs, ADRs and issues; none has been run as a scenario. Unverified:
how much of `planner_task.c` and `voice_turn.c` separates cleanly into
host-compilable cores (ADR-030 §Firmware seams), and whether a scripted world
is rich enough for the planner prompt to behave as it does on the bench.
**Architecture**: [ADR-030](../decisions/ADR-030-robocar-behaviour-twin.md),
[ADR-036](../decisions/ADR-036-robocar-twin-plant-port.md) (the executor in
the twin, and the optional physics plant)

---

## Overview

Teuvo (`packages/robotics/robocar`) is judged by how he behaves in a room: when
he speaks, what he says, whether he answers a person, when he goes quiet, where
he looks. Today that behaviour is evaluated by watching the bench and reading
monitor logs. This PRD defines a set of user stories for that behaviour and a
**behaviour twin**: a host build of the firmware's decision logic, driven by
scripted scenarios and a pluggable model backend, whose output is a trace that
deterministic checks and model judges can score.

With the twin, a change to a prompt, a persona, a gate threshold or the model
behind the planner can be run against every story in minutes, by a person, by
CI or by an agent, and compared against the previous version as a scorecard.

## Problem

- **Behaviour is found on the bench.** The noise remark spoken on top of a
  voice-turn reply (2026-11-05, `planner_task.c` "person spoke during the
  request"), the VAD turn that lost its first word (#616), the gate that failed
  open with no microphone (ADR-022) and the head that never moved (ADR-029)
  were each found by watching the robot or reading a capture. Each fix is held
  by a module-level unit test, but nothing checks the interaction between
  modules over minutes of time.
- **The host test suites test modules, not stories.** They pin the
  millisecond wrap, a gate's threshold, a parser's edge case. None runs the
  planner loop, the voice turn and the speech gates together against a
  timeline, so "a person asks a question while the planner is mid-request"
  is not testable without a person.
- **Models cannot be compared.** The planner is pinned to Gemini Robotics-ER
  and voice turns to `gemini-flash-latest`. A new model version, a cheaper
  tier or another provider can only be judged by flashing it and watching.
- **Live evaluation spends the quota the robot runs on.** The planner already
  saturates the Robotics-ER free tier (`docs/firmware/planner.md`); bench
  experiments compete with the robot itself.
- **The simulation does not model this.** `packages/robotics/robocar/simulation`
  (PRD-003) is a plant: wheels, contacts, rangefinders and floor sensors
  (ADR-036). It models motion, not the planner, speech gates or voice turns.

## Users

| User | Needs |
|------|-------|
| Lauri | Decide whether a prompt, persona or model change made Teuvo better, before flashing |
| Coding agents | A closed loop: change, run, read a scorecard, iterate, open a PR with evidence |
| CI | A deterministic, free, must-pass subset on every PR touching the firmware (`packages/robotics/robocar/main`) or the twin |

## User stories — Teuvo

Each story names its oracle: **D** = deterministic check on the trace (CI-safe),
**J** = rubric scored by a judge model (advisory). The source column points at
the ADR, issue or doc the behaviour comes from; the story is the contract that
source already states, written so a scenario can test it.

### A. Being spoken to

| ID | Story | Acceptance | Oracle | Source |
|----|-------|------------|--------|--------|
| US-A1 | As a person in the room, when I say "Teuvo, what do you see?", he answers about what the camera shows | A voice turn starts within 150 ms of sustained speech plus task latency; the clip begins ≥ 1 s before the trigger; the reply refers to objects in the frame fixture | D + J | voice.md, #616 |
| US-A2 | As a person, when I ask a follow-up within 7 s of his reply, I do not have to say his name again | Second utterance inside the follow-up window starts a turn; history carries the first exchange | D | ADR-024, voice.md |
| US-A3 | As a person, when people talk among themselves near him, he does not butt in | Speech not addressed to him yields `__IGNORE__` and no audio; hands-free turns stay ≤ 12 per rolling 5 min | D + J | #623, voice_budget |
| US-A4 | As a person, when I ask a question while he is mid-thought, I hear one answer, not a remark about "a strange noise" on top of it | No planner utterance is spoken when a voice turn ran or started during the planner request | D | planner_task.c (2026-11-05) |
| US-A5 | As a person, a door slam, a dropped object or his own motors do not start a conversation | Broadband transients and in-cue tones do not trigger a turn | D | #617, speech_trigger |
| US-A6 | As a person, when I type `listen`, he always answers, even when the hands-free ration is spent | `listen` bypasses the ration and counts against the per-boot ceiling (200) | D | #623 |

### B. Speaking unprompted

| ID | Story | Acceptance | Oracle | Source |
|----|-------|------------|--------|--------|
| US-B1 | As a person, when something new appears in view, he remarks on it | Scene score crosses threshold at the current dwell pose; `speak` is offered; one line is spoken | D + J | ADR-020 |
| US-B2 | As a person, he does not repeat himself | No spoken line is a near-repeat of the recall ring; judge rates variety across a 30 min scenario | D + J | dialogue_style |
| US-B3 | As a person, in an unchanging room he falls quiet rather than narrating the same table | No utterance after the scene and audio references are marked spoken and nothing changes | D | ADR-020, scene_change |
| US-B4 | As a person, a sound he cannot identify is described as a change of unknown source, not guessed at | Audio clause says only that the sound changed; judge checks the line makes no claim about the source | D + J | #618, #631 |
| US-B5 | As a person, he stays in his persona (`en-default`, `fi-1950`) for wording, register and delivery tags | Every delivery tag is on the `speech_tags.h` allow-list; judge rates persona fit | D + J | voice_persona, ADR-019 |

### C. Presence and attention

| ID | Story | Acceptance | Oracle | Source |
|----|-------|------------|--------|--------|
| US-C1 | As a person, when nothing directs him, he looks around rather than staring ahead | With no fresh `track` goal or lease, the head visits dwell poses; frames between poses get pose `-` | D | ADR-029 |
| US-C2 | As a person, when I hold a ball in view, his head follows it and the wheels follow the head | `track` goals latch once per write; the executor chooses head, wheels or both | D | ADR-025 |
| US-C3 | As an onlooker, I can tell what he is doing from his LEDs and OLED | Activity trace colours match the state timeline | D | activity-trace.md |
| US-C4 | As a person, when I speak to him he faces me while he listens instead of looking away | While a voice turn runs the head is at centre within 20 ticks (660 ms) and holds there; the dwell starts afresh when the turn ends | D | ADR-029 §1, idle_gaze |
| US-C5 | As a person, when he sets off after looking to one side, he goes where he was looking | A `drive` goal planned on a frame taken at a side dwell pose carries the head's pan at capture, and the executor adds it to the heading: heading 0 gives unequal wheel duties, the slower wheel on the side the head was turned to, and they do not change when the head moves on | D | ADR-025, head-aiming.md, planner_task.c |

### D. Safety

| ID | Story | Acceptance | Oracle | Source |
|----|-------|------------|--------|--------|
| US-D1 | As an owner, he brakes before hitting something in front of him | From a range step below `STOP_THRESHOLD_CM`, brake within `REACTIVE_REFLEX_WORST_CASE_MS` (297 ms), whatever the goal | D | test_reflex_latency |
| US-D2 | As an owner, a stale or missing goal never leaves him driving | Goal TTL expiry holds position; camera failure forces STOP | D | ADR-016 |
| US-D3 | As an owner, he does not back into something behind him | A rear range below `STOP_THRESHOLD_CM` brakes a reverse at the first read of a trigger that ranged it (at most 297 ms after it appears) and holds it until the rear clears; the obstacle reflex does not trip | D | #1026, reactive_controller.c |
| US-D4 | As an owner, he stops at an edge, backs away from it and does not drive at it again | Two no-floor samples brake; after `CLIFF_BRAKE_MS` he backs off for `CLIFF_BACKOFF_MS`; motion toward a sensor with no floor is refused until three floor samples | D | cliff-sensors.md, cliff_core |
| US-D5 | As an owner, when he is driven at a wall he stops short of it and stays there | Against the physics plant: a `drive` goal toward a wall 1 m away ends with both wheels braked, the chassis never in contact with the wall, and no movement afterwards. Ordering and wide bounds, not distances, until the plant is calibrated | D | ADR-036 §4, reactive_controller.c |
| US-D6 | As an owner, on a table he never puts a wheel over the edge | Against the physics plant, on a floor with no walls and the cliff reflex armed: he reaches the edge, brakes, backs off and returns, and no wheel is ever outside the floor. Ordering and wide bounds, as for US-D5 | D | ADR-036 §4, cliff_core |

### E. Quiet and economy

| ID | Story | Acceptance | Oracle | Source |
|----|-------|------------|--------|--------|
| US-E1 | As an owner who leaves him plugged in overnight, he stops calling the API | Request interval climbs 15 → 30 → 60 → 120 → 300 s, then dormant; ≤ 5 requests over the ladder | D | ADR-022 |
| US-E2 | As an owner, he wakes within one tick when I walk in or speak | A scene, audio, range or console event while dormant yields a request on the next 15 s tick | D | ADR-022 |
| US-E3 | As an owner, a runaway never costs more than the fuse | Planner and voice fuses trip at their ceilings and stay tripped until `plan resume` / `voice resume` | D | plan_budget, voice_budget |

### F. Degraded conditions

| ID | Story | Acceptance | Oracle | Source |
|----|-------|------------|--------|--------|
| US-F1 | As an owner, with no WiFi or no key he holds still and plans at once when either arrives | No request, no backoff, STOP held; first cycle after link/key plans | D | #845, #846, ADR-028 |
| US-F2 | As an owner, a 429 from the model backs off instead of retrying at full rate | Backoff doubles to `PLANNER_MAX_BACKOFF_PERIODS`, resets on success | D | planner_task.c |
| US-F3 | As a person, a malformed or truncated model reply never makes him move unpredictably | Parse failure forces STOP; no goal is written | D | gemini_parse |

## User stories — the harness

| ID | Story | Acceptance |
|----|-------|------------|
| H-1 | As a developer, I run every story offline in under 2 minutes | `just robocar-twin::test` runs the scripted D-oracle scenarios and `::test-physics` the ones that need the plant, against recorded model replies; no network |
| H-2 | As a developer, I see which stories a change broke and why | Per-scenario pass/fail with the failing assertion and the trace excerpt |
| H-3 | As a developer, I compare models on identical inputs | One command runs the scenario set against N model adapters and writes a scorecard (pass rate, judge score with spread, latency, tokens, cost) |
| H-4 | As an agent, I iterate on a prompt or persona and show the effect | Scorecard diff against the `main` baseline, attachable to a PR |
| H-5 | As an owner, a bench capture becomes a regression scenario | A converter turns a monitor/MQTT log plus `snap` dumps into a scenario file |
| H-6 | As an owner, live evaluation cannot run away with spend | Per-run request and token ceilings, cache by request hash, a dry-run cost estimate |

## Requirements

| ID | Requirement |
|----|-------------|
| FR-1 | The twin executes the firmware's own decision code compiled for the host (shared-core pattern, `.claude/rules/testing.md`), never a reimplementation |
| FR-2 | Time is virtual: a 30 min scenario runs in seconds and is reproducible bit-for-bit given a seed |
| FR-3 | The model call is the only seam: the twin emits the exact request body the firmware would send and parses the reply with `gemini_parse.c` |
| FR-4 | Scenarios are declarative files: a world timeline (frames, ranges, sounds, speech, console lines), a persona, model bindings and expectations |
| FR-5 | The trace is JSONL using the field names of the firmware's log lines (`scene:`, `still:`, `gate:`, `why:`, `spent:`), so twin traces and bench logs are comparable |
| FR-6 | Model adapters: replay (cassettes keyed by request hash), live Gemini, and other providers through a translating adapter |
| FR-7 | Judged rubrics are versioned files; the judge model and rubric version are recorded in every scorecard |
| FR-8 | D-oracle scenarios with replay cassettes run in CI on every PR touching the firmware (`packages/robotics/robocar/main`) or the twin |

## Non-goals

- Motor dynamics, PWM, I2C timing: not required of the twin; the host unit
  tests cover these. The twin's executor runs the firmware's motor driver, so
  a trace carries the duty it wrote, and what the motors do with that duty is
  available through the optional physics plant (ADR-036).
- Audio quality of TTS or voice FX: the twin carries text and delivery tags, not PCM.
- Photorealistic or physics-accurate worlds: not required of the twin. A
  scripted timeline is its model of the room (ADR-030 §World), and it builds
  and runs with nothing else. A physics world is available through the
  optional plant, for the scenarios that name it (ADR-036 §3); camera frames
  stay scripted either way.
- Replacing bench testing. The twin narrows what the bench has to find.

## Success metrics

- Every story in this PRD has at least one scenario; every D story runs in CI.
- The next behaviour bug found on the bench is reproduced as a failing scenario
  before it is fixed.
- A model or prompt change PR carries a scorecard diff.

## Phases

1. **Tracer bullet**: planner cycle core extracted, twin binary with virtual
   clock, replay adapter, scenarios for US-E1, US-E2, US-A4. CI job.
   *Progress (2026-11-06):* built in `packages/robotics/robocar/twin`; US-E1, US-E2,
   US-A4, US-F2 and the planner half of US-E3 pass under
   `just robocar-twin::test` (CI: `robocar-twin.yml`) with inline replies.
   The voice fuse half of US-E3, and `plan resume` / `voice resume`, wait for
   voice_turn_core (phase 2): `voice_budget` is not in the twin yet.
   The request key is a proxy until `gemini_request.c` is extracted.
2. **Voice path**: voice-turn core and listener trigger driven by scripted
   speech events; US-A1–A6, US-B*.
3. **Live and judged**: live Gemini adapter with spend guard, rubric judge,
   scorecard and model matrix (H-3, H-4).
4. **Field loop**: capture-to-scenario converter (H-5), head/gaze stories with
   a kinematic stub (US-C*).
   *Progress (2026-11-13, ADR-036 §1):* the twin now links the firmware's
   executor and ticks it every 33 ms, so the head and reflex stories need no
   stub. The planner reads range, `robot_moving` and reflex trips from the
   executor's telemetry, the trace carries `wheels` and `head` records, and
   scenarios script range readings (front and rear) and floor readings in
   place of the removed `reflex` event. US-D1, US-D3, US-D4, and US-C1 with
   US-C4 pass as scenarios under `just robocar-twin::test`. US-C1's scenario
   pins where the head rests, when it first moves and the pose of a frame
   taken before the head settled; which pose is visited when depends on the
   seed and is not asserted. US-C2 (a `track` goal leading the head), US-C3
   and US-D2 have no scenario yet. The kinematic stub is not built: ADR-036
   replaces it with the optional physics plant.
   *Progress (2026-11-13, ADR-036 §3 and §4):* the plant port is built. A
   scenario names `plant: physics` with an arena and a start pose; the twin
   then asks for the world after every executor tick and the harness relays
   that to `robocar_sim.plant`, run as a child process, and adds `pose`
   records to the trace. US-D5 (a wall) and US-D6 (a table edge) pass under
   `just robocar-twin::test-physics`, which CI runs as the
   `robocar-twin-physics` suite; `just robocar-twin::test` runs the scripted
   scenarios and never starts the plant. Both physics scenarios drive at 40 %:
   on the plant's estimates the robot reaches the wall at 80 %, and the margin
   at the table edge falls from 3.8 cm to 0.9 cm between 40 % and 80 %. None
   of the plant's figures is measured (ADR-036, open question 1), so those are
   properties of the estimates, not of the robot. US-C5 passes as a scripted
   scenario: it pins the head angles the planner samples at capture, which no
   scenario held before. The pose the head is at when the planner ticks
   depends on the seed, so its seed was found by running it.
