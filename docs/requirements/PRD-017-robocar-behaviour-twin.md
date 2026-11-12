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
**Architecture**: [ADR-030](../decisions/ADR-030-robocar-behaviour-twin.md)

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
- **The existing simulation does not model this.** `packages/robotics/robocar/simulation`
  (PRD-003) is a Pymunk physics model of the removed dual-ESP32 design, with a
  Claude/Ollama backend and an I2C bridge. It models motion, not the planner,
  speech gates or voice turns.

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

### D. Safety

| ID | Story | Acceptance | Oracle | Source |
|----|-------|------------|--------|--------|
| US-D1 | As an owner, he brakes before hitting something in front of him | From a range step below `STOP_THRESHOLD_CM`, brake within `REACTIVE_REFLEX_WORST_CASE_MS` (297 ms), whatever the goal | D | test_reflex_latency |
| US-D2 | As an owner, a stale or missing goal never leaves him driving | Goal TTL expiry holds position; camera failure forces STOP | D | ADR-016 |

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
| H-1 | As a developer, I run every story offline in under 2 minutes | `just robocar-twin::test` runs all D-oracle scenarios against recorded model replies; no network |
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

- Motor dynamics, PWM, I2C timing: the host unit tests cover these.
- Audio quality of TTS or voice FX: the twin carries text and delivery tags, not PCM.
- Photorealistic or physics-accurate worlds. A scripted timeline and a
  kinematic stub are the model of the room (ADR-030 §World).
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
