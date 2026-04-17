# PRP: Hierarchical AI Controller (Planner + Reactive Executor)

**Status**: draft
**Source**: Conversation 2026-04-17, user observation that on-demand inference is a vestige of non-streaming Claude API era
**Priority**: P2
**Confidence**: 7/10
**Project**: `packages/esp32-projects/robocar-unified`

---

## Goal

Replace the current monolithic "AI-as-controller" loop with the hierarchical pattern Google promotes for Gemini Robotics-ER: a **slow planner** (cloud VLM, ~0.5–2 Hz) that emits structured goals/tool calls, and a **fast reactive executor** (on-device, 20–50 Hz) that owns motor commands and drives toward the current goal. The robot moves smoothly and continuously, with the planner redirecting rather than stepping it.

## Background

Current `ai_task` in `robocar-unified/main/ai_backend.c` captures a frame → uploads → blocks on response → parses one-letter motor command (`F`/`B`/`L`/`R`/`S`) via `ai_response_parser.c` → enqueues to motor task. The robot is blind between inferences (500–2000 ms gaps), producing jerky stop-go motion. This shape is a legacy of the Claude API era (non-streaming, per-call cost).

Google's [robotics-overview](https://ai.google.dev/gemini-api/docs/robotics-overview) and the [Robotics-ER 1.5 launch post](https://developers.googleblog.com/building-the-next-generation-of-physical-agents-with-gemini-robotics-er-15/) explicitly position ER as *"the high level brain for your robot"* that *"call[s] a vision-language-action model (VLA) or any other third-party user-defined functions to execute the task"*. Every sample is single-request; the fast control layer is assumed to exist and is left to the implementer. Our reactive executor is that layer.

## Target architecture

```
                ┌──────────────────────┐
                │  Planner task        │   ~1 Hz  (Core 1)
                │  ─ capture frame     │
                │  ─ Gemini ER 1.6     │
                │  ─ emit goal via     │
                │    function call     │
                └──────────┬───────────┘
                           │ writes
                           ▼
                ┌──────────────────────┐
                │  goal_state (shared) │   mutex-protected
                │  ─ kind (track/drive/│
                │    rotate/stop)      │
                │  ─ target (box_2d,   │
                │    heading, …)       │
                │  ─ timestamp, ttl    │
                └──────────┬───────────┘
                           │ reads
                           ▼
                ┌──────────────────────┐
                │  Reactive executor   │   30 Hz  (Core 0)
                │  ─ visual servo /    │
                │    PD toward goal    │
                │  ─ obstacle reflex   │
                │    (ultrasonic?)     │
                │  ─ smooth motor PWM  │
                └──────────┬───────────┘
                           │
                           ▼
                 motor_task (existing)
```

### Contracts
- **Planner → goal_state**: structured write via function calls the model emits. No one-letter commands.
- **goal_state**: last-write-wins, has `ttl_ms`. Executor falls back to "safe hold" when the goal ages out.
- **Executor → motor**: existing `motor_controller.c` API — no change to motor layer.
- **Function-call schema** (initial set):
  - `drive(heading_deg, distance_cm, speed_pct)`
  - `track(box_2d, max_speed_pct)` — visual-servo toward a bounding box
  - `rotate(angle_deg)`
  - `stop()`

### Model
- Gemini Robotics-ER **1.6** (1.5 is preview). Use `thinking_budget=0` for latency; raise only if plans regress.

## Non-goals

- IMU, depth, LiDAR. Reactive layer uses the camera (last bounding box via planner) + a single ultrasonic rangefinder + motor state.
- Real VLA (vision-language-action) motor model. The executor is a hand-written PD/servo controller, not a learned policy.
- Multi-backend support. **Claude and Ollama backends are removed from robocar-unified in this PRP** — YAGNI. Gemini Robotics-ER is the only planner. If a self-hosted option is needed later, re-introduce behind a clean abstraction at that point; don't keep dead code now.

## Hardware addition: ultrasonic rangefinder

A 3.3 V-compatible ultrasonic sensor (HC-SR04P, RCWL-1601, or US-100 — confirm model on first wiring pass) is added to the reactive layer as a distance reflex.

**Pin budget** (XIAO ESP32-S3 Sense header GPIOs; already used: `GPIO1` STBY, `GPIO2` buzzer, `GPIO5`/`6` I2C):

| Signal | Pin | Header | Notes |
|---|---|---|---|
| ULTRASONIC_TRIG | GPIO3 | D2 | Output; 10 µs pulse |
| ULTRASONIC_ECHO | GPIO4 | D3 | Input; measure pulse width via RMT RX (precise, non-blocking) |

`pin_config.h` already reserves D2/D3 for this purpose. Free headers after this change: GPIO7/8/9 (D8-D10). Driver lives in `main/ultrasonic.c/.h` and is called by the reactive executor at ~20 Hz (measurement cycle is ~30 ms max range). Pin mapping goes in `main/pin_config.h`.

**Reflex behavior**: if `distance_cm < STOP_THRESHOLD` (e.g. 15 cm), the executor overrides any planner goal with an immediate `stop()` and a brief reverse, regardless of goal ttl. This is the fast "don't hit things" loop that decouples safety from inference latency.

## Work breakdown — agent team

Six roles. Files listed are each agent's exclusive write scope to avoid collisions. The lead (main session) integrates and does final review.

| # | Agent | subagent_type | Exclusive writes | Depends on |
|---|---|---|---|---|
| A | **Gemini tool-call upgrade** | refactor | `main/gemini_backend.c/.h` | – |
| B | **Goal state module** | refactor | `main/goal_state.c/.h` (new) | – |
| C | **Reactive executor + ultrasonic** | refactor | `main/reactive_controller.c/.h`, `main/ultrasonic.c/.h` (new), additions to `main/pin_config.h` | B |
| D | **Planner task + backend cleanup** | refactor | `main/planner_task.c/.h` (new, replaces `ai_backend.c/.h`); deletes `claude_api.c/.h`, `claude_backend.h`, `ollama_backend.c`, `ollama_discovery.c`, `ai_response_parser.c/.h`; drops Kconfig `CONFIG_AI_BACKEND_CLAUDE`/`OLLAMA`; serialized `main.c` wiring | A, B, C |
| E | **Host tests + simulation hook** | test | `packages/esp32-projects/robocar-simulation/**`, new `packages/esp32-projects/robocar-unified/test/**` | B, C, D |
| F | **Docs** | docs | `docs/blueprint/adrs/ADR-016-hierarchical-ai-controller.md`, `packages/esp32-projects/robocar-unified/CLAUDE.md`, `WIRING.md`, README touchpoints | all others |

### Orchestration

Use the `agent-teams` pattern (TeamCreate → parallel SendMessage with shared TaskList → integrate). If teams are not enabled, fall back to parallel `Agent` calls at each phase boundary.

Phases:

1. **Phase 1 — Foundations** (parallel): A + B + F (ADR skeleton) run concurrently. No file overlap.
2. **Phase 2 — Executor + ultrasonic** (after B): C implements the reactive controller and the ultrasonic driver, plus host tests. Planner still runs old path — robot still works.
3. **Phase 3 — Planner swap + legacy removal** (after A + C): D replaces `ai_backend` with `planner_task`, deletes the Claude/Ollama source files, drops the Kconfig options, and wires everything in `main.c`. Single commit, single reviewer pass.
4. **Phase 4 — Integration** (lead + E + F): bench test on hardware, simulation support, finalize docs.

### Parallelism rules
- Agents must not edit `main.c` concurrently; all `main.c` wiring is a single serialized pass in Phase 3 by D.
- ADR lives in `docs/blueprint/adrs/`; the next free number is `ADR-016` (directory currently goes through ADR-015).

## Validation gates

Each gate must pass before the next phase merges.

| Gate | Check | How |
|---|---|---|
| G1 | Gemini client emits + parses function-call responses | Unit test in `test/` using a recorded ER 1.6 response fixture |
| G2 | Reactive controller runs at stable ≥25 Hz in isolation | Host-based test + `uxTaskGetStackHighWaterMark` + timing log |
| G3 | Goal state race-free under fuzz | Host test: N writer threads, 1 reader thread, assert monotonic + no torn reads |
| G4 | On hardware, robot executes a simple task (track a cup) smoothly, no stop-go | Manual bench test + video, measure control-loop rate via serial log |
| G5 | Failsafe: planner offline → executor halts within 2× ttl_ms | Disconnect WiFi mid-run, confirm `stop()` latched |
| G6 | Ultrasonic reflex: obstacle at <STOP_THRESHOLD overrides active goal | Bench: wave hand at sensor during a `drive` goal, confirm immediate stop in serial log |
| G7 | CI build green after Claude/Ollama removal | `just robocar-unified::build`; no references to `claude_*` / `ollama_*` remain |

## Risks / open questions

- **Ultrasonic cone + blind zones** — one forward-facing sensor only covers a ~15° cone. Side/rear obstacles are invisible. Acceptable for this phase; note in ADR.
- **Gemini function-calling reliability** — ER 1.6 is new. Fixture-based tests will flag schema drift early.
- **Claude/Ollama removal is one-way** — if Gemini becomes unavailable or too expensive later, re-introducing a backend requires a real abstraction, not resurrecting the old code. ADR should flag this as accepted risk.
- **Simulation** — `robocar-simulation` is a Pymunk physics sim; it has no Gemini call in the loop. E's task includes a stub planner that emits goals on a schedule so the executor can be exercised in sim.

## Acceptance criteria

- [ ] ADR-016 merged, describing the hierarchical pattern, the Claude/Ollama removal, and the ultrasonic reflex layer. Supersedes ADR-003 for robocar-unified specifically.
- [ ] `claude_*` and `ollama_*` sources deleted; `CONFIG_AI_BACKEND_*` Kconfig removed; no references remain in `robocar-unified`.
- [ ] Ultrasonic wired (GPIO3 TRIG / GPIO4 ECHO), driver reads distances reliably, pin map documented in `WIRING.md`.
- [ ] Robot completes "track cup" and "drive to wall and stop" tasks smoothly; ultrasonic reflex demonstrably overrides goal on obstacle.
- [ ] Reactive loop telemetry shows ≥25 Hz sustained; planner p50 latency ≤1500 ms on Gemini ER 1.6 with `thinking_budget=0`.
- [ ] All existing tests green; new tests for `goal_state`, reactive controller, and ultrasonic driver added.
- [ ] `CLAUDE.md`, `WIRING.md`, and README reflect the new architecture.

## Related

- ADR-002: Dual ESP32 architecture
- ADR-003: Pluggable AI backends (this PRP partially supersedes its runtime contract)
- ADR-008: Gemini third AI backend
- PRP: `slam-autonomous-navigation.md` (natural successor — SLAM replaces the visual-servo layer with a real world model)
- Google: [Gemini Robotics-ER overview](https://ai.google.dev/gemini-api/docs/robotics-overview), [Physical agents blog post](https://developers.googleblog.com/building-the-next-generation-of-physical-agents-with-gemini-robotics-er-15/)
