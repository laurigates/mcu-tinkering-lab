# ADR-016: Hierarchical AI Controller (Planner + Reactive Executor)

**Status**: accepted
**Date**: 2026-04-17
**Source**: PRP: Hierarchical AI Controller (Planner + Reactive Executor), conversation 2026-04-17
**Confidence**: 8/10

---

## Context

The current `robocar-unified` firmware implements an on-demand inference loop: capture frame → upload to cloud AI → block on response → parse one-letter motor command (`F`/`B`/`L`/`R`/`S`) → enqueue to motor task. The robot is blind between inferences (500–2000 ms gaps), resulting in jerky stop-go motion. This monolithic "AI-as-controller" pattern is a legacy of the non-streaming Claude API era, where each call incurred time and cost.

Google's [Gemini Robotics-ER documentation](https://ai.google.dev/gemini-api/docs/robotics-overview) and [Physical Agents blog post](https://developers.googleblog.com/building-the-next-generation-of-physical-agents-with-gemini-robotics-er-15/) explicitly position ER as the *"high level brain for your robot"* that calls a faster control layer. The pattern is clear: a **slow planner** (~0.5–2 Hz) emits structured goals via function calls; a **fast reactive executor** (20–50 Hz) on-device owns motor output and drives toward the goal. The robot moves smoothly and continuously, with the planner redirecting rather than stepping.

This ADR adopts that hierarchy for `robocar-unified` and removes Claude and Ollama backends from this project entirely.

## Decision

Replace the monolithic AI-as-controller loop with a hierarchical two-layer architecture:

### Layer 1: Planner (Core 1, ~1 Hz)

- Captures a frame on a regular schedule
- Calls **Gemini Robotics-ER 1.6** (only planner; no other backends)
- Parses function-call responses to emit structured goals
- Writes `goal_state` (last-write-wins, with TTL) for the executor to read
- Initial function-call schema:
  - `drive(heading_deg, distance_cm, speed_pct)`
  - `track(box_2d, max_speed_pct)` — visual servo toward a bounding box
  - `rotate(angle_deg)`
  - `stop()`

### Layer 2: Reactive Executor (Core 0, ~30 Hz)

- Reads the current `goal_state` via mutex-protected shared struct
- Implements visual servo (PD control) and heading hold toward the planner's goal
- Falls back to "safe hold" (stop) when goal expires (TTL-based)
- Owns all motor PWM output via the existing `motor_controller.c` API
- Reads distance from a single ultrasonic rangefinder (HC-SR04P / RCWL-1601 class, 3.3 V, GPIO7 TRIG / GPIO8 ECHO via RMT) at ~20 Hz
- **Obstacle reflex**: if `distance_cm < STOP_THRESHOLD` (e.g., 15 cm), immediately stop and reverse, regardless of goal TTL or goal type

### Shared Goal State

- **Structure**: `goal_state_t` (mutex-protected)
  - `kind` (enum: `DRIVE`, `TRACK`, `ROTATE`, `STOP`)
  - `target` (box_2d for track, heading + distance for drive, angle for rotate)
  - `timestamp_ms` (write time)
  - `ttl_ms` (validity window, e.g., 2000 ms)
- **Contract**: planner writes atomically; executor reads; last-write-wins
- **Timeout behavior**: executor checks `now - timestamp_ms > ttl_ms` before each iteration; if stale, reverts to `STOP`

### Removal of Claude and Ollama

- Delete `main/claude_api.c/.h`, `main/claude_backend.h`
- Delete `main/ollama_backend.c`, `main/ollama_discovery.c`
- Delete `main/ai_response_parser.c/.h` (no longer needed; structured goals from function calls replace letter-based parsing)
- Drop `CONFIG_AI_BACKEND_CLAUDE` and `CONFIG_AI_BACKEND_OLLAMA` from Kconfig
- No fallback, no `#ifdef` branches. If Gemini becomes unavailable in the future, that is a breaking change; re-introducing a self-hosted backend is a fresh abstraction design, not a resurrection of dead code.

### Hardware Addition: Ultrasonic Rangefinder

| Signal | Pin | Notes |
|--------|-----|-------|
| `ULTRASONIC_TRIG` | GPIO7 | Output; 10 µs pulse |
| `ULTRASONIC_ECHO` | GPIO8 | Input; measure pulse width via RMT RX (non-blocking, precise) |

- Driver: `main/ultrasonic.c/.h` (new)
- Pin config: `main/pin_config.h`
- Measurement cycle: ~30 ms max range; executor calls at ~20 Hz
- Reflex distance threshold: configurable (default 15 cm); reflex is hardwired safety, not subject to planner override

## Consequences

**Positive**
- Continuous smooth motion: planner emits goals on its slow schedule; executor drives toward them at full framerate
- Safety decoupled from inference latency: ultrasonic reflex operates at 20 Hz regardless of planner round-trip time (typically 1–2 seconds)
- Simpler codebase: no multi-backend compile-time branching; no letter-based command parsing
- Conforms to Google's published pattern for Robotics-ER, lowering risk of drift as the model evolves
- Structured goals (function calls) are more expressive than single letters and less ambiguous than heuristic one-shot parsing

**Negative**
- Vendor lock-in: Gemini Robotics-ER 1.6 is the only planner. If unavailable or prohibitively expensive in the future, re-introducing a backend (Claude, Ollama, or local) requires a proper abstraction and refactor, not a quick revert to old code
- Ultrasonic rangefinder has single forward-facing cone (~15° coverage); side and rear obstacles are invisible. Acceptable for this phase; future work may add a full ring of sensors or vision-based obstacle detection
- If Gemini function-calling reliability degrades or schema drifts, fixture-based tests will catch it, but production recovery requires a redesigned planner

**Accepted Risks**
- One-way removal of Claude and Ollama: keeping dead code in case of future re-introduction is YAGNI and adds maintenance burden (imports, compilation, credential files). When a self-hosted option is needed, it will be designed cleanly from scratch
- Gemini dependency: the robotics-ER API is new (ER 1.5 launch early 2025, ER 1.6 mid 2025). Early adopter risk is mitigated by treating this as an internal implementation detail; the planner-executor contract is stable independent of the backend

## Alternatives Considered

1. **Keep multi-backend support (ADR-003 pattern)** — Rejected. Compile-time selection multiplies code complexity for diminishing return; on-demand inference is fundamentally at odds with smooth reactive control. Removing dead code encourages cleaner future designs
2. **Run planner and executor on the same core** — Rejected. 1 Hz planner blocking on network I/O starves the 30 Hz executor; dual-core is required for independent scheduling
3. **Omit ultrasonic, rely on planner vision** — Rejected. Planner latency (1–2 seconds) is too long for safe obstacle avoidance; reflex must be independent and real-time
4. **Use multiple ultrasonic sensors (ring)** — Deferred. Single forward-facing sensor is sufficient for demonstrating the pattern; mechanical constraint is a future upgrade, not blocking

## Supersedes and Amends

This ADR supersedes the runtime contract of **ADR-003: Pluggable AI Backends** for `robocar-unified` specifically. ADR-003 remains valid for other projects in the monorepo (e.g., `esp32cam-llm-telegram`). The decision to support multiple backends at compile time is reversed for this project; single-backend design is now the pattern.

## Related

- **PRP**: `docs/prompts/hierarchical-ai-controller.md` — Product requirements and work breakdown
- **ADR-002**: Dual ESP32 Architecture (context on what `robocar-unified` consolidated)
- **ADR-003**: Pluggable AI Backends (this ADR narrows that pattern for robocar-unified; other projects unaffected)
- **ADR-008**: Gemini Third AI Backend (Gemini integration, now the sole planner backend)
- **Google Robotics-ER**: [Robotics overview](https://ai.google.dev/gemini-api/docs/robotics-overview), [Physical agents blog](https://developers.googleblog.com/building-the-next-generation-of-physical-agents-with-gemini-robotics-er-15/)
