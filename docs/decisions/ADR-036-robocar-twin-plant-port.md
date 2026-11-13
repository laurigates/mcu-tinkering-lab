# ADR-036: Robocar Twin Executor and Plant Port

**Status**: accepted
**Date**: 2026-11-13
**Confidence**: 6/10 (the executor was linked into a scratch copy of the twin
and ticked with all seven scenario traces byte-identical; the plant library,
the plant exchange and the coupled scenarios are designed here and not yet
built. Every chassis and motor figure the plant uses is an estimate)
**Requirements**: [PRD-017](../requirements/PRD-017-robocar-behaviour-twin.md),
[PRD-003](../requirements/PRD-003-robot-physics-simulation.md)
**Amends**: [ADR-030](ADR-030-robocar-behaviour-twin.md) §3 (the world) and
its open question 4
**Related**: [ADR-016](ADR-016-hierarchical-ai-controller.md) (planner and
executor), [ADR-029](ADR-029-idle-gaze-and-wander.md) (dwell poses)

---

## Context

Two host-side tools describe the robocar and share no code.

- `packages/robotics/robocar/twin` (ADR-030) compiles the firmware's planner
  for the host and runs it on a virtual clock against scripted world events.
  It has no executor: a goal is written and traced, no motor command comes
  out, and nothing the planner decides can change a later sensor reading.
- `packages/robotics/robocar/simulation` is a Python model (Pymunk, a DC motor
  model) of the removed dual-ESP32 car. Its inputs are that car's I2C bridge
  and a Claude/Ollama client, its sensors are one HC-SR04 and an MPU6050, and
  genesis-world and torch are hard dependencies (an 8.4 GB environment).

ADR-030 §3 chose a scripted timeline over a physics engine and left open
whether the simulation is retired. Lauri decided on 2026-11-13 to keep both,
integrated, with each still runnable alone for the aspect being developed.

Facts the design rests on, from a mapping run on `origin/main` at 653a454:

- The executor already has a host path. `reactive_controller.c` under
  `REACTIVE_CONTROLLER_HOST_TEST` exposes `reactive_controller_tick_for_test()`
  and is built by `test/CMakeLists.txt` with `ultrasonic.c`, `head_aim.c`,
  `idle_gaze.c`, `cliff_core.c`, `cliff_sensor.c` and the motor and servo
  stubs.
- Adding those sources to the twin needs one more definition,
  `CLIFF_SENSOR_HOST_TEST=1`. Ticked every 33 ms as a fourth source in
  `advance_to()`, the executor left all seven scenario traces byte-identical;
  a 30-minute run is 54,546 ticks in 0.012 s.
- The twin's planner inputs differ from the firmware's today: `robot_moving`
  counts a `track` goal with `max_speed_pct == 0` as moving, where
  `goal_moves_wheels()` does not, and reflex trips come from a scripted
  `reflex` event, not from a range reading.
- The host `ultrasonic.c` stub returns the injected value on every tick. The
  firmware completes a reading about every 165 ms (`ULTRASONIC_CONVERSION_MS`
  120 plus a tick to read and a tick to re-trigger).
- The simulation's runs are bit-identical across processes. With Pymunk on,
  its front rangefinder never sees anything: the ray starts inside the chassis
  shape and hits it first. Its motor integration overshoots for `dt` above
  33.3 ms.
- A path dependency from the twin harness on the simulation couples the
  harness `uv.lock` to every pin in the simulation's `pyproject.toml`, extras
  included: bumping one dev pin, or the release-please version, made the
  harness's `uv run --locked` fail.
- A blocking exchange per executor tick costs about 0.17 s per simulated
  minute through the runner's pump thread (1,818 round trips).
- The repo holds no measured track width, wheel diameter, mass, motor speed or
  sensor mounting position for the current chassis.

## Decision

The twin gains the firmware's executor and a **plant port**: the source of the
sensor readings is either the scenario's script or a physics plant stepped in
lockstep. The simulation becomes that plant, a deterministic library with its
own entry point, run as a separate process.

```
                       ┌──────────────── twin harness (Python) ────────────────┐
scenario.yaml ───────▶ │ runner ── model port ── inline | replay               │
                       │    │                                                  │
                       │    └───── plant port ── scripted | physics ───────────┼──┐
                       └───────┬───────────────────────────────────────────────┘  │ JSONL
        world events, JSONL    │  ▲ trace, model_request, plant_request           ▼
                               ▼  │                                   robocar_sim.plant
                       robocar-twin (C): planner_cycle_core + executor,       (Python, own
                       ultrasonic_core, cliff_core, on a virtual clock         uv project)
```

| Developing | Run | Needs |
|---|---|---|
| Planner gates, speech, budget | twin, scripted world | C compiler, cJSON, harness |
| Executor, reflexes, head | twin, scripted world | same |
| Motor model, arena, sensor geometry | plant alone | numpy, pymunk, pyyaml |
| Goal-to-motion behaviour | twin with `plant: physics` | both |

### 1. The executor runs in the twin, always

The twin links `reactive_controller.c` (host path), `ultrasonic_core.c`,
`head_aim.c`, `idle_gaze.c`, `cliff_core.c` and `cliff_sensor.c`, and ticks the
executor every 33 ms from `t = 0` as a fourth source in `advance_to()`, after
world events, voice-turn ends and audio frames that fall on the same instant.
Ticks inside a model reply's latency window run against the previous goal, as
on the robot. The tick asserts that the executor's own clock (`ticks * 33`)
equals the virtual clock.

There is no switch that turns the executor off. Two inputs are opt-in per
scenario because each changes what the planner reads:

| `config` key | Default | Effect when set |
|---|---|---|
| `head` | `false` | Servos present: the executor aims the head, `head` records are traced, and a frame's dwell pose comes from the servo position and `head_settled` as in `planner_task.c`. A scripted `frame.pose` is then refused |
| `cliff` | `false` | Cliff reflex armed. The cliff state starts as "no floor" and fails closed, as in the firmware, so a scenario that sets it must supply floor readings |

The planner's evidence comes from `reactive_controller_get_telemetry()` the
way `planner_task.c` reads it: range, range validity, `robot_moving` (through
`goal_moves_wheels()`), and reflex trips. The twin's own copies of those are
removed.

**Rangefinders.** The twin drives `ultrasonic_core.c` through a twin-owned
fake bus that samples the world at trigger time, so a reading takes the
firmware's read cycle. A scripted obstacle shorter than one cycle can be
missed, as on the robot.

**Wheels.** The twin links the real `motor_controller.c` over the PCA9685 host
shim, so the 300 ms rise ramp is the firmware's, and reads each wheel back
from the channels the driver wrote. If `motor_controller.c` cannot be linked
without changing firmware behaviour, the fallback is a twin-owned recorder
implementing `motor_controller.h` (not the shared `test/motor_controller_stub.c`,
which does not record the `move_*`/`rotate_*` calls); the plant then applies no
ramp and the twin README says so.

**Trace.** Two records, emitted on change only:

```json
{"t":"wheels","at_ms":20031,"left":{"drive":"brake","duty":0},"right":{"drive":"brake","duty":0}}
{"t":"head","at_ms":4290,"pan_deg":-24,"tilt_deg":0,"mode":"gaze","settled":false}
```

`drive` is `forward`, `reverse`, `coast` (IN1 = IN2 = low) or `brake`
(IN1 = IN2 = high); `duty` is the PCA9685 count, 0..4095.

**World events.** `reflex` is removed: a reflex is the executor's reaction to a
range reading, so a scenario scripts the obstacle. The scripted events that
feed the executor are:

| Event | Fields |
|---|---|
| `range` | `cm`, optional `sensor: front \| rear` (default `front`); a bare integer still means front |
| `range_fail` | optional `sensor` |
| `floor` | `front`, `rear` booleans (floor present under each sensor) |

`us-n1-note-on-reflex` is rewritten with range dips of at least one read cycle.

**Host hooks.** Firmware changes for this are limited to host-only code
(`#ifdef ..._HOST_TEST`) and declarations: a prototype for the tick, and a
setter that seeds the idle gaze from the scenario `seed`. `just robocar::test`
stays green and the target build is unchanged.

### 2. The simulation becomes the plant

`packages/robotics/robocar/simulation` keeps its directory, its release
component (`robocar-simulation`) and its distribution name
(`esp32-robot-simulation`), so release-please's version files need no change.
Its import package becomes `robocar_sim` at `src/robocar_sim/`.

**Dependencies.** Core: `numpy`, `pymunk`, `pyyaml`. A `viz` extra holds
genesis-world, torch, opencv, matplotlib, trimesh, spatialmath and Pillow with
its override. Dependencies no source imports are dropped (websockets,
pyserial, httpx, crc8, asyncio-mqtt, scipy, pandas, click, toml), and
`requirements.txt` is deleted.

**Removed.** `communication_bridge.py`, `ai_command_processor.py`,
`wifi_simulation.py`, `ota_simulation.py`, `stub_planner.py`, their tests,
`docs/ESP32_FEATURE_INTEGRATION.md`, and the config sections and justfile
recipes that only served them. `swift_subprocess.py` and `src/test_swift.py`
go too: they import packages the project does not declare. The twin runs the
real planner, which is what `stub_planner.py` stood in for.

**Kept as the legacy demo.** `DifferentialDriveRobot`, `motor_controller.py`,
`error_handling.py`, `camera_simulation.py`, `genesis_visualizer.py` and
`main.py` stay under the `viz` extra with their current behaviour, minus the
removed subsystems. They still describe the old chassis
(`config/robot_config.yaml`). Moving the visualiser onto the plant and then
deleting the legacy model is a follow-up.

**The plant.** A new `robocar_sim/plant.py` reuses `DCMotor` and
`PhysicsEngine` and does not go through `DifferentialDriveRobot`:

```python
plant = Plant(config, arena, seed, start=Pose(x_cm, y_cm, heading_deg))
reading = plant.step(dt_ms, left=WheelCmd(drive, duty), right=WheelCmd(drive, duty))
# Reading: at_ms, pose, range_cm {front, rear: int | None},
#          floor {front, rear: bool}, collided: bool, off_floor: bool
```

- Deterministic: one `numpy` generator seeded from `seed`, no wall clock, no
  threads, no process-wide state, no log file, no swallowed exceptions. Two
  plants built with the same arguments give bit-identical readings.
- Motor integration sub-steps inside `step()` so a 33 ms tick is stable.
- `coast` opens the winding (current forced to zero, friction only); `brake`
  shorts it. Supply voltage comes from the config (5 V, the buck that feeds
  the TB6612FNG), not from a constant.
- The range ray ignores the robot's own shape, reports `None` past the
  sensor's maximum, and exists for a front and a rear sensor.
- The arena has a floor polygon, walls and obstacles. A floor sensor reads
  floor when its point is inside the polygon; `off_floor` is set when a wheel
  contact point leaves it.
- The plant reports geometric truth every step. Read pacing, smoothing and
  fail-closed semantics stay in the firmware's `ultrasonic_core` and
  `cliff_core`.

`config/plant.yaml` describes the current robot. Every chassis, motor and
mounting figure in it is marked as an estimate with what to measure, because
the repo has no measured value.

**Standalone.** `python -m robocar_sim.plant` is the stepper below. A demo
recipe steps the plant through scripted wheel commands, writes the pose trace,
and plots arena and path when the `viz` extra is installed.

### 3. The plant port

**Scenario.** A top-level `plant:` key next to `models:`:

```yaml
plant: scripted            # default; may be omitted
# or
plant:
  physics:
    arena: arenas/wall-1m.yaml          # resolved against the twin directory
    start: {x_cm: 0, y_cm: 0, heading_deg: 0}
```

With `physics`, the loader refuses `range`, `range_fail` and `floor` events,
because the plant produces them. The loader validates the scenario and the
arena file with pyyaml alone.

**Twin protocol.** With `"plant":"external"` in `config`, the twin sends one
`plant_request` after each executor tick and blocks for the `plant_response`,
the same pattern as `model_request`:

```json
{"t":"plant_request","id":607,"at_ms":20031,"dt_ms":33,
 "left":{"drive":"forward","duty":2409},"right":{"drive":"forward","duty":2409}}
{"t":"plant_response","id":607,"range_cm":{"front":38,"rear":null},
 "floor":{"front":true,"rear":true}}
```

The response is the world at `at_ms + dt_ms`. It sets what the fake bus
returns at the next trigger and the next cliff sample. A `null` range is a
failed read. Until the first response the sensors read as they do at boot.

**Plant process.** The harness's `PhysicsPlant` starts the plant as a child
process and relays JSONL, so the two `uv.lock` files stay independent:

```json
{"t":"init","seed":7,"arena":{...},"start":{"x_cm":0,"y_cm":0,"heading_deg":0}}
{"t":"ready"}
{"t":"step","dt_ms":33,"left":{"drive":"forward","duty":2409},"right":{...}}
{"t":"reading","at_ms":33,"pose":{"x_cm":0.4,"y_cm":0.0,"heading_deg":0.0},
 "range_cm":{"front":96,"rear":null},"floor":{"front":true,"rear":true},
 "collided":false,"off_floor":false}
```

The default command is `uv run --project <simulation> --locked --quiet python
-m robocar_sim.plant`; `ROBOCAR_PLANT_CMD` overrides it, which is how the
harness unit tests substitute a fake plant. A plant that exits, or answers out
of order, fails the scenario with the plant's stderr.

**Trace and assertions.** The runner does not copy the per-tick exchange into
the trace. It appends a `pose` record from each reading whose pose, `collided`
or `off_floor` differs from the last one. Assertions are added over `wheels`,
`head` and `pose` records, with numeric bounds; the harness README lists them.

**CI.** `just robocar-twin::test` stays light: it runs every scenario not
marked `physics`. A `test-physics` recipe runs the coupled scenarios, as a
second entry in `.github/python-suites.json` (`robocar-twin-physics`) selected
by changes under the twin, `main/`, or the simulation's `src/`, `config/`,
`pyproject.toml` and `uv.lock`.

### 4. Scenarios this adds

Scripted world (step 1): the front reflex brakes a forward drive within
`REACTIVE_REFLEX_WORST_CASE_MS` and releases; a rear reading holds a reverse;
the cliff reflex brakes and backs off; the idle gaze moves the head only
between dwell poses.

Physics plant (step 3): a `drive` goal toward a wall ends with the wheels
braked and no collision; on a table arena with the cliff reflex armed the
robot never leaves the floor. These assert ordering and bounds with wide
tolerances, not distances, until the plant is calibrated.

## Consequences

- The twin's planner inputs now come from the executor, which removes two
  divergences from the firmware (`robot_moving`, scripted reflex latches).
- The limitation list in the twin README shrinks: the executor, the head and
  its dwell poses are modelled when `head` is set. TTS playback time, the
  start cue and manual leases stay unmodelled.
- `robocar-twin` stays buildable with a C compiler and cJSON alone; the plant
  is needed only for scenarios that name it.
- The simulation's CI install drops from the full Genesis stack to three
  packages, including the `ty` sync that runs on every code PR.
- A scenario that asserts a physics outcome is only as good as the plant's
  calibration. The firmware-only scenarios remain the gate that matters for a
  firmware change; a failing physics scenario after a plant change is a plant
  finding first.
- Camera frames stay scripted. The plant does not render, and a frame's scene
  is still named by the scenario.
- A coupled run exchanges one message per 33 ms tick, about 1,800 per
  simulated minute. Hours-long scenarios (dormancy, budget) stay scripted.

## Alternatives considered

- **Import the plant into the harness as an optional extra.** Works once the
  heavy dependencies are extras, but the harness lock then embeds the
  simulation's whole `requires-dist`, so every Renovate bump or release of the
  simulation needs a harness re-lock. The process boundary costs 0.17 s per
  simulated minute.
- **A kinematic stub inside the twin** (ADR-030 phase 4). No contact, no
  motor lag, no arena; it would be a third model of the car.
- **Executor opt-in per scenario.** Two code paths for the planner's inputs,
  one of which the robot never runs. The rehearsal showed the always-on
  executor changes no existing trace.
- **Retire the simulation.** Loses the only motor and contact model, which
  goal-to-motion scenarios need.
- **Port the visualiser to the plant now.** 851 lines with no test coverage
  and no headless check available in CI; deferred.

## Open questions

1. Bench measurements for `config/plant.yaml`: track width, wheel diameter,
   speed and yaw rate per duty, stopping distance, sensor offsets.
2. Whether the plant should select a frame's scene from the robot's pose, so
   a coupled scenario does not script frames by hand.
3. Whether the legacy `DifferentialDriveRobot` and Genesis visualiser are
   worth porting to the plant or are deleted.
