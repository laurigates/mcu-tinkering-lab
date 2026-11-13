# ADR-036: Robocar Twin Executor and Plant Port

**Status**: accepted
**Date**: 2026-11-13
**Confidence**: 6/10 (sections 1 to 3 are built and pass their gates: the
executor, the real `motor_controller.c` and `ultrasonic_core.c` link with one
host-only setter added under `main/`, six of seven scenario traces stay
byte-identical, the plant has its own tests, and the wall and table-edge
scenarios of section 4 pass against it at 40 % duty. Nothing here has run
against the robot, and every chassis and motor figure the plant uses is an
estimate)
**Requirements**: [PRD-017](../requirements/PRD-017-robocar-behaviour-twin.md),
[PRD-003](../requirements/PRD-003-robot-physics-simulation.md)
**Amends**: [ADR-030](ADR-030-robocar-behaviour-twin.md) §3 (the world) and
its open question 4; PRD-017's non-goals on motor dynamics and physics worlds
(not required of the twin, available through the optional plant); PRD-003
R6–R8 (the WebSocket server, the serial bridge and the hardware command
protocol are withdrawn)
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

Facts the design rests on, from a mapping run and a rehearsal on `origin/main`
at 653a454:

- The executor already has a host path. `reactive_controller.c` under
  `REACTIVE_CONTROLLER_HOST_TEST` exposes `reactive_controller_tick_for_test()`
  and is built by `test/CMakeLists.txt` with `head_aim.c`, `idle_gaze.c`,
  `cliff_core.c`, `cliff_sensor.c` and the motor and servo stubs.
- Ticked every 33 ms as a fourth source in `advance_to()`, with the planner
  reading the executor's telemetry, the executor left six of the seven
  scenario traces byte-identical. The seventh, `us-n1`, scripts `reflex`
  events. A 30-minute run is 54,546 ticks in 0.012 s.
- The twin's planner inputs differ from the firmware's today: `robot_moving`
  counts a `track` goal with `max_speed_pct == 0` as moving, where
  `goal_moves_wheels()` does not, and reflex trips come from a scripted
  `reflex` event, not from a range reading.
- The host `ultrasonic.c` stub returns the injected value on every tick. The
  firmware completes a reading about every 165 ms (`ULTRASONIC_CONVERSION_MS`
  120 plus a tick to read and a tick to re-trigger). Through `ultrasonic_core.c`
  a range dip of 165 ms is never missed and the worst event-to-brake time is
  296 ms, against `REACTIVE_REFLEX_WORST_CASE_MS` 297.
- `motor_controller.c` compiles unmodified on the host and takes its time from
  `esp_timer_get_time()`, which the twin already defines as the virtual clock,
  so its 300 ms rise ramp runs with no hook (duty 546, 996, 1446, 1896, 2346,
  2457 on consecutive ticks for a 60 % drive).
- The simulation's runs are bit-identical across processes. With Pymunk on,
  its front rangefinder never sees anything: the ray starts inside the chassis
  shape and hits it first. The chassis sinks about 10 cm into an obstacle it
  drives at. Its motor integration overshoots for `dt` above 33.3 ms.
- A path dependency from the twin harness on the simulation couples the
  harness `uv.lock` to every pin in the simulation's `pyproject.toml`, extras
  included: bumping one dev pin, or the release-please version, made the
  harness's `uv run --locked` fail.
- A blocking exchange per executor tick costs about 0.1 to 0.2 s per simulated
  minute (1,819 round trips).
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

**Build.** The twin links, unmodified, `reactive_controller.c` (host path),
`motor_controller.c`, `ultrasonic_core.c`, `head_aim.c`, `idle_gaze.c`,
`cliff_core.c` and `cliff_sensor.c` (with `CLIFF_SENSOR_HOST_TEST=1`), and
`test/servo_controller_stub.c`. `main/ultrasonic.c` is left out: the twin
supplies the `ultrasonic.h` functions over `ultrasonic_core.c`, as
`test/test_reflex_latency.c` does. `motor_controller.c` builds against
`test/include/pca9685_host` and `pca9685_decl`, with a twin-owned
`i2c_bus_pca9685_set_multi()` that records the six motor channels and stubs
for the STBY GPIO calls. The twin calls `motor_controller_init()` before the
first tick and exits with an error if it does not return `ESP_OK`; every other
`motor_*` call would otherwise fail silently.

**Tick.** The executor ticks every 33 ms from `t = 0` as a fourth source in
`advance_to()`, after world events, voice-turn ends and audio frames that fall
on the same instant. Ticks inside a model reply's latency window run against
the previous goal, as on the robot. The twin asserts that the number of ticks
it has issued times 33 equals the virtual clock.

There is no switch that turns the executor off. Two inputs are opt-in per
scenario: `head` changes what the planner reads, and `cliff`, failing closed,
holds the wheels until the scenario supplies floor readings.

| `config` key | Default | Effect when set |
|---|---|---|
| `head` | `false` | Servos present (`servo_stub_reset(true)` before `reactive_controller_init()`): the executor aims the head and `head` records are traced. At each planner tick the twin samples the servo position and `telemetry.head_settled` as `planner_task.c` does at capture, sets the fingerprint's pose and the descriptor's `pose` from them (`-` when unsettled or between poses), and writes the sampled pan and tilt into `goal.head_pan_deg` / `head_tilt_deg`. Before each tick the twin passes its voice-turn busy flag to `reactive_controller_set_face_forward_for_test()`, so the gaze faces forward during a voice turn |
| `cliff` | `false` | Cliff reflex armed with `reactive_controller_set_cliff(true)` after init. The cliff state starts as "no floor" and fails closed, as in the firmware |

With `head` set the scenario loader refuses a `pose` key in a `frame` and
sends the frame without one; the twin ignores `pose` on a frame when `head` is
set. With `head` unset the head reads as unavailable and `frame.pose` works as
before.

**Planner inputs.** The planner's evidence comes from
`reactive_controller_get_telemetry()` the way `planner_task.c` reads it: range,
range validity, `robot_moving` (through `goal_moves_wheels()`), and reflex
trips. The twin's own copies of those are removed.

A range is first readable one read after it is set (132 ms scripted, 297 ms
with the plant). The planner tick at `t = 0` therefore reads the executor's
boot value (400 cm, valid), `plan_activity` adopts it as its range reference,
and the first real reading wakes the planner with `R` on the next cycle. The
twin runs that code as it is, and scenario request counts include the wake.
Whether the robot does the same depends on how long its first camera capture
takes, which is unmeasured.

**Rangefinders.** The twin-owned bus samples the world at trigger time, so a
reading takes the firmware's read cycle. A failed read (`range_fail`, a `null`
range from the plant, and any trigger latched before the first `range` event
or plant response) is the bus accepting the trigger and failing the read, so
the cycle stays 165 ms. Ranges are whole centimetres in 2..400, the bounds
`ultrasonic_core_decode()` accepts: the loader refuses other values in a
scenario, and the twin treats any other plant value as a failed read.

**Trace.** Two records, emitted on change only:

```json
{"t":"wheels","at_ms":20262,"left":{"drive":"brake","duty":0},"right":{"drive":"brake","duty":0}}
{"t":"head","at_ms":4290,"pan_deg":-24,"tilt_deg":0,"mode":"gaze","settled":false}
```

`drive` is `forward`, `reverse`, `coast` (IN1 = IN2 = low) or `brake`
(IN1 = IN2 = high), decoded from the recorded direction channels; `duty` is
the PCA9685 count the driver wrote, 0..4095. The state after
`motor_controller_init()` (both wheels coast, duty 0) is the reference and is
not traced. With `head` set the first `head` record is the state after tick 0.

**World events.** `reflex` is removed: a reflex is the executor's reaction to a
range reading, so a scenario scripts the obstacle. The scripted events that
feed the executor are:

| Event | Fields |
|---|---|
| `range` | `cm` (2..400), optional `sensor: front \| rear` (default `front`); a bare integer still means front |
| `range_fail` | optional `sensor` |
| `floor` | `front`, `rear` booleans (floor present under each sensor) |

The loader refuses a `floor` event unless `cliff` is set: without it the floor
sensors read as absent and the event would change nothing.
`us-n1-note-on-reflex` is rewritten with range dips of at least one read cycle.

**Host hooks.** The only new code under `main/` is a setter in the host branch
of `reactive_controller.c` that seeds the idle gaze from the scenario `seed`.
The twin calls it after `reactive_controller_init()`, which resets the
generator, and it maps the seed to a non-zero xorshift state (a seed of 0
would otherwise fix the gaze). `reactive_controller.h` gains host-only
prototypes for that setter, `reactive_controller_tick_for_test()` and
`reactive_controller_set_face_forward_for_test()`. `just robocar::test` stays
green and the target build is unchanged.

### 2. The simulation becomes the plant

`packages/robotics/robocar/simulation` keeps its directory, its release
component (`robocar-simulation`) and its distribution name
(`esp32-robot-simulation`), so release-please's version files need no change.
Its import package becomes `robocar_sim` at `src/robocar_sim/`, with relative
imports inside it.

**Dependencies.** Core: `numpy`, `pymunk`, `pyyaml`. A `viz` extra holds
genesis-world, torch, opencv, matplotlib, trimesh, spatialmath and Pillow with
its override; `tests/test_pillow_override.py` reads the pin from that extra.
The `gpu` and empty `visualization` extras are deleted, so `viz` is the only
optional runtime extra. Dependencies no source imports are dropped (websockets,
pyserial, httpx, crc8, asyncio-mqtt, scipy, pandas, click, toml).

**Removed.** `communication_bridge.py`, `ai_command_processor.py`,
`wifi_simulation.py`, `ota_simulation.py`, `stub_planner.py`, their tests,
`docs/ESP32_FEATURE_INTEGRATION.md`, and the config sections and justfile
recipes that only served them. `swift_subprocess.py` and `src/test_swift.py`
go too: they import packages the project does not declare. `requirements.txt`
is deleted, and with it `setup_uv.py`, `migrate_to_uv.py` and
`UV_MIGRATION_GUIDE.md`, which exist to convert it. The twin runs the real
planner, which is what `stub_planner.py` stood in for.

The bridge is PRD-003's R6–R8. The plant's stepper (below) is the simulation's
external interface from here on; the feature tracker marks FR-006 deprecated
and notes on FR-007 that hardware-in-the-loop has no bridge to build on.

**Package move.** The console scripts become `robocar_sim.main:main`, every
`src/main.py` recipe becomes `python -m robocar_sim.main` under `--extra viz`,
and the ruff first-party list, the tests' and `debug_*.py` imports, the
simulation's README and CLAUDE.md, and the `sim` skill follow.

**Kept as the legacy demo.** `DifferentialDriveRobot`, `motor_controller.py`,
`error_handling.py`, `camera_simulation.py`, `genesis_visualizer.py` and
`main.py` stay with their current behaviour, minus the removed subsystems.
`DifferentialDriveRobot.__init__` no longer imports `camera_simulation` (only
`enable_camera()` does), so the robot-model tests run on the core
dependencies; the visualiser and camera need the `viz` extra. They still
describe the old chassis (`config/robot_config.yaml`). Moving the visualiser
onto the plant and then deleting the legacy model is a follow-up.

**The plant.** A new `robocar_sim/plant.py` reuses `DCMotor` and
`PhysicsEngine` and does not go through `DifferentialDriveRobot`:

```python
plant = Plant(arena, seed, start=Pose(x_cm, y_cm, heading_deg), config=None)
reading = plant.step(dt_ms, left=WheelCmd(drive, duty), right=WheelCmd(drive, duty))
# WheelCmd: drive in {forward, reverse, coast, brake}, duty 0..4095 (PCA9685 counts)
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
- The range ray ignores the robot's own shape, reports `None` outside the
  sensor's 2..400 cm, and exists for a front and a rear sensor.
- The chassis does not pass through a wall or an obstacle: the pose comes from
  the Pymunk body after contact resolution, and `collided` is set when a
  contact held during any part of the step.
- A floor sensor reads floor when its point is inside the arena's floor
  polygon; `off_floor` is set while a wheel contact point is outside it and
  is not latched. The plant does not model the fall.
- Rangefinders see walls and obstacles, not the floor's edge.
- The plant reports geometric truth every step. Read pacing, smoothing and
  fail-closed semantics stay in the firmware's `ultrasonic_core` and
  `cliff_core`.

The default robot description is package data, `src/robocar_sim/plant.yaml`,
loaded relative to the package and never the working directory. Every chassis,
motor and mounting figure in it is marked as an estimate with what to measure,
because the repo has no measured value.

**The arena** is one schema with one owner, `robocar_sim.arena`. Units are
centimetres and degrees; heading 0 points along +x and angles grow
counter-clockwise. Unknown keys are an error.

```yaml
floor:                                   # required: polygon the robot stands on
  - [-50, -60]
  - [150, -60]
  - [150, 60]
  - [-50, 60]
walls:                                   # optional: rangefinders see them, the chassis hits them
  - {from: [150, -60], to: [150, 60]}
obstacles:                               # optional
  - {box: {center: [80, 0], size: [20, 30], heading_deg: 0}}
  - {cylinder: {center: [40, 30], radius_cm: 5}}
```

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
    arena: arenas/wall-1m.yaml          # resolved against the scenario's twin directory, like replay:
    start: {x_cm: 0, y_cm: 0, heading_deg: 0}
```

With `physics`, the loader refuses `range`, `range_fail` and `floor` events,
because the plant produces them. The harness checks only that the arena file
exists and parses to a mapping, then forwards it; the plant validates it.

**Twin protocol.** With `"plant":"external"` in `config`, the twin sends one
`plant_request` after each executor tick and blocks for the `plant_response`,
the same pattern as `model_request`. The request id is the tick index.

```json
{"t":"plant_request","id":607,"at_ms":20031,"dt_ms":33,
 "left":{"drive":"forward","duty":2409},"right":{"drive":"forward","duty":2409}}
{"t":"plant_response","id":607,"range_cm":{"front":38,"rear":null},
 "floor":{"front":true,"rear":true}}
```

The response is the world at `at_ms + dt_ms`. It sets what the bus returns at
the next trigger and the next cliff sample. Before the first response the
cliff sample is invalid. A range that is `null`, outside 2..400 or fractional
is a failed read; a range of any other JSON type is a broken plant and ends
the run, because a failed read counts as a clear path. Requests are strictly sequential with model requests:
ticks inside a reply's latency window exchange with the plant before the reply
is handled.

**Plant process.** The harness's `PhysicsPlant` starts the plant as a child
process and relays JSONL, so the two `uv.lock` files stay independent:

```json
{"t":"init","seed":7,"arena":{"floor":[[-50,-60],[150,-60],[150,60],[-50,60]]},"start":{"x_cm":0,"y_cm":0,"heading_deg":0}}
{"t":"ready"}
{"t":"step","dt_ms":33,"left":{"drive":"forward","duty":2409},"right":{"drive":"forward","duty":2409}}
{"t":"reading","at_ms":33,"pose":{"x_cm":0.086,"y_cm":0.0,"heading_deg":0.0},
 "range_cm":{"front":null,"rear":null},"floor":{"front":true,"rear":true},
 "collided":false,"off_floor":false}
```

`init` and `step` carry exactly these keys; an extra or missing key is an
error, so the harness does not forward the twin's `id` or `at_ms`. `dt_ms` and
`duty` are JSON integers. Pose figures are rounded to 0.001, so a chassis held
against a wall reports the same pose from tick to tick. A bad `init` or `step`
gets `{"t":"error","message":"..."}` and a non-zero exit. The default command is `uv run --project <simulation> --locked --quiet
python -m robocar_sim.plant`, where `<simulation>` is the directory next to
the harness's own twin directory (`default_twin_dir().parent / "simulation"`),
not the scenario's. `ROBOCAR_PLANT_CMD`, split with `shlex.split`, overrides it
for recipes and CI; `run_scenario` takes `plant=` beside `port=`, which unit
tests use with a fake plant. A plant that exits, or answers out of order, fails
the scenario with the plant's stderr.

**Trace and assertions.** The runner does not copy the per-tick exchange into
the trace. It appends a `pose` record from each reading whose pose, `collided`
or `off_floor` differs from the last one. Assertions are added over `wheels`,
`head` and `pose` records, with numeric bounds and a state-at-time lookup; the
harness README lists them. A `pose` assertion is refused in a scripted
scenario, which has no pose records and would pass it without checking
anything.

**Suites.** A scenario whose `plant:` is `physics` gets the pytest marker
`physics` on `test_scenario_passes` only, derived at collection time.
`test_scenario_validates` is unmarked, so the light suite validates every
scenario file, physics ones included; under `-m physics` a scenario is
validated when its run loads it. The marker
is registered in the harness `pyproject.toml`. `just robocar-twin::test` runs
`pytest -m "not physics"` and stays light. `test-physics` first runs
`uv run --project <simulation> --locked python -c "import robocar_sim.plant"`,
so the plant environment is built before any scenario's timeout starts and a
stale simulation lock fails as a setup error, then `pytest -m physics`. It is
`uv run`, not `uv sync`, so a developer's installed extras stay. `run` writes
the scorecard for every scenario and primes the plant the same way.

`.github/python-suites.json` gains `robocar-twin-physics` with the
`robocar-twin` suite's paths plus the simulation's `src/**`, `pyproject.toml`
and `uv.lock`. Both suites also list `test/servo_controller_stub.c`, which the
twin compiles from outside `test/include/`. The recipe and the suite entry
land with the first physics scenario.

### 4. Scenarios this adds

Scripted world (step 1): the front reflex brakes a forward drive within
`REACTIVE_REFLEX_WORST_CASE_MS` and releases; a rear reading holds a reverse;
the cliff reflex brakes and backs off; the idle gaze moves the head only
between dwell poses and faces forward during a voice turn.

Physics plant (step 3): a `drive` goal toward a wall ends with the wheels
braked and no collision; on a table arena with the cliff reflex armed the
robot never leaves the floor. These assert ordering and bounds with wide
tolerances, not distances, until the plant is calibrated.

## Consequences

- The twin's planner inputs now come from the executor, which removes two
  divergences from the firmware (`robot_moving`, scripted reflex latches).
- The limitation list in the twin README shrinks: the executor is modelled,
  and the head and its dwell poses are when `head` is set. Still unmodelled:
  TTS playback time, the start cue, manual leases, and the microphone's
  quarantine after a head movement (`ambient_head_quiet`).
- `robocar-twin` stays buildable with a C compiler and cJSON alone; the plant
  is needed only for scenarios that name it.
- The simulation's CI install drops from the full Genesis stack to numpy,
  pymunk and pyyaml plus the dev extra, with no torch or genesis, including
  the `ty` sync that runs on every code PR. `ty` in CI then no longer resolves
  the `viz` packages, so the legacy demo modules are type-checked only against
  the core until the visualiser follow-up.
- The simulation's `--cov-fail-under=25` floor stays and is met by the plant's
  own tests, not by omitting the legacy modules from the measurement.
- PRD-003's bridge requirements are withdrawn. Nothing speaks that protocol
  since the dual-board firmware was removed, and the JSONL stepper replaces
  the WebSocket interface.
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
  simulation needs a harness re-lock. The process boundary costs under 0.2 s
  per simulated minute.
- **A kinematic stub inside the twin** (ADR-030 phase 4). No contact, no
  motor lag, no arena; it would be a third model of the car.
- **Executor opt-in per scenario.** Two code paths for the planner's inputs,
  one of which the robot never runs. The rehearsal showed the always-on
  executor changes no existing trace except the one that scripted `reflex`.
- **The every-tick rangefinder stub and the recording motor stub.** Both link
  with less glue, but the first has no read cycle and the second no ramp and
  no record of the `move_*` and `rotate_*` calls.
- **Retire the simulation.** Loses the only motor and contact model, which
  goal-to-motion scenarios need.
- **Port the visualiser to the plant now.** 851 lines with no test coverage
  and no headless check available in CI; deferred.

## Open questions

1. Bench measurements for `plant.yaml`: track width, wheel diameter, speed and
   yaw rate per duty, stopping distance, sensor offsets.
2. Whether the executor's boot range value (400 cm, valid, before the first
   read) reaches the planner on the robot as it does in the twin.
3. Whether the plant should select a frame's scene from the robot's pose, so
   a coupled scenario does not script frames by hand.
4. Whether the legacy `DifferentialDriveRobot` and Genesis visualiser are
   worth porting to the plant or are deleted.
