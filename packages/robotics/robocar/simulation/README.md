# Robocar Simulation

Two things live in this project, in one import package, `robocar_sim`:

- **The plant** (`robocar_sim.plant`): a deterministic model of the robocar's
  wheels, contacts, two rangefinders and two floor sensors. It is stepped with
  one wheel command per side and returns what the sensors would see. It runs
  alone for work on the motor model, an arena or the sensor geometry. Design:
  [ADR-036](../../../../docs/decisions/ADR-036-robocar-twin-plant-port.md),
  whose section 3 has the behaviour twin's harness ([`../twin`](../twin))
  start it as a child process for scenarios that name a physics plant.
- **The legacy demo** (`robocar_sim.main`, `robot_model`, `genesis_visualizer`,
  `camera_simulation`): a model of the removed dual-ESP32 car with a
  Matplotlib or Genesis viewer. It still describes the old chassis
  (`config/robot_config.yaml`) and is kept until the viewer is moved onto the
  plant or deleted (ADR-036, open question 4).

The plant needs numpy, pymunk and pyyaml. The legacy demo's viewer, camera and
entry point need the `viz` extra (genesis-world, torch, OpenCV, matplotlib).

The firmware is [`robocar`](../) (XIAO ESP32-S3 Sense). Nothing here talks to
hardware: the WebSocket and serial bridge, the WiFi and OTA models and the
Claude/Ollama client were removed with ADR-036.

## Setup

```bash
cd packages/robotics/robocar/simulation
uv sync --extra dev        # the plant, the robot model and the test tools
uv sync --extra dev --extra viz   # also the legacy demo's viewer (several GB)
```

`.python-version` pins Python 3.12. From the repo root every recipe is
`just sim::<recipe>` (`just --list sim`).

## The plant

```python
from robocar_sim.plant import Plant, Pose, WheelCmd

arena = {
    "floor": [[-50, -60], [150, -60], [150, 60], [-50, 60]],
    "walls": [{"from": [150, -60], "to": [150, 60]}],
}
plant = Plant(arena, seed=7, start=Pose(x_cm=0, y_cm=0, heading_deg=0))
reading = plant.step(33, left=WheelCmd("forward", 2409), right=WheelCmd("forward", 2409))
# reading.at_ms, reading.pose, reading.range_cm.front / .rear (int or None),
# reading.floor.front / .rear (bool), reading.collided, reading.off_floor
```

- `WheelCmd.drive` is what the TB6612FNG does: `forward`, `reverse`, `coast`
  (winding open) or `brake` (winding shorted). `duty` is the PCA9685 count,
  0..4095.
- The pose is the midpoint between the drive wheels, in centimetres, with the
  heading in degrees: 0 along +x, growing counter-clockwise.
- A range is whole centimetres, or `None` when nothing is within 2..400 cm of
  that sensor. The ray ignores the robot's own chassis.
- A floor sensor reads floor while its point is inside the arena's floor
  polygon. `off_floor` is true while a drive wheel's contact point is outside
  it. The plant does not model the fall.
- `collided` is true when the chassis touched a wall or an obstacle during the
  step. The chassis stops at the face: blocked wheels stall.
- Two plants built with the same arguments give identical readings. The seed
  feeds the rangefinder noise and nothing else. There is no wall clock, no
  thread and no file written.
- A step of any length is integrated in 1 ms sub-steps, so 33 ms ticks are
  stable and one 33 ms step equals thirty-three 1 ms steps.

The plant reports geometric truth every step. The rangefinders' read cycle,
the smoothing and the fail-closed rules are the firmware's (`ultrasonic_core`,
`cliff_core`), which the twin runs.

### The robot description

[`src/robocar_sim/plant.yaml`](src/robocar_sim/plant.yaml) is package data,
loaded from the package and never from the working directory. **Every chassis,
motor and mounting figure in it is an estimate**: the repo holds no measured
value for the current chassis. Each one is commented with the measurement that
replaces it. A distance or a time read off the plant is as good as those
estimates.

`Plant(..., config=mapping)` takes another description with the same keys.
Unknown and missing keys are errors.

### The arena

`robocar_sim.arena` owns the schema; its module docstring is the reference.
Centimetres and degrees, unknown keys refused:

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

The floor's edge is not a wall: a table top is a floor with no walls.

### The stepper

`python -m robocar_sim.plant` (`just sim::plant`) reads JSON lines on stdin
and answers each with one line on stdout. Nothing else is written to stdout.

```json
{"t":"init","seed":7,"arena":{"floor":[[-50,-60],[150,-60],[150,60],[-50,60]]},"start":{"x_cm":0,"y_cm":0,"heading_deg":0}}
{"t":"ready"}
{"t":"step","dt_ms":33,"left":{"drive":"forward","duty":2409},"right":{"drive":"forward","duty":2409}}
{"t":"reading","at_ms":33,"pose":{"x_cm":0.086,"y_cm":0.0,"heading_deg":0.0},"range_cm":{"front":null,"rear":null},"floor":{"front":true,"rear":true},"collided":false,"off_floor":false}
```

- The first line must be `init`; every later line is a `step`. `dt_ms` is an
  integer 1..1000.
- Each message carries exactly the keys shown. A missing or unknown key, a
  value out of range, an invalid arena or a line that is not JSON is answered
  with `{"t":"error","message":"..."}` and the process exits with status 1.
- Pose figures are rounded to 0.001 on the wire.
- End of input exits with status 0.

### The demo

```bash
just sim::plant-demo
```

steps the plant through a fixed list of wheel commands in a walled room and
writes one `reading` line per step to `tmp/plant_demo_trace.jsonl`. Where
matplotlib is installed (the `viz` extra) it also draws the arena and the path
to `tmp/plant_demo.png`; otherwise it says the plot was skipped.
`python -m robocar_sim.plant_demo --help` lists the options (`--arena`,
`--seed`, `--out`, `--plot`).

## The legacy demo

```bash
just sim::run            # headless
just sim::run-visual     # GUI window
just sim::run-browser    # browser viewer, http://localhost:52000
```

Each runs `python -m robocar_sim.main` under `uv run --extra viz`, which
installs the `viz` extra on first use. The demo drives a fixed PWM pattern on
`DifferentialDriveRobot` and shows it; see
[VISUALIZATION_CONFIG.md](VISUALIZATION_CONFIG.md) for the viewer modes.
`config/robot_config.yaml` is its configuration and describes the removed
dual-ESP32 car.

The root-level `debug_*.py` scripts are standalone analyses of the legacy
motor model (`uv run python debug_differential.py`).

Known limits of the legacy model, which the plant does not share: with Pymunk
on, its one rangefinder ray starts inside the chassis and never sees an
obstacle; the chassis sinks about 10 cm into an obstacle it drives at; the
demo steps it on the wall clock; it writes `simulation_errors.log` to the
working directory.

## Tests

```bash
just sim::test           # everything
just sim::test-plant     # the plant, the arena schema, the stepper
uv run --extra dev pytest tests/ --cov=src --cov-report=term   # with coverage, as CI runs it
```

CI (`test.yml`, the Python Simulation Tests job) runs the suite on the core and `dev`
dependencies with `--cov-fail-under=25`. Tests of the viewer and the camera
are skipped there because the `viz` extra is not installed.
`tests/test_imports.py` imports every module in the package, since `ty`
ignores unresolved imports and would not report a module that imports one
that was removed. The viewer, the camera and `main` cannot be imported there,
so the same file also reads every module's source and checks that each import
from within the package names a module and a name that exist.

```bash
just sim::lint           # ruff check
just sim::format         # ruff format
just sim::type-check     # ty check src/
```

## Releases

release-please owns `version.txt`, `CHANGELOG.md`, the `version` line in
`pyproject.toml` and the project's entry in `uv.lock` (component
`robocar-simulation`, distribution `esp32-robot-simulation`). Do not edit them
by hand.
