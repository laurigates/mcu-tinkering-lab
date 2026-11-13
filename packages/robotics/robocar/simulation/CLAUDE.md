# CLAUDE.md — robocar-simulation

The robocar plant (wheels, contacts, rangefinders, floor sensors) that the
behaviour twin steps in lockstep, and the legacy demo of the removed
dual-ESP32 car. Design: [ADR-036](../../../../docs/decisions/ADR-036-robocar-twin-plant-port.md).
Usage, the stepper protocol and the arena schema: [README.md](README.md).

## Quick Reference

```bash
just --list          # All available recipes (from the repo root: just --list sim)
just plant-demo      # Step the plant through scripted wheel commands
just plant           # The JSONL stepper on stdin/stdout
just test            # Run test suite
just test-plant      # Plant, arena and stepper tests only
just lint            # Ruff check
just format          # Ruff format
just type-check      # ty (astral)
just run             # Legacy demo, headless (installs the viz extra)
```

**Package manager**: `uv`. `uv sync --extra dev` installs the plant, the robot
model and the test tools. Python is pinned to 3.12 (`.python-version`).

**Import package**: `robocar_sim`, at `src/robocar_sim/`, with relative imports
inside it. Tests and scripts import `robocar_sim.<module>`; nothing edits
`sys.path`.

---

## Modules (`src/robocar_sim/`)

| Module | Purpose | Needs |
|--------|---------|-------|
| `plant.py` | `Plant`, `WheelCmd`, `Pose`, `Reading`; `python -m robocar_sim.plant` is the JSONL stepper | core |
| `arena.py` | The arena schema and its validation (`parse_arena`, `load_arena`) | core |
| `plant.yaml` | The plant's robot description, package data. Every chassis, motor and mounting figure is an estimate | — |
| `plant_demo.py` | Scripted drive, pose trace, plot where matplotlib is installed | core (plot: viz) |
| `robot_model.py` | `DCMotor` and `PhysicsEngine` (used by the plant), `DifferentialDriveRobot` (legacy) | core |
| `motor_controller.py` | Legacy PID control and encoder model | core |
| `error_handling.py` | Legacy error handler; a process singleton that writes `simulation_errors.log` | core |
| `camera_simulation.py` | Legacy synthetic camera feed | viz (OpenCV, spatialmath) |
| `genesis_visualizer.py` | Legacy Genesis 3D viewer with a Matplotlib 2D fallback | viz |
| `main.py` | Legacy demo entry point, `SimulationManager` | viz |

Core is numpy, pymunk and pyyaml. `viz` is the one optional runtime extra.

## Rules

- **The plant stays deterministic.** One numpy generator seeded from `seed`,
  no wall clock, no threads, no process-wide state, no file written, no
  swallowed exception. `tests/test_plant.py` and `tests/test_plant_stepper.py`
  hold it, in-process and across processes.
- **The plant does not go through `DifferentialDriveRobot`**, `error_handling`
  or `motor_controller`. It reuses `DCMotor` and `PhysicsEngine` only.
- **Keep the core light.** ADR-036 section 3 has the twin harness start the
  plant with `uv run --project <this directory> --locked`, so a dependency
  added to `[project] dependencies` is installed for every coupled scenario. Anything
  the viewer needs goes in the `viz` extra.
- **A module that imports a `viz` package at module level** is listed with
  that package in `tests/test_imports.py`, which imports every module in the
  package. `ty` ignores unresolved imports (`import-not-found = "ignore"`), so
  that test is what catches an import of a removed module. CI does not install
  `viz`, so `ty` there checks the legacy viewer modules only against the core.
- **The stepper's stdout carries protocol lines only.** Diagnostics go to
  stderr. Its messages and the arena schema are an interface with the twin
  harness (`../twin/harness`): change them together with ADR-036.
- **A new figure in `plant.yaml` is marked as an estimate** with the
  measurement that replaces it, unless the repo holds a measured value to cite.
- **`DCMotor` and `PhysicsEngine` are shared with the legacy model.** The
  plant's additions to them are optional arguments whose defaults keep the
  legacy behaviour; `tests/test_robot_model.py` holds that.
- **release-please owns** `version.txt`, `CHANGELOG.md`, the `version` line in
  `pyproject.toml` and this project's entry in `uv.lock`. The Pillow pin in the
  `viz` extra and the `[tool.uv]` override stay equal
  (`tests/test_pillow_override.py`).
- **Regenerate `uv.lock` with the uv version CI pins**
  (`.github/actions/setup-tools/action.yml`).

## Physics

- **Motors**: `DCMotor` (back EMF, a first-order current lag, friction),
  integrated in 1 ms sub-steps. `coast` opens the winding, `brake` shorts it,
  `forward`/`reverse` apply `supply_v * duty / 4096`.
- **Kinematics**: `v = (v_L + v_R) / 2`, `ω = (v_R - v_L) / track`, written to a
  Pymunk body each sub-step.
- **Contacts**: Pymunk resolves them, and the pose is read back from the body.
  While the chassis is in contact the wheel speeds are taken from the resolved
  body velocity (blocked wheels stall), which is what keeps the chassis at the
  obstacle's face. The plant sets Chipmunk's `collision_slop` to 0.5 mm; the
  default of 0.1 is 10 cm in metres.
- **Rangefinders**: one ray per sensor, 1 mm in radius, in the robot shape's
  filter group so it does not hit the chassis. The ray starts 1 cm behind the
  sensor, so a sensor on the edge of a chassis held against a wall still sees
  that wall (and reads `None`, under 2 cm) and not what lies behind it.

## Tests

```bash
just test
uv run --extra dev pytest tests/ --cov=src --cov-report=term --cov-fail-under=25 -v   # what CI runs
```

CI enforces `--cov-fail-under=25` over the whole package on the core and `dev`
dependencies; do not omit a module from the measurement to meet it. The floor
is passed without the plant's tests too (see `.claude/rules/testing.md`), so
read `plant.py` and `arena.py` in the coverage table when changing them.
Tests that need a `viz` package skip where it is not installed.
The legacy robot-model tests write `simulation_errors.log` to the working
directory (git-ignored).

**Test markers**: `slow`, `integration`, `hardware`

## Legacy demo

`config/robot_config.yaml` describes the removed dual-ESP32 car and is read
only by the legacy modules. The `debug_*.py` scripts at the project root are
standalone analyses of the legacy motor model. Moving the viewer onto the
plant, or deleting the legacy model, is an open question in ADR-036.
