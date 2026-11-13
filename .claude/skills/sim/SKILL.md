---
name: sim
description: Run robocar physics simulation environment
argument-hint: "[options]"
allowed-tools: Bash(python:*), Bash(cd:*), Bash(uv:*), Bash(just:*), Read
---

# Robocar Physics Simulation

Run the robocar simulation: $ARGUMENTS

The project is `packages/robotics/robocar/simulation/`, import package
`robocar_sim`. It holds two things (its README has the detail):

- **The plant** (`robocar_sim.plant`, ADR-036): wheels, contacts, rangefinders
  and floor sensors, stepped with wheel commands. Needs numpy, pymunk and
  pyyaml only.
- **The legacy demo** (`robocar_sim.main`): the removed dual-ESP32 car with a
  Matplotlib or Genesis viewer. Needs the `viz` extra, a download of several GB
  (genesis-world, torch).

## Steps

1. Decide which one is asked for. A request about motion, an arena, the
   rangefinders or the floor sensors of the current robot is the plant. Only a
   request for the viewer is the legacy demo.
2. From the repo root, list the recipes: `just --list sim`.
3. The plant:
   - `just sim::plant-demo` steps it through a scripted drive and writes the
     pose trace to `tmp/plant_demo_trace.jsonl` in the project directory (and
     `tmp/plant_demo.png` where matplotlib is installed). Extra arguments go
     through, for example `just sim::plant-demo --arena path/to/arena.yaml --seed 3`.
   - `just sim::plant` is the JSONL stepper on stdin/stdout: one `init` line,
     then `step` lines (protocol in the README).
   - From Python: `uv run python -c "from robocar_sim.plant import Plant, Pose, WheelCmd; ..."`
     in the project directory.
4. The legacy demo: `just sim::run` (headless), `just sim::run-visual`,
   `just sim::run-browser`. Say before running that the first run installs the
   `viz` extra. Its options are `--headless`, `--visual`, `--browser`,
   `--viz-mode` and `--config`.
5. Tests: `just sim::test` after `uv sync --extra dev` in the project directory.

Every chassis, motor and mounting figure in the plant's robot description
(`src/robocar_sim/plant.yaml`) is an estimate. Report a distance or a time
from the plant as the model's, not as a measurement of the robot.

Report any errors or missing dependencies clearly.
