---
id: PRD-003
title: Robot Car Physics Simulation
status: active
created: 2026-03-05
---

# PRD-003: Robot Car Physics Simulation

## Problem Statement

Developing and validating robot movement logic on physical hardware is slow and risks
component damage. A software simulation that faithfully reproduces the ESP32 robot's
kinematics and communication protocol allows movement algorithms to be tested and iterated
without hardware in the loop.

## Goals

- Simulate differential drive kinematics and DC motor dynamics at 100 Hz.
- Mirror the exact UART command protocol used by the hardware so the same controller
  code drives both the simulation and the real robot.
- Provide a 2D visualization suitable for both interactive development and headless CI.
- Expose a WebSocket interface for external tools (e.g., dashboards, test harnesses).

## Non-Goals

- 3D rendering (Genesis 3D disabled; matplotlib 2D is the supported visualizer).
- Full sensor fusion / SLAM (sensor simulation is present but navigation is out of scope
  for this PRD).

## Requirements

### Physics Model

| Req | Description |
|-----|-------------|
| R1 | Differential drive kinematics: independent left/right PWM input (-255 to 255) |
| R2 | DC motor dynamics: model electrical resistance, back-EMF, and stall torque |
| R3 | Pymunk integration for collision detection and friction |
| R4 | Configurable robot geometry via `config/robot_config.yaml` |
| R5 | Simulation timestep configurable (default 0.01 s / 100 Hz) |

### Communication

R6–R8 are **withdrawn** by
[ADR-036](../decisions/ADR-036-robocar-twin-plant-port.md) (2026-11-13).
Nothing speaks that protocol since the dual-board firmware was removed, and the
bridge that implemented them (`communication_bridge.py`) is deleted. The
simulation's external interface is the JSONL stepper
`python -m robocar_sim.plant` (ADR-036 §3). What depended on the bridge goes
with it: the two Goals about the UART protocol and the WebSocket interface,
`websockets` and `pyserial` in R15, and the `--demo-only` flag in the Success
Criteria (the demo is the only mode left). R12, R14, R15 and the Success
Criteria below are written for the package as it is after that change.

| Req | Description |
|-----|-------------|
| ~~R6~~ | Withdrawn. WebSocket server on port 8765 accepting motor and servo command JSON messages |
| ~~R7~~ | Withdrawn. Serial UART bridge for connecting a real ESP32 to the virtual environment |
| ~~R8~~ | Withdrawn. Command protocol must match hardware: `MSG_MOVE`, `MSG_SOUND`, `MSG_SERVO` type codes |

### Visualization

| Req | Description |
|-----|-------------|
| R9 | 2D matplotlib rendering of robot position, trajectory, and obstacles |
| R10 | Headless mode (`--headless` / `--no-viz`) for CI and server environments |

### Testing

| Req | Description |
|-----|-------------|
| R11 | pytest suite with coverage reporting (`uv run --extra dev pytest tests/ --cov`) |
| R12 | Every test passes on the core and `dev` dependencies, which is what CI installs; a test that needs a `viz` package skips there. `uv run --extra dev pytest tests/ -q` in `packages/robotics/robocar/simulation` prints the counts |
| R13 | CI runs simulation build and test suite on every push |

### Package Management

| Req | Description |
|-----|-------------|
| R14 | Python 3.12 or newer (`requires-python` in `pyproject.toml`; `.python-version` selects 3.12); managed with `uv` |
| R15 | Core dependencies: `numpy`, `pymunk`, `pyyaml`. `matplotlib` and the visualiser's other packages are in the `viz` extra (ADR-036 §2) |

## Success Criteria

- The legacy demo (`uv run --extra viz python -m robocar_sim.main --headless`) completes a 30 s movement pattern without errors.
- Straight-line simulation matches physical robot trajectory within 2% over 5 m.
- All CI simulation checks pass on GitHub Actions (`ubuntu-latest`, Python 3.12).
