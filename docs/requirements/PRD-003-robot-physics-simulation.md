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

| Req | Description |
|-----|-------------|
| R6 | WebSocket server on port 8765 accepting motor and servo command JSON messages |
| R7 | Serial UART bridge for connecting a real ESP32 to the virtual environment |
| R8 | Command protocol must match hardware: `MSG_MOVE`, `MSG_SOUND`, `MSG_SERVO` type codes |

### Visualization

| Req | Description |
|-----|-------------|
| R9 | 2D matplotlib rendering of robot position, trajectory, and obstacles |
| R10 | Headless mode (`--headless` / `--no-viz`) for CI and server environments |

### Testing

| Req | Description |
|-----|-------------|
| R11 | pytest suite with coverage reporting (`uv run pytest --cov`) |
| R12 | Target: all non-physics-calibration tests pass (currently 18/24) |
| R13 | CI runs simulation build and test suite on every push |

### Package Management

| Req | Description |
|-----|-------------|
| R14 | Python 3.11 required; managed with `uv` (pyproject.toml) |
| R15 | Core dependencies: `pymunk`, `numpy`, `matplotlib`, `websockets`, `pyserial` |

## Success Criteria

- Demo mode (`--demo-only --headless`) completes a 30 s movement pattern without errors.
- Straight-line simulation matches physical robot trajectory within 2% over 5 m.
- All CI simulation checks pass on GitHub Actions (`ubuntu-latest`, Python 3.11).
