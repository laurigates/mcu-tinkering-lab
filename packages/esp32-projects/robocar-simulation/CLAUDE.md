# CLAUDE.md — robocar-simulation

Python physics simulation environment for an ESP32-based differential drive robot car.

## Quick Reference

```bash
just --list          # All available recipes
just run             # Headless simulation
just run-visual      # GUI window (matplotlib)
just run-browser     # Browser-based (http://localhost:52000)
just test            # Run test suite
just lint            # Ruff check
just format          # Ruff format
just type-check      # Mypy
```

**Package manager**: `uv` (primary). Use `uv run <cmd>` for one-off commands. Run `uv sync --extra dev` to install dev dependencies.

**Entry point**: `src/main.py` → `SimulationManager`

---

## Architecture

### Core Modules (`src/`)

| Module | Purpose |
|--------|---------|
| `main.py` | `SimulationManager` — async coordination, CLI, signal handling |
| `robot_model.py` | `DifferentialDriveRobot`, `DCMotor`, `PhysicsEngine` (Pymunk) |
| `motor_controller.py` | PID control, encoder simulation, velocity filtering |
| `communication_bridge.py` | WebSocket/Serial/I2C protocol server |
| `genesis_visualizer.py` | Genesis 3D visualization (with Matplotlib 2D fallback when genesis-world is unavailable) |
| `camera_simulation.py` | Synthetic camera feed (OpenCV) |
| `wifi_simulation.py` | WiFi state machine |
| `ota_simulation.py` | OTA update flow, partition management |
| `ai_command_processor.py` | Claude/Ollama AI backend integration |
| `error_handling.py` | Resilience framework, circuit breaker, component health |

### Data Flow

```
ESP32 Hardware (Serial/WebSocket)
    → Communication Bridge (I2C protocol interpreter)
    → Robot Model (DifferentialDriveRobot)
        → Motor Controller (PWM → velocity via PID)
        → Physics Engine (Pymunk forces, collisions)
    → Visualization (Matplotlib)
    → Communication Bridge (sensor data back to hardware)
```

### Physics

- **Engine**: Pymunk 2D rigid body physics
- **Motor model**: Electrical (back-EMF, inductance) + mechanical (friction, inertia)
- **Kinematics**: `v = (v_L + v_R) / 2`, `ω = (v_R - v_L) / track_width`
- **Arena**: 5×5m with configurable obstacles (boxes, cylinders)

---

## Configuration

`config/robot_config.yaml` is the single source of truth for all tunable parameters:

- `robot`: Physical specs (dimensions, mass, motor params, sensors)
- `simulation`: Physics timestep (10ms/100Hz), PID gains (Kp=15, Ki=5, Kd=0.5), arena
- `ai_backend`: Claude or Ollama; uses `${CLAUDE_API_KEY}` env var

---

## Communication Protocol

The simulation mirrors the **exact I2C message format** used in ESP32 firmware (`robocar-main`):

```python
# Python (simulation) — matches ESP32 C firmware byte-for-byte
class MessageType(Enum):
    MOVE = 0x01
    SERVO = 0x03
    STATUS = 0x05
    AI_COMMAND = 0x07
    # ...
```

- **WebSocket**: `localhost:8765` — JSON messages `{"type": "motor_command", "payload": {...}}`
- **Serial UART**: 115200 bps with CRC8 checksum — connect real ESP32 via `just run-serial /dev/ttyUSB0`

---

## Test Suite

```bash
just test                          # All tests
uv run pytest tests/ -v            # Verbose
uv run pytest -m "not slow"        # Skip slow tests
uv run pytest tests/test_robot_model.py  # Specific file
```

**Current status**: 18/24 tests pass. 6 failing tests are physics calibration tests (energy conservation, velocity matching accuracy) — non-critical, known issues.

**Test markers**: `slow`, `integration`, `hardware`

---

## Development Notes

- **Genesis 3D** is enabled (`genesis-world>=0.2.0` supports Python 3.11). Matplotlib 2D is used as a fallback when genesis-world is not installed.
- Debug scripts at the repo root (`debug_motor_dynamics.py`, `debug_differential.py`, etc.) are standalone analysis tools, not part of the main simulation.
- The `migrate_to_uv.py` and `setup_uv.py` scripts are one-time migration artifacts — they can be ignored.
- Hardware validation accuracy: straight line 98.5%, circular motion 96.8%, motor response within 3%.
