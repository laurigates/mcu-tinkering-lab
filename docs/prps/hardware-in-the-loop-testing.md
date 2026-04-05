# PRP: Hardware-in-the-Loop (HIL) Testing

**Status**: planned  
**Source**: Feature FR-007, PRD-003 (Robot Car Physics Simulation)  
**Priority**: P2  
**Confidence**: 6/10

---

## Goal

Connect real ESP32 hardware to the Python physics simulation, enabling automated integration tests that run firmware code against a virtual environment. This bridges the gap between host-based unit tests (no hardware) and full hardware tests (requires physical robot).

## Background

The `robocar-simulation` project provides a 2D Pymunk physics simulation of the robot car with a WebSocket bridge (port 8765). The simulation currently runs standalone or with PC-based test clients. Connecting real ESP32 firmware to the simulation via the WebSocket bridge would allow:

- Testing motor command handling with realistic physics feedback
- Validating AI inference → movement command pipelines end-to-end
- Running regression tests against firmware changes without a physical robot

## Architecture

```
ESP32 firmware (UART)
        ↕
    UART bridge (PC script)
        ↕
robocar-simulation (WebSocket :8765)
        ↕
    Test assertions (pytest)
```

## Implementation Plan

### Phase 1: UART bridge script
- Python script that forwards ESP32 UART output to WebSocket messages
- Parse motor commands (F/B/L/R/S) from UART and inject into simulation

### Phase 2: Simulation test harness
- Pytest fixtures for starting/stopping simulation
- Assertions on robot position, velocity after command sequences
- Timeout handling for commands that don't produce expected motion

### Phase 3: CI integration
- HIL test job requires physical ESP32 (not suitable for GitHub Actions runners)
- Local `just test-hil` recipe with hardware detection

## Acceptance Criteria

- [ ] Simulation responds to motor commands forwarded from real ESP32 via UART
- [ ] Pytest suite validates 5+ key command sequences
- [ ] Tests detect regressions in motor control logic
- [ ] Setup documented in `packages/esp32-projects/robocar-simulation/README.md`

## Related

- PRD-003: Robot Car Physics Simulation
- Feature FR-007: Hardware-in-the-loop testing
- `packages/esp32-projects/robocar-simulation/`
