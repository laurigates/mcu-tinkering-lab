# Add example scripts and developer documentation

**Labels**: `documentation`, `enhancement`, `good first issue`, `priority: medium`

## Problem
New developers need practical examples to understand how to use the simulation. Current README is good but needs hands-on code examples.

## Proposed Examples

### Basic Examples (Simple, <50 lines each)
- [ ] `examples/01_basic_movement.py` - Move forward, turn, stop
- [ ] `examples/02_square_path.py` - Follow a square trajectory
- [ ] `examples/03_sensor_reading.py` - Read IMU and ultrasonic sensors
- [ ] `examples/04_motor_control.py` - Direct PWM control vs velocity control

### Communication Examples
- [ ] `examples/websocket_client.py` - Connect and send commands via WebSocket
- [ ] `examples/websocket_telemetry.py` - Subscribe to robot state updates
- [ ] `examples/serial_bridge.py` - Simulate ESP32 serial communication

### Advanced Examples
- [ ] `examples/trajectory_following.py` - Follow a predefined path (circle, figure-8)
- [ ] `examples/obstacle_avoidance.py` - Simple reactive navigation
- [ ] `examples/pid_tuning.py` - Tune PID parameters for motor control
- [ ] `examples/camera_capture.py` - Capture and save camera frames

### Developer Guides
- [ ] `docs/DEVELOPER_GUIDE.md` - How to extend the simulation
- [ ] `docs/ADDING_SENSORS.md` - Adding new sensor types
- [ ] `docs/PHYSICS_MODEL.md` - Understanding the physics equations
- [ ] `docs/COMMUNICATION_PROTOCOL.md` - I2C and WebSocket protocol details

## Example Template Structure
```python
#!/usr/bin/env python3
"""
Example: Basic Movement
Demonstrates how to create a robot and move it forward.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robot_model import DifferentialDriveRobot

def main():
    # Create robot
    robot = DifferentialDriveRobot('config/robot_config.yaml')

    # Move forward for 2 seconds
    robot.set_motor_commands(100, 100)
    for _ in range(200):  # 2 seconds at 100Hz
        robot.update()

    # Print final position
    print(f"Final position: ({robot.state.x:.2f}, {robot.state.y:.2f})")

if __name__ == "__main__":
    main()
```

## Documentation Improvements
- [ ] Add "Quick Examples" section to README
- [ ] Add API reference documentation
- [ ] Create troubleshooting guide
- [ ] Add common recipes/patterns

## Impact
- **Priority**: Medium ⭐⭐
- **Effort**: Low-Medium (1-2 hours)
- **Benefit**: Easier onboarding, reference implementations, better adoption

## Files to Create
- `examples/` directory with all example scripts
- `docs/` directory with developer guides
- Update `README.md` with links to examples

## Success Criteria
- [ ] At least 8 working example scripts
- [ ] Each example has clear comments and docstrings
- [ ] Examples can be run with `uv run python examples/xxx.py`
- [ ] Developer guide covers extending simulation
- [ ] Documentation is beginner-friendly

## Related
- Makes simulation more accessible
- Helps validate API design
- Good "good first issue" for contributors
