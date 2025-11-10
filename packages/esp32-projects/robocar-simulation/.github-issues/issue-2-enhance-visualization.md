# Enhance 2D matplotlib visualization

**Labels**: `enhancement`, `visualization`, `priority: medium`

## Problem
Current matplotlib visualization is basic fallback mode. Need better real-time visualization for debugging and demos.

## Current State
- Basic 2D plot exists but not actively updated
- No real-time animation
- No trajectory trail
- No sensor visualization
- Headless mode works well

## Proposed Features

### Core Enhancements
- [ ] Real-time animated robot position updates (FuncAnimation)
- [ ] Trajectory trail showing robot path (last 100 points)
- [ ] Robot orientation indicator (arrow showing heading)
- [ ] Smooth animation at 10-30 FPS

### Sensor Visualization
- [ ] Ultrasonic sensor beam/cone visualization
- [ ] Camera FOV indicator (triangle showing view)
- [ ] Distance measurement labels
- [ ] IMU orientation visualization

### Environment
- [ ] Obstacle rendering (boxes, cylinders from config)
- [ ] Boundary walls
- [ ] Waypoint markers
- [ ] Grid background

### Interactive Controls (Nice-to-have)
- [ ] Pause/Resume button
- [ ] Speed control slider (0.1x - 10x)
- [ ] Reset button
- [ ] Status text overlay (position, velocity, motor PWM)

## Impact
- **Priority**: Medium ⭐⭐
- **Effort**: Medium (2-3 hours)
- **Benefit**: Better debugging, demo-friendly, visual feedback

## Files to Modify
- `src/genesis_visualizer.py` - MatplotlibVisualizer class
- Enhance `start_update_loop()` and `animate()` methods
- Add sensor visualization methods

## Example Reference
Look at existing matplotlib robot visualization examples:
- Peter Corke's Robotics Toolbox visualizations
- Mobile robot simulators with matplotlib

## Related
- Replaces Genesis 3D which is currently disabled
- Should work headless and in GUI mode
