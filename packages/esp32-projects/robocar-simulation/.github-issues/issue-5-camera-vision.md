# Enhance camera simulation with synthetic scene rendering

**Labels**: `enhancement`, `computer vision`, `priority: medium`

## Problem
Camera simulation exists but generates basic frames. Need synthetic scene rendering to test computer vision algorithms before deploying to real ESP32-CAM hardware.

## Current State
- `CameraSimulation` class exists in `src/camera_simulation.py`
- OpenCV is available
- Basic frame capture infrastructure present
- No synthetic scene generation

## Proposed Features

### Scene Rendering
- [ ] Generate synthetic RGB images based on robot position/orientation
- [ ] Render floor with checkerboard or grid pattern
- [ ] Render obstacles from simulation config (boxes, cylinders)
- [ ] Render wall boundaries
- [ ] Apply camera perspective projection

### Camera Model
- [ ] Implement proper camera intrinsics (FOV, focal length)
- [ ] Add lens distortion simulation (barrel/pincushion)
- [ ] Add motion blur when robot is moving fast
- [ ] Add realistic noise (sensor noise, compression artifacts)
- [ ] Support different resolutions (from config)

### Visual Features
- [ ] Add ArUco marker detection/simulation
- [ ] Add line following track rendering
- [ ] Add colored objects for detection testing
- [ ] Add lighting simulation (shadows, brightness variation)

### Computer Vision Integration
- [ ] Example: Line detection algorithm
- [ ] Example: ArUco marker tracking
- [ ] Example: Simple object detection
- [ ] Example: Visual odometry testing
- [ ] Export camera calibration matrix

### Performance
- [ ] Optimize rendering for real-time (20-30 FPS)
- [ ] Add frame caching for static scenes
- [ ] Support headless rendering (for CI/CD)
- [ ] GPU acceleration if available

## Implementation Approach

### Option 1: OpenCV Drawing (Simple)
- Use cv2.rectangle, cv2.circle, etc.
- Fast and simple
- Limited realism

### Option 2: PyGame/Matplotlib (Medium)
- Better graphics capabilities
- 2D rendering with perspective
- Moderate performance

### Option 3: OpenGL/PyOpenGL (Advanced)
- Full 3D rendering pipeline
- Realistic lighting and shadows
- Best quality but more complex

**Recommendation**: Start with Option 1 (OpenCV), upgrade later if needed

## Example Usage
```python
from camera_simulation import CameraSimulation
robot = DifferentialDriveRobot('config.yaml')

# Camera automatically generates synthetic frames
for i in range(100):
    robot.update()
    frame = robot.state.camera_frame
    # Process frame with CV algorithm
    detected_objects = detect_objects(frame)
```

## Impact
- **Priority**: Medium ⭐⭐
- **Effort**: Medium-High (3-4 hours)
- **Benefit**: Test CV algorithms without hardware, complete simulation environment

## Files to Modify
- `src/camera_simulation.py` - Add scene rendering
- Create `src/scene_renderer.py` - Separate rendering logic
- Update `config/robot_config.yaml` - Add scene configuration

## Testing
- [ ] Visual validation: saved frames look correct
- [ ] Performance: maintains 20+ FPS
- [ ] Integration: works with existing robot simulation
- [ ] Examples: line following, marker detection work

## Future Enhancements
- ESP32-CAM streaming integration
- Real camera image + synthetic overlay
- Record and replay camera data
- Multi-camera support

## Related
- Complements WiFi/OTA simulation (PR #43)
- Enables full autonomous robot testing in simulation
- Prepares for ESP32-CAM hardware integration
