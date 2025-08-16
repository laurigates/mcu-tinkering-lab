"""
Swift Visualizer for ESP32 Robot Car Simulation

This module provides 3D visualization using the Swift simulator backend
with the Robotics Toolbox for Python.
"""

import threading
import time
from typing import Dict, List, Optional

import numpy as np
import yaml

try:
    import roboticstoolbox as rtb
    import swift
    import trimesh
    from spatialmath import SE3
    from swift import Swift
    HAS_SWIFT = True
except ImportError:
    HAS_SWIFT = False
    print("Warning: Swift and Robotics Toolbox not available. Install with:")
    print("pip install robotics-toolbox-python swift-sim")

from robot_model import DifferentialDriveRobot, RobotState


class RobotVisualizer:
    """Simplified 3D visualization of the robot car using Swift"""
    
    def __init__(self, config_path: str, robot: DifferentialDriveRobot, viz_mode: str = 'headless'):
        if not HAS_SWIFT:
            print("Warning: Swift and Robotics Toolbox not available")
            print("3D visualization will be disabled")
            self.enabled = False
            return
        
        self.enabled = True
        self.config = self._load_config(config_path)
        self.robot = robot
        self.viz_mode = viz_mode
        
        # Swift environment - simplified initialization
        self.env = None
        self.robot_mesh = None
        self.wheel_meshes = []
        self.camera_mesh = None
        self.trail_points = []
        self.obstacles = []
        
        # Control variables
        self._running = False
        self._thread = None
        
        # Use configured update rate, with fallback
        simulation_timestep = self.config['simulation'].get('timestep', 0.01)
        self.update_rate = 1.0 / simulation_timestep  # Convert timestep to Hz
        self.trail_length = 100  # Trail visualization length
        
        # Initialize Swift environment directly
        self._init_swift_environment()
        
        if self.env:
            self._create_environment()
            self._create_robot_model()
        
    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file"""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    def _init_swift_environment(self):
        """Initialize Swift environment directly (simplified)"""
        try:
            print(f"Initializing Swift in {self.viz_mode} mode...")
            self.env = Swift()
            
            if self.viz_mode == 'browser':
                print("Launching browser mode...")
                self.env.launch(realtime=True, headless=False, browser='default')
                print("✓ Browser mode active at http://localhost:52000")
            elif self.viz_mode == 'visual':
                print("Launching visual mode...")
                self.env.launch(realtime=True, headless=False)
                print("✓ Visual mode active")
            else:  # headless
                print("Launching headless mode...")
                self.env.launch(realtime=True, headless=True)
                print("✓ Headless mode active")
                
        except Exception as e:
            print(f"Swift initialization failed: {e}")
            print("Attempting headless fallback...")
            try:
                self.env = Swift()
                self.env.launch(realtime=True, headless=True)
                print("✓ Headless fallback successful")
            except Exception as e2:
                print(f"Headless fallback failed: {e2}")
                self.env = None
                self.enabled = False
    
    def _create_environment(self):
        """Create the simulation environment"""
        if not self.env:
            return
            
        # Create ground plane
        ground_size = self.config['simulation']['environment']['size']
        ground = trimesh.primitives.Box(
            extents=[ground_size[0], ground_size[1], 0.01],
            transform=SE3(0, 0, -0.005).A
        )
        ground.visual.face_colors = [200, 200, 200, 255]
        self.env.add(ground)
        
        # Add obstacles
        if 'obstacles' in self.config['simulation']['environment']:
            for i, obstacle in enumerate(self.config['simulation']['environment']['obstacles']):
                self._create_obstacle(obstacle, i)
        
        # Add coordinate axes
        self._add_coordinate_axes()
        
        # Set camera view
        self._set_camera_view()
    
    def _create_obstacle(self, obstacle: Dict, index: int):
        """Create obstacle in the environment"""
        pos = obstacle['position']
        
        if obstacle['type'] == 'box':
            size = obstacle['size']
            mesh = trimesh.primitives.Box(
                extents=[size[0], size[1], 0.2],
                transform=SE3(pos[0], pos[1], 0.1).A
            )
            mesh.visual.face_colors = [255, 100, 100, 255]  # Red
            
        elif obstacle['type'] == 'cylinder':
            radius = obstacle['radius']
            mesh = trimesh.primitives.Cylinder(
                radius=radius,
                height=0.2,
                transform=SE3(pos[0], pos[1], 0.1).A
            )
            mesh.visual.face_colors = [100, 100, 255, 255]  # Blue
        
        else:
            return
        
        if self.env:
            self.env.add(mesh)
        self.obstacles.append(mesh)
    
    def _add_coordinate_axes(self):
        """Add coordinate axes to the environment"""
        if not self.env:
            return
            
        # X-axis (red)
        x_axis = trimesh.primitives.Cylinder(
            radius=0.01,
            height=0.5,
            transform=SE3(0.25, 0, 0.01) * SE3.Ry(np.pi/2).A
        )
        x_axis.visual.face_colors = [255, 0, 0, 255]
        self.env.add(x_axis)
        
        # Y-axis (green)
        y_axis = trimesh.primitives.Cylinder(
            radius=0.01,
            height=0.5,
            transform=SE3(0, 0.25, 0.01) * SE3.Rx(-np.pi/2).A
        )
        y_axis.visual.face_colors = [0, 255, 0, 255]
        self.env.add(y_axis)
        
        # Z-axis (blue)
        z_axis = trimesh.primitives.Cylinder(
            radius=0.01,
            height=0.5,
            transform=SE3(0, 0, 0.25).A
        )
        z_axis.visual.face_colors = [0, 0, 255, 255]
        self.env.add(z_axis)
    
    def _create_robot_model(self):
        """Create 3D model of the robot"""
        if not self.env:
            return
            
        # Robot dimensions
        robot_config = self.config['robot']
        length = robot_config['dimensions']['length']
        width = robot_config['dimensions']['width']
        height = robot_config['dimensions']['height']
        wheel_radius = robot_config['dimensions']['wheel_radius']
        
        # Main chassis
        chassis = trimesh.primitives.Box(
            extents=[length, width, height],
            transform=SE3(0, 0, height/2).A
        )
        chassis.visual.face_colors = [100, 150, 200, 255]  # Blue-gray
        self.robot_mesh = chassis
        self.env.add(self.robot_mesh)
        
        # Wheels
        wheel_positions = [
            [-length/4, -width/2, wheel_radius/2],  # Left rear
            [-length/4, width/2, wheel_radius/2],   # Right rear
            [length/4, -width/2, wheel_radius/2],   # Left front
            [length/4, width/2, wheel_radius/2]     # Right front
        ]
        
        for i, pos in enumerate(wheel_positions):
            wheel = trimesh.primitives.Cylinder(
                radius=wheel_radius,
                height=0.02,
                transform=SE3(pos[0], pos[1], pos[2]) * SE3.Rx(np.pi/2).A
            )
            wheel.visual.face_colors = [50, 50, 50, 255]  # Dark gray
            self.wheel_meshes.append(wheel)
            if self.env:
                self.env.add(wheel)
        
        # Camera (ESP32-CAM)
        camera_pos = robot_config['sensors']['camera']['position']
        camera = trimesh.primitives.Box(
            extents=[0.03, 0.03, 0.02],
            transform=SE3(camera_pos[0], camera_pos[1], camera_pos[2] + height/2).A
        )
        camera.visual.face_colors = [255, 255, 0, 255]  # Yellow
        self.camera_mesh = camera
        if self.env:
            self.env.add(self.camera_mesh)
        
        # Ultrasonic sensor
        ultrasonic = trimesh.primitives.Cylinder(
            radius=0.01,
            height=0.02,
            transform=SE3(length/2, 0, height/2).A
        )
        ultrasonic.visual.face_colors = [0, 255, 255, 255]  # Cyan
        if self.env:
            self.env.add(ultrasonic)
    
    def _set_camera_view(self):
        """Set optimal camera view"""
        if not self.env:
            return
        # Only set camera pose if cam attribute exists (not available in headless mode)
        if hasattr(self.env, 'cam') and self.env.cam:
            # Position camera to view the robot from above and at an angle
            self.env.cam.set_pose(SE3(2, 2, 3) * SE3.Rz(-np.pi/4) * SE3.Rx(-np.pi/6))
        else:
            print("Note: Camera control not available in headless mode")
    
    def update_robot_visualization(self):
        """Update robot visualization based on current state - simplified"""
        if not self.enabled or not self.env:
            return
            
        state = self.robot.get_state()
        self._update_robot_state(state)
    
    def _update_robot_state(self, state: RobotState):
        """Update robot visualization with new state"""
        if not self.env:
            return
            
        # Update robot position and orientation
        robot_transform = SE3(state.x, state.y, 0.05) * SE3.Rz(state.theta)
        
        # Update chassis
        if self.robot_mesh:
            self.robot_mesh.apply_transform(
                np.linalg.inv(self.robot_mesh.transform) @ robot_transform.A
            )
        
        # Update wheels
        wheel_positions = [
            [-0.05, -0.075, 0.0175],  # Left rear
            [-0.05, 0.075, 0.0175],   # Right rear
            [0.05, -0.075, 0.0175],   # Left front
            [0.05, 0.075, 0.0175]     # Right front
        ]
        
        for i, (wheel, local_pos) in enumerate(zip(self.wheel_meshes, wheel_positions)):
            wheel_transform = robot_transform * SE3(local_pos[0], local_pos[1], local_pos[2])
            wheel.apply_transform(
                np.linalg.inv(wheel.transform) @ wheel_transform.A
            )
        
        # Update camera with pan angle
        if self.camera_mesh:
            camera_config = self.config['robot']['sensors']['camera']
            camera_local_pos = camera_config['position']
            camera_pan = np.radians(state.camera_pan_angle)
            
            camera_transform = (robot_transform * 
                              SE3(camera_local_pos[0], camera_local_pos[1], camera_local_pos[2] + 0.05) *
                              SE3.Rz(camera_pan))
            
            self.camera_mesh.apply_transform(
                np.linalg.inv(self.camera_mesh.transform) @ camera_transform.A
            )
        
        # Update trail
        self._update_trail(state.x, state.y)
        
        # Update sensor visualization
        self._update_sensor_visualization(state)
        
        # Step the simulation
        try:
            self.env.step(0.033)  # ~30 FPS
        except Exception as e:
            # Ignore Swift step errors in simplified mode
            pass
    
    def _update_trail(self, x: float, y: float):
        """Update robot trail visualization"""
        # Add current position to trail
        self.trail_points.append([x, y, 0.01])
        
        # Limit trail length
        if len(self.trail_points) > self.trail_length:
            self.trail_points.pop(0)
        
        # Create trail line (simplified - in practice you'd use a line renderer)
        # This is a placeholder - Swift doesn't have a built-in trail renderer
        # You would typically use a line mesh or point cloud
    
    def _update_sensor_visualization(self, state: RobotState):
        """Update sensor visualization"""
        if not self.env:
            return
            
        # TODO: Fix ultrasonic ray visualization (trimesh Path3D color format issues)
        # Temporarily disabled to allow main visualization to work
        # 
        # # Visualize ultrasonic sensor reading
        # if hasattr(self, 'ultrasonic_ray'):
        #     self.env.remove(self.ultrasonic_ray)
        # 
        # # Create ray for ultrasonic sensor
        # ray_end_x = state.x + state.ultrasonic_distance * np.cos(state.theta)
        # ray_end_y = state.y + state.ultrasonic_distance * np.sin(state.theta)
        # 
        # # Create simple sphere marker instead of ray for now
        # marker = trimesh.primitives.Sphere(
        #     radius=0.02,
        #     center=[ray_end_x, ray_end_y, 0.05]
        # )
        # marker.visual.face_colors = [255, 0, 255, 128]  # Magenta
        # self.ultrasonic_ray = marker
        # self.env.add(self.ultrasonic_ray)
        pass
    
    def add_trajectory_point(self, x: float, y: float, color: List[int] = [255, 255, 255, 255]):
        """Add a trajectory point marker"""
        if not self.env:
            return
        marker = trimesh.primitives.Sphere(
            radius=0.01,
            center=[x, y, 0.02]
        )
        marker.visual.face_colors = color
        self.env.add(marker)
    
    def add_waypoint(self, x: float, y: float, z: float = 0.1):
        """Add a waypoint marker"""
        if not self.env:
            return
        waypoint = trimesh.primitives.Cylinder(
            radius=0.05,
            height=0.2,
            transform=SE3(x, y, z).A
        )
        waypoint.visual.face_colors = [255, 255, 0, 128]  # Semi-transparent yellow
        self.env.add(waypoint)
    
    def clear_markers(self):
        """Clear all trajectory points and waypoints"""
        # This would need to be implemented to track and remove markers
        pass
    
    def start_update_loop(self):
        """Start the visualization update loop - simplified"""
        if not self.enabled:
            return
            
        self._running = True
        
        # Start a simple update thread
        self._thread = threading.Thread(target=self._simple_update_loop, daemon=True)
        self._thread.start()
        print("✓ Visualization update loop started")
    
    def _simple_update_loop(self):
        """Simple update loop for visualization"""
        last_update = time.time()
        update_interval = 1.0 / self.update_rate
        
        while self._running:
            current_time = time.time()
            
            if current_time - last_update > update_interval:
                try:
                    self.update_robot_visualization()
                    last_update = current_time
                except Exception as e:
                    # Ignore visualization errors to keep simulation running
                    pass
            
            time.sleep(0.01)  # Small sleep to prevent busy waiting
    
    def _create_simple_robot_visualization(self):
        """Create a simple robot visualization for testing"""
        if not self.env:
            return
        
        # Simple robot representation
        robot_box = trimesh.primitives.Box(
            extents=[0.15, 0.10, 0.05],  # 15cm x 10cm x 5cm
            transform=SE3(0, 0, 0.025).A
        )
        robot_box.visual.face_colors = [100, 150, 200, 255]  # Blue
        self.robot_mesh = robot_box
        self.env.add(self.robot_mesh)
        
        # Add a direction indicator (arrow)
        arrow = trimesh.primitives.Cylinder(
            radius=0.01,
            height=0.08,
            transform=SE3(0.06, 0, 0.03) * SE3.Ry(np.pi/2).A
        )
        arrow.visual.face_colors = [255, 0, 0, 255]  # Red arrow
        self.env.add(arrow)
        
        print("✓ Simple robot visualization created")
    
    def stop(self):
        """Stop the visualization - simplified"""
        print("Stopping visualization...")
        self._running = False
        
        # Stop the update thread
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            
        # Clean up Swift environment
        if self.env:
            try:
                if hasattr(self.env, 'close'):
                    self.env.close()
                self.env = None
                print("✓ Swift environment cleaned up")
            except Exception as e:
                print(f"Note: Swift cleanup: {e}")
                self.env = None
    
    def screenshot(self, filename: str):
        """Take a screenshot of the visualization"""
        # This would need to be implemented in Swift
        pass
    
    def record_video(self, filename: str, duration: float):
        """Record a video of the simulation"""
        # This would need to be implemented in Swift
        pass


class SwiftSimulation:
    """Simplified Swift-based simulation environment"""
    
    def __init__(self, config_path: str, viz_mode: str = 'headless'):
        self.config_path = config_path
        self.viz_mode = viz_mode
        self.robot = DifferentialDriveRobot(config_path)
        self.visualizer = RobotVisualizer(config_path, self.robot, viz_mode=viz_mode)
        
        # Simulation control
        self.paused = False
        self.simulation_speed = 1.0
        
    def run_simulation(self, duration: float = 10.0):
        """Run the simulation for a specified duration"""
        print(f"Starting simulation for {duration} seconds...")
        
        # Start visualization if enabled
        if self.visualizer.enabled:
            self.visualizer.start_update_loop()
        
        start_time = time.time()
        last_update = start_time
        
        try:
            while time.time() - start_time < duration:
                if not self.paused:
                    current_time = time.time()
                    dt = (current_time - last_update) * self.simulation_speed
                    
                    # Update robot simulation
                    self.robot.update(dt)
                    
                    last_update = current_time
                
                time.sleep(0.01)  # 100Hz simulation rate
                
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        
        finally:
            if self.visualizer.enabled:
                self.visualizer.stop()
            print("Simulation complete")
    
    def set_motor_commands(self, left_pwm: int, right_pwm: int):
        """Set motor commands for the robot"""
        self.robot.set_motor_commands(left_pwm, right_pwm)
    
    def pause(self):
        """Pause the simulation"""
        self.paused = True
    
    def resume(self):
        """Resume the simulation"""
        self.paused = False
    
    def set_speed(self, speed: float):
        """Set simulation speed multiplier"""
        self.simulation_speed = max(0.1, min(10.0, speed))
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset simulation to initial state"""
        self.robot.reset(x, y, theta)
        if self.visualizer.enabled:
            self.visualizer.trail_points.clear()


def main():
    """Main function for testing simplified Swift visualization"""
    config_path = "../config/robot_config.yaml"
    
    try:
        # Create simulation
        print("Creating simulation...")
        sim = SwiftSimulation(config_path, viz_mode='headless')
        
        # Set some motor commands to make the robot move
        sim.set_motor_commands(100, 80)  # Slight turn
        
        # Run simulation
        sim.run_simulation(duration=10.0)
        
    except Exception as e:
        print(f"Error running simulation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    print("ESP32 Robot Car Swift Visualization (Simplified)")
    print("=" * 50)
    main()
