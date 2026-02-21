"""
Genesis Visualizer for ESP32 Robot Car Simulation

This module provides 3D visualization using the Genesis simulation framework.
"""

import threading
import time
from typing import Dict, List, Optional
import queue
import sys
import os

import numpy as np
import yaml

try:
    import genesis as gs
    import trimesh
    from spatialmath import SE3

    HAS_GENESIS = True
except ImportError:
    HAS_GENESIS = False
    print("Info: Genesis simulation framework not available - using matplotlib fallback")

# Configure matplotlib for thread safety BEFORE importing pyplot
import matplotlib

# Use Agg backend for headless, TkAgg for GUI (with main thread requirement)
if "DISPLAY" not in os.environ or os.environ.get("TERM_PROGRAM") == "vscode":
    matplotlib.use("Agg")  # Headless backend for servers/containers
else:
    # Use default GUI backend but ensure main thread usage
    pass

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

from robot_model import DifferentialDriveRobot, RobotState


class MatplotlibVisualizer:
    """2D visualization fallback using matplotlib with thread safety"""

    def __init__(self, config_path: str, robot: DifferentialDriveRobot, viz_mode: str = "visual"):
        self.config_path = config_path
        self.robot = robot
        self.viz_mode = viz_mode
        self.enabled = viz_mode in ["visual", "browser"]
        self.is_main_thread = threading.current_thread() is threading.main_thread()
        self.update_queue = queue.Queue()
        self._running = False

        # Check if we can safely use GUI
        if self.enabled and not self.is_main_thread:
            print("Warning: Matplotlib visualization requested from non-main thread.")
            print("Switching to headless mode to prevent crash.")
            self.enabled = False
            self.viz_mode = "headless"

        if self.enabled:
            try:
                self.fig, self.ax = plt.subplots(figsize=(10, 8))
                self.ax.set_xlim(-5, 5)
                self.ax.set_ylim(-5, 5)
                self.ax.set_aspect("equal")
                self.ax.grid(True, alpha=0.3)
                self.ax.set_title("ESP32 Robot Car Simulation (2D View)")

                # Robot representation
                self.robot_patch = patches.Circle((0, 0), 0.2, color="blue", alpha=0.7)
                self.direction_line = None
                self.trail_points = []

                self.ax.add_patch(self.robot_patch)
                print("✓ Matplotlib visualizer initialized on main thread")
            except Exception as e:
                print(f"Warning: Failed to initialize matplotlib GUI: {e}")
                print("Falling back to headless mode")
                self.enabled = False

    def start_update_loop(self):
        """Start the visualization update loop with thread safety"""
        if not self.enabled:
            return

        if not self.is_main_thread:
            print("Error: Cannot start matplotlib GUI from non-main thread")
            return

        def animate(frame):
            try:
                state = self.robot.get_state()

                # Update robot position
                self.robot_patch.center = (state.x, state.y)

                # Update direction indicator
                if self.direction_line:
                    self.direction_line.remove()

                direction_x = 0.3 * np.cos(state.theta)
                direction_y = 0.3 * np.sin(state.theta)
                self.direction_line = self.ax.arrow(
                    state.x, state.y, direction_x, direction_y,
                    head_width=0.1, head_length=0.1, fc="red", ec="red"
                )

                # Add trail point
                self.trail_points.append((state.x, state.y))
                if len(self.trail_points) > 100:  # Keep last 100 points
                    self.trail_points.pop(0)

                # Draw trail
                if len(self.trail_points) > 1:
                    trail_x, trail_y = zip(*self.trail_points)
                    if hasattr(self, "trail_line"):
                        self.trail_line.remove()
                    (self.trail_line,) = self.ax.plot(
                        trail_x, trail_y, "g-", alpha=0.5, linewidth=2
                    )

                return [self.robot_patch]
            except Exception as e:
                # Silently handle animation errors to prevent crash
                return []

        try:
            # Fix FuncAnimation warnings
            self.anim = FuncAnimation(
                self.fig,
                animate,
                interval=50,
                blit=False,
                cache_frame_data=False,  # Prevent cache warning
                save_count=100,  # Limit frame cache
            )

            # Only show if we're on the main thread
            plt.show(block=False)
            plt.pause(0.1)
            self._running = True
            print("✓ Matplotlib animation started successfully")

        except Exception as e:
            print(f"Warning: Failed to start matplotlib animation: {e}")
            print("Continuing without animation")

    def update_robot_pose(self, x: float, y: float, theta: float):
        """Update robot visualization pose"""
        if not self.enabled:
            return

        self.robot_patch.center = (x, y)
        plt.draw()
        plt.pause(0.01)

    def close(self):
        """Close visualization safely"""
        self._running = False
        if self.enabled and hasattr(self, "fig"):
            try:
                if hasattr(self, "anim"):
                    self.anim.event_source.stop()
                plt.close(self.fig)
                print("✓ Matplotlib visualization closed")
            except Exception as e:
                print(f"Note: Error closing matplotlib: {e}")


class RobotVisualizer:
    """3D visualization of the robot car using Genesis simulation framework"""

    DEFAULT_FPS = 60

    def __init__(self, config_path: str, robot: DifferentialDriveRobot, viz_mode: str = "headless"):
        self.using_fallback = False

        if not HAS_GENESIS:
            # Use matplotlib fallback for visualization
            self.fallback_viz = MatplotlibVisualizer(config_path, robot, viz_mode)
            self.enabled = self.fallback_viz.enabled
            self.using_fallback = True
            return

        self.enabled = True
        self.config = self._load_config(config_path)
        self.robot = robot
        self.viz_mode = viz_mode

        # Genesis environment
        self.scene = None
        self.robot_entity = None
        self.wheel_entities = []
        self.camera_entity = None
        self.trail_points = []
        self.obstacles = []

        # Control variables
        self._running = False
        self._thread = None

        # Use configured update rate, with fallback
        simulation_timestep = self.config["simulation"].get("timestep", 0.01)
        self.update_rate = 1.0 / simulation_timestep  # Convert timestep to Hz
        self.trail_length = 100  # Trail visualization length

        # Initialize Genesis environment
        self._init_genesis_environment()

        if self.scene:
            self._create_environment()
            self._create_robot_model()

    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file"""
        with open(config_path, "r") as f:
            return yaml.safe_load(f)

    def _init_genesis_environment(self):
        """Initialize Genesis simulation environment"""
        try:
            print(f"Initializing Genesis in {self.viz_mode} mode...")

            # Initialize Genesis
            gs.init(backend=gs.cpu if self.viz_mode == "headless" else gs.gpu)

            # Create scene
            self.scene = gs.Scene(
                show_viewer=(self.viz_mode in ["visual", "browser"]),
                viewer_options=gs.options.ViewerOptions(
                    camera_pos=(2.0, 2.0, 2.0),
                    camera_lookat=(0.0, 0.0, 0.0),
                    max_FPS=RobotVisualizer.DEFAULT_FPS if self.viz_mode != "headless" else None,
                ),
            )

            print(f"✓ Genesis {self.viz_mode} mode initialized successfully")

        except Exception as e:
            print(f"Genesis initialization failed: {e}")
            print("Falling back to headless mode...")
            try:
                gs.init(backend=gs.cpu)
                self.scene = gs.Scene(show_viewer=False)
                self.viz_mode = "headless"
                print("✓ Genesis headless fallback successful")
            except Exception as e2:
                print(f"Genesis headless fallback failed: {e2}")
                self.scene = None
                self.enabled = False

    def _create_environment(self):
        """Create the simulation environment"""
        if not self.scene:
            return

        # Create ground plane
        ground_size = self.config["simulation"]["environment"]["size"]
        ground = self.scene.add_entity(
            gs.morphs.Box(size=(ground_size[0], ground_size[1], 0.01), pos=(0, 0, -0.005)),
            material=gs.materials.Rigid(color=(0.8, 0.8, 0.8)),
        )

        # Add obstacles
        if "obstacles" in self.config["simulation"]["environment"]:
            for i, obstacle in enumerate(self.config["simulation"]["environment"]["obstacles"]):
                self._create_obstacle(obstacle, i)

        # Build the scene
        self.scene.build()

    def _create_obstacle(self, obstacle: Dict, index: int):
        """Create obstacle in the environment"""
        pos = obstacle["position"]

        if obstacle["type"] == "box":
            size = obstacle["size"]
            obstacle_entity = self.scene.add_entity(
                gs.morphs.Box(size=(size[0], size[1], 0.2), pos=(pos[0], pos[1], 0.1)),
                material=gs.materials.Rigid(color=(1.0, 0.4, 0.4)),  # Red
            )

        elif obstacle["type"] == "cylinder":
            radius = obstacle["radius"]
            obstacle_entity = self.scene.add_entity(
                gs.morphs.Cylinder(radius=radius, height=0.2, pos=(pos[0], pos[1], 0.1)),
                material=gs.materials.Rigid(color=(0.4, 0.4, 1.0)),  # Blue
            )

        else:
            return

        self.obstacles.append(obstacle_entity)

    def _add_coordinate_axes(self):
        """Add coordinate axes to the environment"""
        if not self.scene:
            return

        # X-axis (red)
        x_axis = self.scene.add_entity(
            gs.morphs.Cylinder(
                radius=0.01,
                height=0.5,
                pos=(0.25, 0, 0.01),
                quat=gs.quat_from_euler([0, np.pi / 2, 0]),
            ),
            material=gs.materials.Rigid(color=(1.0, 0.0, 0.0)),
        )

        # Y-axis (green)
        y_axis = self.scene.add_entity(
            gs.morphs.Cylinder(
                radius=0.01,
                height=0.5,
                pos=(0, 0.25, 0.01),
                quat=gs.quat_from_euler([-np.pi / 2, 0, 0]),
            ),
            material=gs.materials.Rigid(color=(0.0, 1.0, 0.0)),
        )

        # Z-axis (blue)
        z_axis = self.scene.add_entity(
            gs.morphs.Cylinder(radius=0.01, height=0.5, pos=(0, 0, 0.25)),
            material=gs.materials.Rigid(color=(0.0, 0.0, 1.0)),
        )

    def _create_robot_model(self):
        """Create 3D model of the robot"""
        if not self.scene:
            return

        # Robot dimensions
        robot_config = self.config["robot"]
        length = robot_config["dimensions"]["length"]
        width = robot_config["dimensions"]["width"]
        height = robot_config["dimensions"]["height"]
        wheel_radius = robot_config["dimensions"]["wheel_radius"]

        # Main chassis
        self.robot_entity = self.scene.add_entity(
            gs.morphs.Box(size=(length, width, height), pos=(0, 0, height / 2)),
            material=gs.materials.Rigid(color=(0.4, 0.6, 0.8)),  # Blue-gray
        )

        # Wheels
        wheel_positions = [
            [-length / 4, -width / 2, wheel_radius / 2],  # Left rear
            [-length / 4, width / 2, wheel_radius / 2],  # Right rear
            [length / 4, -width / 2, wheel_radius / 2],  # Left front
            [length / 4, width / 2, wheel_radius / 2],  # Right front
        ]

        for i, pos in enumerate(wheel_positions):
            wheel = self.scene.add_entity(
                gs.morphs.Cylinder(
                    radius=wheel_radius,
                    height=0.02,
                    pos=pos,
                    quat=gs.quat_from_euler([np.pi / 2, 0, 0]),
                ),
                material=gs.materials.Rigid(color=(0.2, 0.2, 0.2)),  # Dark gray
            )
            self.wheel_entities.append(wheel)

        # Camera (ESP32-CAM)
        camera_pos = robot_config["sensors"]["camera"]["position"]
        self.camera_entity = self.scene.add_entity(
            gs.morphs.Box(
                size=(0.03, 0.03, 0.02),
                pos=(camera_pos[0], camera_pos[1], camera_pos[2] + height / 2),
            ),
            material=gs.materials.Rigid(color=(1.0, 1.0, 0.0)),  # Yellow
        )

        # Ultrasonic sensor
        ultrasonic = self.scene.add_entity(
            gs.morphs.Cylinder(radius=0.01, height=0.02, pos=(length / 2, 0, height / 2)),
            material=gs.materials.Rigid(color=(0.0, 1.0, 1.0)),  # Cyan
        )

    def _set_camera_view(self):
        """Set optimal camera view"""
        if not self.scene:
            return
        # Camera view is set during scene initialization with ViewerOptions
        if hasattr(self.scene, "viewer") and self.scene.viewer:
            print("✓ Camera positioned to view robot from optimal angle")
        else:
            print("Note: Camera control not available in headless mode")

    def update_robot_visualization(self):
        """Update robot visualization based on current state"""
        if not self.enabled or not self.scene:
            return

        state = self.robot.get_state()
        self._update_robot_state(state)

    def _update_robot_state(self, state: RobotState):
        """Update robot visualization with new state"""
        if not self.scene:
            return

        # Update robot position and orientation using Genesis entity transforms
        robot_pos = (state.x, state.y, 0.05)
        robot_quat = gs.quat_from_euler([0, 0, state.theta])

        # Update chassis
        if self.robot_entity:
            self.robot_entity.set_pos(robot_pos)
            self.robot_entity.set_quat(robot_quat)

        # Update wheels with robot transform
        wheel_local_positions = [
            [-0.05, -0.075, 0.0175],  # Left rear
            [-0.05, 0.075, 0.0175],  # Right rear
            [0.05, -0.075, 0.0175],  # Left front
            [0.05, 0.075, 0.0175],  # Right front
        ]

        for i, (wheel, local_pos) in enumerate(zip(self.wheel_entities, wheel_local_positions)):
            # Transform local wheel position to world coordinates
            cos_theta = np.cos(state.theta)
            sin_theta = np.sin(state.theta)

            world_x = state.x + cos_theta * local_pos[0] - sin_theta * local_pos[1]
            world_y = state.y + sin_theta * local_pos[0] + cos_theta * local_pos[1]
            world_z = local_pos[2]

            wheel.set_pos((world_x, world_y, world_z))
            wheel.set_quat(robot_quat)

        # Update camera with pan angle
        if self.camera_entity:
            camera_config = self.config["robot"]["sensors"]["camera"]
            camera_local_pos = camera_config["position"]
            camera_pan = np.radians(state.camera_pan_angle)

            # Transform camera position
            cos_theta = np.cos(state.theta)
            sin_theta = np.sin(state.theta)

            cam_x = state.x + cos_theta * camera_local_pos[0] - sin_theta * camera_local_pos[1]
            cam_y = state.y + sin_theta * camera_local_pos[0] + cos_theta * camera_local_pos[1]
            cam_z = camera_local_pos[2] + 0.05

            camera_quat = gs.quat_from_euler([0, 0, state.theta + camera_pan])

            self.camera_entity.set_pos((cam_x, cam_y, cam_z))
            self.camera_entity.set_quat(camera_quat)

        # Update trail
        self._update_trail(state.x, state.y)

        # Update sensor visualization
        self._update_sensor_visualization(state)

        # Step the simulation
        try:
            self.scene.step()
        except Exception as e:
            import logging
            logging.getLogger(__name__).debug("Genesis scene step error: %s", e)

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
        if not self.scene:
            return

        # Visualize ultrasonic sensor reading with a marker
        ray_end_x = state.x + state.ultrasonic_distance * np.cos(state.theta)
        ray_end_y = state.y + state.ultrasonic_distance * np.sin(state.theta)

        # Create or update ultrasonic marker
        if hasattr(self, "ultrasonic_marker") and self.ultrasonic_marker:
            self.ultrasonic_marker.set_pos((ray_end_x, ray_end_y, 0.05))
        else:
            self.ultrasonic_marker = self.scene.add_entity(
                gs.morphs.Sphere(radius=0.02, pos=(ray_end_x, ray_end_y, 0.05)),
                material=gs.materials.Rigid(color=(1.0, 0.0, 1.0)),  # Magenta
            )

    def add_trajectory_point(self, x: float, y: float, color: List[float] = [1.0, 1.0, 1.0]):
        """Add a trajectory point marker"""
        if not self.scene:
            return
        marker = self.scene.add_entity(
            gs.morphs.Sphere(radius=0.01, pos=(x, y, 0.02)),
            material=gs.materials.Rigid(color=color),
        )

    def add_waypoint(self, x: float, y: float, z: float = 0.1):
        """Add a waypoint marker"""
        if not self.scene:
            return
        waypoint = self.scene.add_entity(
            gs.morphs.Cylinder(radius=0.05, height=0.2, pos=(x, y, z)),
            material=gs.materials.Rigid(color=(1.0, 1.0, 0.0, 0.5)),  # Semi-transparent yellow
        )

    def clear_markers(self):
        """Clear all trajectory points and waypoints"""
        # This would need to be implemented to track and remove markers
        pass

    def start_update_loop(self):
        """Start the visualization update loop - simplified"""
        if not self.enabled:
            return

        # Use fallback visualizer if Genesis not available
        if self.using_fallback:
            self.fallback_viz.start_update_loop()
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
            transform=SE3(0, 0, 0.025).A,
        )
        robot_box.visual.face_colors = [100, 150, 200, 255]  # Blue
        self.robot_mesh = robot_box
        self.env.add(self.robot_mesh)

        # Add a direction indicator (arrow)
        arrow = trimesh.primitives.Cylinder(
            radius=0.01, height=0.08, transform=SE3(0.06, 0, 0.03) * SE3.Ry(np.pi / 2).A
        )
        arrow.visual.face_colors = [255, 0, 0, 255]  # Red arrow
        self.env.add(arrow)

        print("✓ Simple robot visualization created")

    def stop(self):
        """Stop the visualization"""
        print("Stopping visualization...")
        self._running = False

        # Use fallback visualizer if using matplotlib
        if self.using_fallback:
            if hasattr(self, "fallback_viz"):
                self.fallback_viz.close()
            return

        # Stop the update thread (only for Genesis mode)
        if hasattr(self, "_thread") and self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

        # Clean up Genesis environment
        if self.scene:
            try:
                if hasattr(self.scene, "close"):
                    self.scene.close()
                self.scene = None
                print("✓ Genesis environment cleaned up")
            except Exception as e:
                print(f"Note: Genesis cleanup: {e}")
                self.scene = None

    def screenshot(self, filename: str):
        """Take a screenshot of the visualization"""
        # This would need to be implemented in Swift
        pass

    def record_video(self, filename: str, duration: float):
        """Record a video of the simulation"""
        # This would need to be implemented in Swift
        pass


class GenesisSimulation:
    """Genesis-based simulation environment"""

    def __init__(self, config_path: str, viz_mode: str = "headless"):
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


# Keep SwiftSimulation as an alias for backward compatibility
SwiftSimulation = GenesisSimulation


def main():
    """Main function for testing Genesis visualization"""
    config_path = "../config/robot_config.yaml"

    try:
        # Create simulation
        print("Creating simulation...")
        sim = GenesisSimulation(config_path, viz_mode="headless")

        # Set some motor commands to make the robot move
        sim.set_motor_commands(100, 80)  # Slight turn

        # Run simulation
        sim.run_simulation(duration=10.0)

    except Exception as e:
        print(f"Error running simulation: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    print("ESP32 Robot Car Genesis Visualization")
    print("=" * 50)
    main()
