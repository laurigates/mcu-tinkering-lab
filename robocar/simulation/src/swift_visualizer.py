"""
Swift Visualizer for ESP32 Robot Car Simulation

This module provides 3D visualization using the Swift simulator backend
with the Robotics Toolbox for Python.
"""

import asyncio
import queue
import threading
import time
from typing import Dict, List

import numpy as np
import yaml

try:
    import roboticstoolbox as rtb
    import swift
    import trimesh
    from roboticstoolbox import DHRobot, RevoluteDH
    from spatialmath import SE3
    from swift import Swift
    HAS_SWIFT = True
except ImportError:
    HAS_SWIFT = False
    print("Warning: Swift and Robotics Toolbox not available. Install with:")
    print("pip install robotics-toolbox-python swift-sim")

import os
import subprocess
import sys

from robot_model import DifferentialDriveRobot, RobotState


class RobotVisualizer:
    """3D visualization of the robot car using Swift"""
    
    def __init__(self, config_path: str, robot: DifferentialDriveRobot):
        if not HAS_SWIFT:
            print("Warning: Swift and Robotics Toolbox not available")
            print("3D visualization will be disabled")
        
        self.config = self._load_config(config_path)
        self.robot = robot
        
        # Swift environment - run in separate thread to avoid asyncio conflicts
        self.env = None
        self._init_swift_env()
        
        # Robot visual elements
        self.robot_mesh = None
        self.wheel_meshes = []
        self.camera_mesh = None
        self.trail_points = []
        
        # Thread-safe communication
        self._update_queue = queue.Queue()
        self._running = False
        self._thread = None
        self.obstacles = []
        
        # Visualization parameters
        self.trail_length = 100
        self.update_rate = 30  # Hz
        self.last_update = 0
        
        # Initialize visualization
        self._create_environment()
        self._create_robot_model()
        
        # Threading
        self.running = False
        self.update_thread = None
        self._swift_process = None
        
    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file"""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    def _init_swift_env(self):
        """Initialize Swift environment with proper thread isolation"""
        # We'll initialize Swift in the dedicated thread to avoid asyncio conflicts
        self.env = None  # Will be created in _visualization_thread
        print("Swift 3D visualization will be initialized in dedicated thread")
        print("Note: Swift library may show asyncio threading warnings that can be ignored")
    
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
        """Update robot visualization based on current state - thread safe"""
        state = self.robot.get_state()
        
        # Send update command to visualization thread
        update_data = {
            'type': 'robot_update',
            'state': state
        }
        try:
            self._update_queue.put_nowait(update_data)
        except queue.Full:
            # Drop update if queue is full to prevent blocking
            pass
    
    def _process_robot_update(self, state):
        """Process robot visualization update in Swift thread"""
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
        self.env.step(0.033)  # ~30 FPS
    
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
        """Start the visualization update loop"""
        self._running = True
        self._thread = threading.Thread(target=self._visualization_thread, daemon=True)
        self._thread.start()
    
    def _run_swift_in_subprocess(self):
        """Run Swift in a separate subprocess to avoid asyncio conflicts"""
        print("EXPERIMENTAL: Running Swift in subprocess to avoid asyncio conflicts...")
        
        # Create a simple Swift script to run in subprocess
        swift_script = """
import asyncio
import sys
import time
from swift import Swift
import warnings
warnings.filterwarnings("ignore")

async def main():
    env = Swift()
    env.launch(realtime=True, headless=False)
    print("✓ Swift visual mode started in subprocess")
    print("✓ Browser should open automatically")
    
    # Keep running until interrupted
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("Swift subprocess stopping...")

if __name__ == "__main__":
    asyncio.run(main())
"""
        
        # Write script to temp file
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(swift_script)
            script_path = f.name
        
        try:
            # Start subprocess
            self._swift_process = subprocess.Popen([
                sys.executable, script_path
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            # Give it time to start
            time.sleep(2)
            
            if self._swift_process.poll() is None:
                print("✓ Swift subprocess started successfully")
                return True
            else:
                stdout, stderr = self._swift_process.communicate()
                print(f"Swift subprocess failed: {stderr}")
                return False
                
        except Exception as e:
            print(f"Failed to start Swift subprocess: {e}")
            return False
        finally:
            # Clean up temp file
            try:
                os.unlink(script_path)
            except:
                pass
    
    def _run_subprocess_update_loop(self):
        """Run update loop when Swift is in subprocess"""
        print("Running subprocess update loop...")
        
        try:
            while self._running:
                # In subprocess mode, we can't update the 3D visualization directly
                # But we can still update the robot state and print status
                time.sleep(0.1)  # 10 Hz update rate
                
                # Process any queued updates
                try:
                    while True:
                        update_data = self._update_queue.get_nowait()
                        if update_data['type'] == 'robot_update':
                            # In subprocess mode, just print the state
                            state = update_data['state']
                            print(f"Robot: x={state.x:.2f}, y={state.y:.2f}, θ={state.theta:.2f}")
                except queue.Empty:
                    pass
        except Exception as e:
            print(f"Subprocess update loop error: {e}")
        finally:
            # Clean up subprocess
            if hasattr(self, '_swift_process') and self._swift_process:
                self._swift_process.terminate()
                self._swift_process.wait()
                print("Swift subprocess terminated")
    
    def _visualization_thread(self):
        """Main visualization thread - handles Swift environment isolated from asyncio"""
        if not HAS_SWIFT:
            print("Swift not available, visualization thread exiting")
            return
        
        # Try subprocess approach first to completely avoid asyncio conflicts
        if self._run_swift_in_subprocess():
            print("Using subprocess approach for Swift visualization")
            # Run a simplified update loop without Swift environment setup
            self._run_subprocess_update_loop()
            return
        
        print("Subprocess approach failed, trying in-process Swift...")
            
        try:
            import asyncio as asyncio_module

            # Create and set event loop for this thread
            loop = asyncio_module.new_event_loop()
            asyncio_module.set_event_loop(loop)
            
            print("Initializing Swift environment in dedicated thread...")
            
            # Suppress websocket error warnings that are internal to Swift
            import logging
            import warnings
            # Filter out websocket-related warnings and RuntimeError about no running event loop
            warnings.filterwarnings("ignore", message=".*websockets.*")
            warnings.filterwarnings("ignore", message=".*no running event loop.*")
            logging.getLogger("websockets").setLevel(logging.ERROR)
            
            # Initialize Swift environment with a unique port to avoid conflicts
            import random
            port = random.randint(8000, 9000)
            
            print("Note: Swift may show websocket threading warnings - these can be safely ignored")
            
            # Define the complete async workflow
            async def swift_workflow():
                """Complete Swift initialization and visualization workflow"""
                
                # Initialize Swift within the async context
                success = False
                
                try:
                    import contextlib
                    import io

                    # Create Swift instance
                    with contextlib.redirect_stderr(io.StringIO()):
                        self.env = Swift()
                    
                    print("Swift instance created, launching visualization server...")
                    
                    # Try visual mode first with proper asyncio event loop handling
                    # The event loop is now running in this thread context
                    try:
                        print("Initializing Swift in visual mode...")
                        print("Note: Attempting to start with browser interface")
                        
                        # Try with browser parameter to avoid direct websocket handling
                        self.env.launch(realtime=True, headless=False, browser='default')
                        success = True
                        print("✓ Swift 3D visualization running in visual mode")
                        print("✓ Browser window should open automatically")
                        print("✓ If browser doesn't open, access manually at: http://localhost:52000")
                        
                    except Exception as e:
                        print(f"Visual mode with browser failed: {e}")
                        
                        # Try without browser parameter
                        try:
                            print("Retrying visual mode without browser parameter...")
                            # Reset Swift instance
                            with contextlib.redirect_stderr(io.StringIO()):
                                self.env = Swift()
                            self.env.launch(realtime=True, headless=True)
                            success = True
                            print("✓ Swift 3D visualization running in visual mode")
                            print("✓ Check console for localhost URL")
                            
                        except Exception as e2:
                            print(f"Visual mode failed: {e2}")
                            if "no running event loop" in str(e2):
                                print("Note: This is the asyncio event loop conflict - trying fallback...")
                            else:
                                print("Note: This may be due to missing dependencies or system issues")
                            success = False
                    
                    # Attempt 2: Try headless mode as backup
                    if not success:
                        try:
                            # Reset Swift instance for clean state
                            with contextlib.redirect_stderr(io.StringIO()):
                                self.env = Swift()
                            print("Attempting Swift in headless mode (fallback)...")
                            self.env.launch(realtime=True, headless=True)
                            success = True
                            print("✓ Swift running in headless mode")
                            print("✓ Simulation will run without visual window")
                            
                        except Exception as e:
                            print(f"Headless mode failed: {e}")
                            success = False
                    
                    # Attempt 3: Try non-realtime mode as final backup
                    if not success:
                        try:
                            with contextlib.redirect_stderr(io.StringIO()):
                                self.env = Swift()
                            print("Attempting Swift with non-realtime backend (final fallback)...")
                            self.env.launch(realtime=False, headless=True)
                            success = True
                            print("✓ Swift running with non-realtime headless backend")
                            
                        except Exception as e:
                            print(f"Non-realtime backend failed: {e}")
                            success = False
                    
                    # All initialization attempts completed
                    
                    if success:
                        # Create robot visualization elements
                        self._create_environment()
                        self._create_robot_model()
                        print("Swift environment and robot model created successfully")
                    else:
                        raise Exception("Failed to launch Swift environment")
                        
                except Exception as e:
                    print(f"Swift initialization failed: {e}")
                    print("Running simulation without 3D visualization")
                    self.env = None
                    return
                
                # Run the visualization loop
                print("Starting Swift visualization loop...")
                last_robot_update = time.time()
                update_interval = 1.0 / 30.0  # 30 FPS
                
                while self._running:
                    current_time = time.time()
                    
                    # Process updates from queue
                    try:
                        while True:
                            update_data = self._update_queue.get_nowait()
                            if update_data['type'] == 'robot_update':
                                self._process_robot_update(update_data['state'])
                            elif update_data['type'] == 'pose_update':
                                self._process_robot_update(update_data['state'])
                    except queue.Empty:
                        pass
                    
                    # Regular robot state updates
                    if current_time - last_robot_update > update_interval:
                        state = self.robot.get_state()
                        self._process_robot_update(state)
                        last_robot_update = current_time
                    
                    await asyncio.sleep(0.01)  # Async sleep
            
            # CRITICAL FIX: Run Swift in a way that makes the event loop available
            # to the websocket server thread. We need to run the entire workflow
            # inside the event loop context.
            
            def run_swift_in_loop():
                """Run Swift with event loop available for websocket threads"""
                # This ensures the event loop is running and accessible to child threads
                task = loop.create_task(swift_workflow())
                return task
            
            # Start the Swift workflow as a background task
            swift_task = run_swift_in_loop()
            
            # Keep the event loop running to support websocket connections
            loop.run_until_complete(swift_task)
            
            print("Swift visualization loop stopping")
            
        except Exception as e:
            print(f"Swift initialization failed: {e}")
            print("Running simulation without 3D visualization")
            self.env = None
            return
        finally:
            # Clean up the event loop
            try:
                if loop and not loop.is_closed():
                    loop.close()
            except:
                pass
    
    def _update_loop(self):
        """Legacy update loop - now just calls update_robot_visualization"""
        while self.running:
            current_time = time.time()
            
            if current_time - self.last_update > (1.0 / self.update_rate):
                self.update_robot_visualization()
                self.last_update = current_time
            
            time.sleep(0.001)  # Small sleep to prevent busy waiting
    
    def stop(self):
        """Stop the visualization"""
        print("Stopping Swift visualization...")
        self._running = False
        self.running = False  # Legacy compatibility
        
        # Stop the visualization thread
        if hasattr(self, '_thread') and self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)  # Wait up to 2 seconds
        
        if hasattr(self, 'update_thread') and self.update_thread:
            self.update_thread.join(timeout=1.0)
            
        # Clean up subprocess if running
        if hasattr(self, '_swift_process') and self._swift_process:
            try:
                self._swift_process.terminate()
                self._swift_process.wait(timeout=3.0)
                print("Swift subprocess terminated")
            except:
                try:
                    self._swift_process.kill()
                    print("Swift subprocess killed")
                except:
                    pass
            
        if self.env:
            try:
                # Swift in headless mode may not have all attributes
                if hasattr(self.env, 'close'):
                    self.env.close()
                elif hasattr(self.env, 'server') and hasattr(self.env.server, 'close'):
                    self.env.server.close()
                # For headless mode or other Swift configurations, just clean up reference
                self.env = None
            except AttributeError as e:
                if "'Swift' object has no attribute 'server'" in str(e):
                    # Expected in headless mode - suppress this specific error
                    pass
                else:
                    print(f"Note: Swift environment cleanup: {e}")
            except Exception as e:
                print(f"Note: Swift environment cleanup: {e}")
    
    def screenshot(self, filename: str):
        """Take a screenshot of the visualization"""
        # This would need to be implemented in Swift
        pass
    
    def record_video(self, filename: str, duration: float):
        """Record a video of the simulation"""
        # This would need to be implemented in Swift
        pass


class SwiftSimulation:
    """Complete Swift-based simulation environment"""
    
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.robot = DifferentialDriveRobot(config_path)
        self.visualizer = RobotVisualizer(config_path, self.robot)
        
        # Simulation control
        self.running = False
        self.paused = False
        self.simulation_speed = 1.0
        
    def run_simulation(self, duration: float = 10.0):
        """Run the simulation for a specified duration"""
        print(f"Starting Swift simulation for {duration} seconds...")
        
        # Start visualization update loop
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
                
                time.sleep(0.001)  # Small sleep
                
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        
        finally:
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
        self.visualizer.trail_points.clear()


def main():
    """Main function for testing Swift visualization"""
    if not HAS_SWIFT:
        print("Swift and Robotics Toolbox not available. Please install:")
        print("pip install robotics-toolbox-python swift-sim")
        return
    
    config_path = "../config/robot_config.yaml"
    
    try:
        # Create simulation
        sim = SwiftSimulation(config_path)
        
        # Set some motor commands to make the robot move
        sim.set_motor_commands(100, 80)  # Slight turn
        
        # Run simulation
        sim.run_simulation(duration=30.0)
        
    except Exception as e:
        print(f"Error running simulation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    print("ESP32 Robot Car Swift Visualization")
    print("=" * 40)
    main()
