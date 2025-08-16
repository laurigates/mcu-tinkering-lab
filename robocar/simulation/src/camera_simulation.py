"""
Camera Simulation Module

This module provides realistic camera simulation for the ESP32-CAM robot,
generating synthetic camera feeds with OpenCV that match what the physical
camera would see in the simulated environment.
"""

import cv2
import numpy as np
import time
import threading
from typing import Optional, Tuple, Dict, Any
from pathlib import Path
import yaml
from dataclasses import dataclass
from spatialmath import SE3

from robot_model import RobotState


@dataclass
class CameraConfig:
    """Camera configuration parameters"""
    resolution: Tuple[int, int] = (640, 480)
    fov_degrees: float = 70.0
    fps: int = 30
    depth_range: Tuple[float, float] = (0.1, 10.0)  # min, max depth in meters
    noise_level: float = 0.02  # Gaussian noise standard deviation
    compression_quality: int = 85  # JPEG quality for ESP32-CAM simulation


class CameraSimulation:
    """
    Simulates the ESP32-CAM camera feed with realistic image generation
    """
    
    def __init__(self, config_path: str):
        self.config = self._load_config(config_path)
        
        # Extract camera parameters from both robot and simulation configs
        robot_camera_config = self.config['robot']['sensors']['camera']
        camera_sim_config = self.config['simulation'].get('camera', {})
        camera_params = {}
        
        # Use robot camera config as primary source, simulation config as override
        if 'resolution' in robot_camera_config:
            camera_params['resolution'] = tuple(robot_camera_config['resolution'])
        if 'resolution' in camera_sim_config:  # Simulation override
            camera_params['resolution'] = tuple(camera_sim_config['resolution'])
            
        if 'field_of_view' in robot_camera_config:
            camera_params['fov_degrees'] = robot_camera_config['field_of_view']
        if 'fov_degrees' in camera_sim_config:  # Simulation override
            camera_params['fov_degrees'] = camera_sim_config['fov_degrees']
            
        if 'fps' in robot_camera_config:
            camera_params['fps'] = robot_camera_config['fps']
        if 'fps' in camera_sim_config:  # Simulation override
            camera_params['fps'] = camera_sim_config['fps']
            
        # Simulation-specific parameters
        if 'noise_level' in camera_sim_config:
            camera_params['noise_level'] = camera_sim_config['noise_level']
        if 'compression_quality' in camera_sim_config:
            camera_params['compression_quality'] = camera_sim_config['compression_quality']
        
        self.camera_config = CameraConfig(**camera_params)
        
        # Camera state
        self.current_frame: Optional[np.ndarray] = None
        self.is_running = False
        self.frame_lock = threading.Lock()
        
        # Camera intrinsics (approximate ESP32-CAM parameters)
        self._setup_camera_intrinsics()
        
        # Synthetic environment setup
        self._setup_environment()
        
        # Threading
        self.capture_thread: Optional[threading.Thread] = None
        self.last_frame_time = 0
        
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load configuration from YAML file"""
        config_file = Path(config_path)
        if not config_file.exists():
            config_file = Path(__file__).parent.parent / "config" / "robot_config.yaml"
        
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    
    def _setup_camera_intrinsics(self):
        """Setup camera intrinsic parameters for ESP32-CAM simulation"""
        width, height = self.camera_config.resolution
        
        # Typical ESP32-CAM (OV2640) parameters
        focal_length_px = width / (2 * np.tan(np.radians(self.camera_config.fov_degrees / 2)))
        
        self.camera_matrix = np.array([
            [focal_length_px, 0, width / 2],
            [0, focal_length_px, height / 2],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Distortion coefficients (barrel distortion typical for wide-angle lens)
        self.distortion_coeffs = np.array([0.1, -0.2, 0, 0, 0], dtype=np.float32)
    
    def _setup_environment(self):
        """Setup synthetic environment for camera simulation"""
        # Create a simple room environment with obstacles
        self.environment_objects = self._create_environment_objects()
        
        # Floor pattern for visual reference
        self.floor_pattern = self._create_floor_pattern()
        
    def _create_environment_objects(self) -> Dict[str, Any]:
        """Create 3D environment objects for rendering"""
        objects = {}
        
        # Room boundaries (walls)
        room_size = 5.0  # 5m x 5m room
        wall_height = 2.5
        
        objects['walls'] = [
            {'type': 'plane', 'position': [room_size/2, 0, wall_height/2], 
             'size': [0.1, room_size, wall_height], 'color': [200, 200, 200]},  # Right wall
            {'type': 'plane', 'position': [-room_size/2, 0, wall_height/2], 
             'size': [0.1, room_size, wall_height], 'color': [200, 200, 200]},  # Left wall
            {'type': 'plane', 'position': [0, room_size/2, wall_height/2], 
             'size': [room_size, 0.1, wall_height], 'color': [200, 200, 200]},  # Front wall
            {'type': 'plane', 'position': [0, -room_size/2, wall_height/2], 
             'size': [room_size, 0.1, wall_height], 'color': [200, 200, 200]},  # Back wall
        ]
        
        # Obstacles
        objects['obstacles'] = [
            {'type': 'box', 'position': [1.0, 1.0, 0.25], 
             'size': [0.5, 0.5, 0.5], 'color': [100, 150, 200]},  # Blue box
            {'type': 'cylinder', 'position': [-1.0, -1.0, 0.5], 
             'radius': 0.3, 'height': 1.0, 'color': [200, 100, 100]},  # Red cylinder
            {'type': 'box', 'position': [2.0, -0.5, 0.15], 
             'size': [0.3, 0.8, 0.3], 'color': [100, 200, 100]},  # Green box
        ]
        
        return objects
    
    def _create_floor_pattern(self) -> np.ndarray:
        """Create a checkered floor pattern for visual reference"""
        pattern_size = 64
        checker_size = 8
        
        pattern = np.zeros((pattern_size, pattern_size), dtype=np.uint8)
        for i in range(0, pattern_size, checker_size):
            for j in range(0, pattern_size, checker_size):
                if (i // checker_size + j // checker_size) % 2 == 0:
                    pattern[i:i+checker_size, j:j+checker_size] = 255
                else:
                    pattern[i:i+checker_size, j:j+checker_size] = 200
        
        return cv2.cvtColor(pattern, cv2.COLOR_GRAY2BGR)
    
    def _render_synthetic_view(self, robot_state: RobotState) -> np.ndarray:
        """
        Render synthetic camera view based on robot position and camera angle
        """
        width, height = self.camera_config.resolution
        
        # Create blank image
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Calculate camera pose in world coordinates
        robot_transform = SE3(robot_state.x, robot_state.y, 0) * SE3.Rz(robot_state.theta)
        
        # Camera is mounted on top of robot with pan capability
        camera_config = self.config['robot']['sensors']['camera']
        camera_local_pos = camera_config['position']
        camera_height = camera_local_pos[2] + 0.1  # Camera height above ground
        
        camera_pan = np.radians(robot_state.camera_pan_angle)
        camera_transform = (robot_transform * 
                          SE3(camera_local_pos[0], camera_local_pos[1], camera_height) *
                          SE3.Rz(camera_pan))
        
        # Camera position and orientation
        cam_pos = camera_transform.t
        cam_forward = camera_transform.R @ np.array([1, 0, 0])  # Camera points forward
        cam_up = camera_transform.R @ np.array([0, 0, 1])       # Camera up vector
        
        # Render floor
        self._render_floor(img, cam_pos, cam_forward, cam_up)
        
        # Render environment objects
        self._render_objects(img, cam_pos, cam_forward, cam_up)
        
        # Add some basic lighting effects
        self._apply_lighting(img, cam_pos)
        
        # Add camera distortion to simulate ESP32-CAM lens characteristics
        img = self._apply_camera_distortion(img)
        
        # Add noise to simulate camera sensor characteristics
        img = self._add_sensor_noise(img)
        
        return img
    
    def _render_floor(self, img: np.ndarray, cam_pos: np.ndarray, 
                     cam_forward: np.ndarray, cam_up: np.ndarray):
        """Render checkered floor pattern"""
        height, width = img.shape[:2]
        
        # Simple floor rendering using perspective projection
        for y in range(height):
            for x in range(width):
                # Convert screen coordinates to world ray
                ray_dir = self._screen_to_world_ray(x, y, width, height, cam_forward, cam_up)
                
                # Intersect ray with floor plane (z = 0)
                if ray_dir[2] < -0.01:  # Ray pointing downward
                    t = -cam_pos[2] / ray_dir[2]  # Distance to floor intersection
                    if t > 0 and t < 10:  # Reasonable distance
                        floor_point = cam_pos + t * ray_dir
                        
                        # Sample floor pattern
                        fx, fy = floor_point[0], floor_point[1]
                        if abs(fx) < 5 and abs(fy) < 5:  # Within room bounds
                            # Convert to pattern coordinates
                            px = int((fx + 2.5) * 12) % 64
                            py = int((fy + 2.5) * 12) % 64
                            
                            if 0 <= px < 64 and 0 <= py < 64:
                                color = self.floor_pattern[py, px]
                                img[y, x] = color
    
    def _render_objects(self, img: np.ndarray, cam_pos: np.ndarray,
                       cam_forward: np.ndarray, cam_up: np.ndarray):
        """Render environment objects"""
        height, width = img.shape[:2]
        
        # Render walls
        for wall in self.environment_objects['walls']:
            self._render_box(img, wall, cam_pos, cam_forward, cam_up, width, height)
        
        # Render obstacles
        for obstacle in self.environment_objects['obstacles']:
            if obstacle['type'] == 'box':
                self._render_box(img, obstacle, cam_pos, cam_forward, cam_up, width, height)
            elif obstacle['type'] == 'cylinder':
                self._render_cylinder(img, obstacle, cam_pos, cam_forward, cam_up, width, height)
    
    def _render_box(self, img: np.ndarray, box: Dict, cam_pos: np.ndarray,
                   cam_forward: np.ndarray, cam_up: np.ndarray, width: int, height: int):
        """Render a box object"""
        box_pos = np.array(box['position'])
        box_size = np.array(box['size'])
        color = box['color']
        
        # Simple box rendering - just check if objects are in view
        # For production, this would use proper 3D rendering
        distance = np.linalg.norm(box_pos - cam_pos)
        
        if distance < 5.0:  # Only render nearby objects
            # Project to screen coordinates (simplified)
            screen_pos = self._world_to_screen(box_pos, cam_pos, cam_forward, cam_up, width, height)
            
            if screen_pos is not None:
                sx, sy = screen_pos
                
                # Render as a simple colored rectangle
                size_scale = max(1, int(50 / distance))  # Size based on distance
                
                x1 = max(0, int(sx - size_scale))
                x2 = min(width, int(sx + size_scale))
                y1 = max(0, int(sy - size_scale))
                y2 = min(height, int(sy + size_scale))
                
                if x2 > x1 and y2 > y1:
                    img[y1:y2, x1:x2] = color
    
    def _render_cylinder(self, img: np.ndarray, cylinder: Dict, cam_pos: np.ndarray,
                        cam_forward: np.ndarray, cam_up: np.ndarray, width: int, height: int):
        """Render a cylinder object"""
        # Similar to box rendering but with circular shape
        cyl_pos = np.array(cylinder['position'])
        radius = cylinder['radius']
        color = cylinder['color']
        
        distance = np.linalg.norm(cyl_pos - cam_pos)
        
        if distance < 5.0:
            screen_pos = self._world_to_screen(cyl_pos, cam_pos, cam_forward, cam_up, width, height)
            
            if screen_pos is not None:
                sx, sy = screen_pos
                screen_radius = max(1, int(50 * radius / distance))
                
                # Draw circle
                cv2.circle(img, (int(sx), int(sy)), screen_radius, color, -1)
    
    def _screen_to_world_ray(self, x: int, y: int, width: int, height: int,
                            cam_forward: np.ndarray, cam_up: np.ndarray) -> np.ndarray:
        """Convert screen coordinates to world ray direction"""
        # Normalize screen coordinates to [-1, 1]
        nx = (2.0 * x / width) - 1.0
        ny = 1.0 - (2.0 * y / height)
        
        # Camera right vector
        cam_right = np.cross(cam_forward, cam_up)
        cam_right = cam_right / np.linalg.norm(cam_right)
        
        # Proper up vector (orthogonal to forward and right)
        cam_up_corrected = np.cross(cam_right, cam_forward)
        
        # FOV calculations
        fov_rad = np.radians(self.camera_config.fov_degrees)
        tan_half_fov = np.tan(fov_rad / 2)
        
        # Ray direction
        ray_dir = (cam_forward + 
                  nx * tan_half_fov * cam_right + 
                  ny * tan_half_fov * cam_up_corrected)
        
        return ray_dir / np.linalg.norm(ray_dir)
    
    def _world_to_screen(self, world_pos: np.ndarray, cam_pos: np.ndarray,
                        cam_forward: np.ndarray, cam_up: np.ndarray, 
                        width: int, height: int) -> Optional[Tuple[int, int]]:
        """Project world coordinates to screen coordinates"""
        # Vector from camera to object
        obj_vec = world_pos - cam_pos
        
        # Check if object is in front of camera
        if np.dot(obj_vec, cam_forward) <= 0:
            return None
        
        # Camera coordinate system
        cam_right = np.cross(cam_forward, cam_up)
        cam_right = cam_right / np.linalg.norm(cam_right)
        cam_up_corrected = np.cross(cam_right, cam_forward)
        
        # Project to camera coordinates
        x_cam = np.dot(obj_vec, cam_right)
        y_cam = np.dot(obj_vec, cam_up_corrected)
        z_cam = np.dot(obj_vec, cam_forward)
        
        # Perspective projection
        fov_rad = np.radians(self.camera_config.fov_degrees)
        tan_half_fov = np.tan(fov_rad / 2)
        
        if z_cam > 0:
            x_screen = x_cam / (z_cam * tan_half_fov)
            y_screen = y_cam / (z_cam * tan_half_fov)
            
            # Convert to pixel coordinates
            px = int((x_screen + 1) * width / 2)
            py = int((1 - y_screen) * height / 2)
            
            if 0 <= px < width and 0 <= py < height:
                return (px, py)
        
        return None
    
    def _apply_lighting(self, img: np.ndarray, cam_pos: np.ndarray):
        """Apply basic lighting effects"""
        # Simple ambient lighting with distance-based attenuation
        height, width = img.shape[:2]
        
        # Add some brightness variation based on viewing angle
        brightness_factor = 0.8 + 0.2 * np.random.random()
        img = cv2.convertScaleAbs(img, alpha=brightness_factor, beta=10)
    
    def _apply_camera_distortion(self, img: np.ndarray) -> np.ndarray:
        """Apply camera lens distortion to simulate ESP32-CAM characteristics"""
        height, width = img.shape[:2]
        
        # Apply barrel distortion using OpenCV
        map1, map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.distortion_coeffs, None, 
            self.camera_matrix, (width, height), cv2.CV_16SC2
        )
        
        # Reverse the distortion correction to add distortion
        distorted = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        
        return distorted
    
    def _add_sensor_noise(self, img: np.ndarray) -> np.ndarray:
        """Add sensor noise to simulate camera characteristics"""
        if self.camera_config.noise_level > 0:
            noise = np.random.normal(0, self.camera_config.noise_level * 255, img.shape)
            img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        
        return img
    
    def start_capture(self, robot_state_callback):
        """Start camera capture thread"""
        if self.is_running:
            return
        
        self.is_running = True
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            args=(robot_state_callback,),
            daemon=True
        )
        self.capture_thread.start()
        print(f"Camera simulation started at {self.camera_config.fps} FPS")
    
    def stop_capture(self):
        """Stop camera capture"""
        self.is_running = False
        if self.capture_thread:
            self.capture_thread.join()
        print("Camera simulation stopped")
    
    def _capture_loop(self, robot_state_callback):
        """Main capture loop"""
        frame_interval = 1.0 / self.camera_config.fps
        
        while self.is_running:
            current_time = time.time()
            
            if current_time - self.last_frame_time >= frame_interval:
                # Get current robot state
                robot_state = robot_state_callback()
                
                if robot_state is not None:
                    # Generate synthetic camera frame
                    frame = self._render_synthetic_view(robot_state)
                    
                    # Update current frame
                    with self.frame_lock:
                        self.current_frame = frame.copy()
                        robot_state.camera_frame = frame
                    
                    self.last_frame_time = current_time
            
            # Sleep for a short time to prevent excessive CPU usage
            time.sleep(0.001)
    
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the latest camera frame"""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None
    
    def get_compressed_frame(self) -> Optional[bytes]:
        """Get JPEG-compressed frame (simulating ESP32-CAM output)"""
        frame = self.get_latest_frame()
        if frame is not None:
            # Compress as JPEG with quality setting
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.camera_config.compression_quality]
            result, compressed = cv2.imencode('.jpg', frame, encode_param)
            if result:
                return compressed.tobytes()
        return None
    
    def save_frame(self, filename: str) -> bool:
        """Save current frame to file"""
        frame = self.get_latest_frame()
        if frame is not None:
            return cv2.imwrite(filename, frame)
        return False


def main():
    """Test the camera simulation"""
    from robot_model import DifferentialDriveRobot
    
    # Test configuration
    config_path = "../config/robot_config.yaml"
    
    # Create robot and camera simulation
    robot = DifferentialDriveRobot(config_path)
    camera = CameraSimulation(config_path)
    
    # Start camera simulation
    camera.start_capture(lambda: robot.get_state())
    
    print("Camera simulation test started")
    print("Press 'q' to quit, 's' to save frame")
    
    try:
        while True:
            frame = camera.get_latest_frame()
            if frame is not None:
                # Display frame
                cv2.imshow('Camera Simulation', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"camera_frame_{int(time.time())}.jpg"
                    if camera.save_frame(filename):
                        print(f"Frame saved as {filename}")
            
            # Update robot (simple movement for testing)
            robot.set_motor_commands(50, 45)  # Slight turn
            robot.update()
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nStopping camera simulation...")
    
    finally:
        camera.stop_capture()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()