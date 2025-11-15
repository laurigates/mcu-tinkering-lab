"""
ESP32 Robot Car Model with Differential Drive Kinematics

This module implements the robot model for the ESP32-based robot car with
differential drive kinematics, motor dynamics, and sensor simulation.
"""

import numpy as np
from typing import Tuple, Dict, List, Optional
from dataclasses import dataclass
import time
import yaml
from pathlib import Path
import pymunk

from error_handling import get_error_handler, resilient_operation, ErrorSeverity, ErrorEvent
from wifi_simulation import WiFiManagerSimulation
from ota_simulation import OTASimulation


@dataclass
class RobotState:
    """Robot state representation"""

    # Position and orientation
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    theta: float = 0.0  # radians

    # Linear and angular velocities
    v: float = 0.0  # m/s
    omega: float = 0.0  # rad/s

    # Motor states
    motor_left_pwm: float = 0.0  # PWM value (-255 to 255)
    motor_right_pwm: float = 0.0  # PWM value (-255 to 255)
    motor_left_rpm: float = 0.0  # RPM
    motor_right_rpm: float = 0.0  # RPM

    # Sensor readings
    camera_frame: Optional[np.ndarray] = None
    ultrasonic_distance: float = 0.0  # meters
    imu_accel: np.ndarray = None  # [ax, ay, az] m/s²
    imu_gyro: np.ndarray = None  # [gx, gy, gz] rad/s

    # Servo positions
    camera_pan_angle: float = 0.0  # degrees

    def __post_init__(self):
        if self.imu_accel is None:
            self.imu_accel = np.zeros(3)
        if self.imu_gyro is None:
            self.imu_gyro = np.zeros(3)


class DCMotor:
    """DC Motor model with electrical and mechanical dynamics"""

    def __init__(self, config: Dict):
        self.resistance = config["resistance"]  # Ohms
        self.inductance = config["inductance"]  # H
        self.back_emf_constant = config["back_emf_constant"]  # V⋅s/rad
        # Torque constant derived from stall specifications for consistency
        self.torque_constant = config["stall_torque"] / config["stall_current"]  # N⋅m/A
        self.max_rpm = config["max_rpm"]
        self.stall_torque = config["stall_torque"]  # N⋅m
        self.no_load_current = config["no_load_current"]  # A
        self.stall_current = config["stall_current"]  # A

        # State variables
        self.current = 0.0  # A
        self.angular_velocity = 0.0  # rad/s
        self.torque = 0.0  # N⋅m

        # Friction parameters (realistic for small DC motor)
        self.friction_static = 0.001  # N⋅m (reduced for small motor)
        self.friction_kinetic = 0.0005  # N⋅m (reduced for small motor)
        self.friction_viscous = 0.0001  # N⋅m⋅s/rad (reduced for small motor)

    def update(self, voltage: float, load_torque: float, dt: float) -> Tuple[float, float]:
        """
        Update motor state based on applied voltage and load torque

        Args:
            voltage: Applied voltage (V)
            load_torque: External load torque (N⋅m)
            dt: Time step (s)

        Returns:
            Tuple of (angular_velocity, torque)
        """
        # Back EMF
        back_emf = self.back_emf_constant * self.angular_velocity

        # Current calculation using more stable integration
        # di/dt = (V - back_emf - R*i) / L
        # Use implicit integration for better stability
        time_constant = self.inductance / self.resistance
        target_current = (voltage - back_emf) / self.resistance
        self.current += (
            (target_current - self.current) * (dt / time_constant) * 0.1
        )  # Damping factor for stability

        # Limit current to stall current
        self.current = np.clip(self.current, -self.stall_current, self.stall_current)

        # Motor torque
        motor_torque = self.torque_constant * self.current

        # Friction torque
        if abs(self.angular_velocity) > 0.01:  # Moving (reduced threshold)
            friction_torque = np.sign(self.angular_velocity) * self.friction_kinetic
        else:  # Static friction - only applies if motor torque exceeds static friction
            static_threshold = abs(motor_torque - load_torque)
            if static_threshold > self.friction_static:
                friction_torque = np.sign(motor_torque - load_torque) * self.friction_static
            else:
                friction_torque = motor_torque - load_torque  # Static friction prevents motion

        # Viscous friction
        viscous_torque = self.friction_viscous * self.angular_velocity

        # Net torque
        net_torque = motor_torque - load_torque - friction_torque - viscous_torque

        # Angular acceleration with proper motor inertia
        # J * dw/dt = net_torque, where J is motor rotor inertia
        motor_inertia = 0.001  # kg⋅m² - realistic value for small DC motor rotor
        angular_acceleration = net_torque / motor_inertia
        self.angular_velocity += angular_acceleration * dt

        # Limit to maximum RPM
        max_angular_velocity = self.max_rpm * 2 * np.pi / 60  # rad/s
        self.angular_velocity = np.clip(
            self.angular_velocity, -max_angular_velocity, max_angular_velocity
        )

        self.torque = motor_torque

        return self.angular_velocity, self.torque


class PhysicsEngine:
    """Physics engine wrapper for pymunk"""

    def __init__(self, config: Dict):
        self.space = pymunk.Space()
        self.space.gravity = (0, -9.81)  # Gravity in m/s²

        # Environment setup
        self.environment_config = config["simulation"]["environment"]
        self.robots = {}
        self.obstacles = []

        # Create environment boundaries and obstacles
        self._setup_environment()

    def _setup_environment(self):
        """Set up environment boundaries and obstacles"""
        env_size = self.environment_config["size"]
        width, height = env_size[0], env_size[1]

        # Create boundaries
        walls = [
            pymunk.Segment(self.space.static_body, (0, 0), (width, 0), 0.1),  # Bottom
            pymunk.Segment(self.space.static_body, (0, 0), (0, height), 0.1),  # Left
            pymunk.Segment(self.space.static_body, (width, 0), (width, height), 0.1),  # Right
            pymunk.Segment(self.space.static_body, (0, height), (width, height), 0.1),  # Top
        ]

        for wall in walls:
            wall.friction = 0.7
            wall.collision_type = 1  # Wall collision type
            self.space.add(wall)

        # Add obstacles
        for obstacle in self.environment_config.get("obstacles", []):
            self._create_obstacle(obstacle)

    def _create_obstacle(self, obstacle_config: Dict):
        """Create a physics obstacle from configuration"""
        if obstacle_config["type"] == "box":
            pos = obstacle_config["position"]
            size = obstacle_config["size"]

            # Create box body
            mass = 1000  # Static obstacle (high mass)
            moment = pymunk.moment_for_box(mass, size)
            body = pymunk.Body(mass, moment, body_type=pymunk.Body.STATIC)
            body.position = pos

            # Create box shape
            shape = pymunk.Poly.create_box(body, size)
            shape.friction = 0.7
            shape.collision_type = 2  # Obstacle collision type

            self.space.add(body, shape)
            self.obstacles.append({"body": body, "shape": shape, "config": obstacle_config})

        elif obstacle_config["type"] == "cylinder":
            pos = obstacle_config["position"]
            radius = obstacle_config["radius"]

            # Create cylinder body
            mass = 1000  # Static obstacle
            moment = pymunk.moment_for_circle(mass, 0, radius)
            body = pymunk.Body(mass, moment, body_type=pymunk.Body.STATIC)
            body.position = pos

            # Create circle shape
            shape = pymunk.Circle(body, radius)
            shape.friction = 0.7
            shape.collision_type = 2  # Obstacle collision type

            self.space.add(body, shape)
            self.obstacles.append({"body": body, "shape": shape, "config": obstacle_config})

    def add_robot(
        self,
        robot_id: str,
        mass: float,
        dimensions: Dict,
        position: Tuple[float, float] = (0.5, 0.5),
    ):
        """Add a robot to the physics simulation"""
        # Robot dimensions
        length = dimensions["length"]
        width = dimensions["width"]

        # Create robot body
        moment = pymunk.moment_for_box(mass, (length, width))
        body = pymunk.Body(mass, moment)
        body.position = position

        # Create robot shape (rectangle)
        shape = pymunk.Poly.create_box(body, (length, width))
        shape.friction = 0.3
        shape.collision_type = 3  # Robot collision type

        self.space.add(body, shape)
        self.robots[robot_id] = {
            "body": body,
            "shape": shape,
            "mass": mass,
            "dimensions": dimensions,
        }

        return body, shape

    def update_robot_forces(
        self, robot_id: str, force_left: float, force_right: float, track_width: float
    ):
        """Apply differential drive forces to robot"""
        if robot_id not in self.robots:
            return

        robot = self.robots[robot_id]
        body = robot["body"]

        # Convert wheel forces to body force and torque
        # Force is applied at wheel contact points
        total_force = force_left + force_right
        torque = (force_right - force_left) * track_width / 2

        # Apply force in robot's forward direction
        angle = body.angle
        force_vector = (total_force * np.cos(angle), total_force * np.sin(angle))

        body.force = force_vector
        body.torque = torque

    def get_robot_state(self, robot_id: str) -> Dict:
        """Get robot physics state"""
        if robot_id not in self.robots:
            return {}

        robot = self.robots[robot_id]
        body = robot["body"]

        return {
            "position": (body.position.x, body.position.y),
            "angle": body.angle,
            "velocity": (body.velocity.x, body.velocity.y),
            "angular_velocity": body.angular_velocity,
        }

    def raycast_sensor(
        self, start_pos: Tuple[float, float], direction: float, max_range: float
    ) -> float:
        """Perform raycast for ultrasonic sensor simulation"""
        end_pos = (
            start_pos[0] + max_range * np.cos(direction),
            start_pos[1] + max_range * np.sin(direction),
        )

        # Perform segment query (raycast)
        query_info = self.space.segment_query_first(start_pos, end_pos, 0, pymunk.ShapeFilter())

        if query_info.shape:
            # Calculate distance to hit point
            hit_point = query_info.point
            distance = np.sqrt(
                (hit_point.x - start_pos[0]) ** 2 + (hit_point.y - start_pos[1]) ** 2
            )
            return max(0.02, distance)  # Minimum 2cm

        return max_range  # No collision detected

    def step(self, dt: float):
        """Step the physics simulation"""
        self.space.step(dt)


class DifferentialDriveRobot:
    """Differential drive robot model with realistic motor dynamics"""

    def __init__(self, config_path: str):
        """Initialize robot model from configuration file"""
        # Initialize error handling
        self.error_handler = get_error_handler()
        self.error_handler.register_component("robot_model")
        self._register_recovery_strategies()

        self.config = self._load_config(config_path)
        self.state = RobotState()

        # Robot physical parameters
        robot_config = self.config["robot"]
        self.wheelbase = robot_config["dimensions"]["wheelbase"]
        self.track_width = robot_config["dimensions"]["track_width"]
        self.wheel_radius = robot_config["dimensions"]["wheel_radius"]
        self.mass = robot_config["mass"]
        self.inertia = robot_config["inertia"]

        # Initialize motors
        self.motor_left = DCMotor(robot_config["motors"]["left"])
        self.motor_right = DCMotor(robot_config["motors"]["right"])

        # Initialize motor controllers with PID if configured
        self.use_motor_controllers = (
            self.config["simulation"].get("motor_control", {}).get("enabled", False)
        )
        self.motor_left_controller = None
        self.motor_right_controller = None

        if self.use_motor_controllers:
            from motor_controller import MotorController

            control_config = self.config["simulation"]["motor_control"]

            self.motor_left_controller = MotorController(
                robot_config["motors"]["left"], control_config
            )
            self.motor_right_controller = MotorController(
                robot_config["motors"]["right"], control_config
            )

        # Simulation parameters
        self.dt = self.config["simulation"]["timestep"]

        # Noise parameters for sensors
        self.imu_noise = robot_config["sensors"]["imu"]["noise"]

        # Initialize random number generator for sensor noise
        self.rng = np.random.RandomState(42)

        # Initialize physics engine if configured
        self.physics_engine = None
        self.robot_body = None
        self.robot_shape = None
        self.use_physics = self.config["simulation"].get("physics_engine") == "pymunk"

        if self.use_physics:
            self.physics_engine = PhysicsEngine(self.config)
            self.robot_body, self.robot_shape = self.physics_engine.add_robot(
                "main", self.mass, robot_config["dimensions"]
            )

        # Initialize camera simulation if configured
        self.camera_simulation = None
        self.use_camera_simulation = (
            self.config["simulation"].get("camera", {}).get("enabled", False)
        )

        if self.use_camera_simulation:
            from camera_simulation import CameraSimulation

            self.camera_simulation = CameraSimulation(config_path)
            # Start camera simulation after robot initialization
            self.camera_simulation.start_capture(lambda: self.state)

        # Initialize WiFi simulation
        self.wifi_simulation = None
        self.use_wifi_simulation = self.config["simulation"].get("wifi", {}).get("enabled", True)

        if self.use_wifi_simulation:
            self.wifi_simulation = WiFiManagerSimulation(config_path)
            self.wifi_simulation.init()
            print("Robot: WiFi simulation initialized")

        # Initialize OTA simulation
        self.ota_simulation = None
        self.use_ota_simulation = self.config["simulation"].get("ota", {}).get("enabled", True)

        if self.use_ota_simulation:
            self.ota_simulation = OTASimulation(config_path)
            if self.wifi_simulation:
                self.ota_simulation.set_wifi_manager(self.wifi_simulation)
            print("Robot: OTA simulation initialized")

    def _register_recovery_strategies(self):
        """Register recovery strategies for robot model errors"""

        def motor_recovery(error: ErrorEvent) -> bool:
            """Recovery strategy for motor-related errors"""
            try:
                # Reset motor states to safe defaults
                self.state.motor_left_pwm = 0.0
                self.state.motor_right_pwm = 0.0

                # Reset motor internal states if they exist
                if hasattr(self, "motor_left"):
                    self.motor_left.current = 0.0
                    self.motor_left.angular_velocity = 0.0
                    self.motor_left.torque = 0.0

                if hasattr(self, "motor_right"):
                    self.motor_right.current = 0.0
                    self.motor_right.angular_velocity = 0.0
                    self.motor_right.torque = 0.0

                return True
            except Exception:
                return False

        def physics_recovery(error: ErrorEvent) -> bool:
            """Recovery strategy for physics engine errors"""
            try:
                # Fall back to simplified kinematics if physics fails
                if hasattr(self, "use_physics"):
                    self.use_physics = False
                    self.physics_engine = None
                return True
            except Exception:
                return False

        def sensor_recovery(error: ErrorEvent) -> bool:
            """Recovery strategy for sensor-related errors"""
            try:
                # Reset sensor readings to safe defaults
                self.state.ultrasonic_distance = 2.0  # 2m default
                self.state.imu_accel = np.array([0.0, 0.0, -9.81])
                self.state.imu_gyro = np.zeros(3)
                return True
            except Exception:
                return False

        # Register strategies (will be called after error_handler is initialized)
        if hasattr(self, "error_handler"):
            self.error_handler.register_recovery_strategy("robot_model", motor_recovery)
            self.error_handler.register_recovery_strategy("robot_model", physics_recovery)
            self.error_handler.register_recovery_strategy("robot_model", sensor_recovery)

    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file"""
        with open(config_path, "r") as f:
            return yaml.safe_load(f)

    def pwm_to_voltage(self, pwm: int) -> float:
        """Convert PWM value to voltage (assuming 3.3V logic, 7.4V motor supply)"""
        # PWM range: -255 to 255
        # Motor supply voltage: 7.4V
        return (pwm / 255.0) * 7.4

    def set_motor_commands(self, left_pwm: int, right_pwm: int):
        """Set motor PWM commands"""
        try:
            if self.use_motor_controllers:
                # Set PWM commands to controllers (they handle the control internally)
                self.motor_left_controller.set_pwm_command(left_pwm)
                self.motor_right_controller.set_pwm_command(right_pwm)

            self.state.motor_left_pwm = np.clip(left_pwm, -255, 255)
            self.state.motor_right_pwm = np.clip(right_pwm, -255, 255)
            self.error_handler.report_component_success("robot_model")
        except Exception as e:
            self.error_handler.handle_error(
                "robot_model",
                "motor_command_failed",
                f"Failed to set motor commands: {str(e)}",
                e,
                ErrorSeverity.MEDIUM,
            )

    def set_velocity_commands(self, left_velocity: float, right_velocity: float):
        """Set motor velocity commands in rad/s (PID mode)"""
        if self.use_motor_controllers:
            self.motor_left_controller.set_velocity_setpoint(left_velocity)
            self.motor_right_controller.set_velocity_setpoint(right_velocity)
        else:
            # Convert velocity to approximate PWM for non-PID mode
            # This is a rough approximation
            max_velocity = self.motor_left.max_rpm * 2 * np.pi / 60  # rad/s
            left_pwm = (left_velocity / max_velocity) * 255
            right_pwm = (right_velocity / max_velocity) * 255
            self.set_motor_commands(left_pwm, right_pwm)

    def set_position_commands(self, left_position: float, right_position: float):
        """Set motor position commands in degrees (PID mode)"""
        if self.use_motor_controllers:
            self.motor_left_controller.set_position_setpoint(left_position)
            self.motor_right_controller.set_position_setpoint(right_position)
        else:
            # Position control not available without PID controllers
            print("Warning: Position control requires PID controllers to be enabled")

    def get_encoder_positions(self) -> Tuple[float, float]:
        """Get encoder positions in degrees"""
        if self.use_motor_controllers:
            left_pos = self.motor_left_controller.get_encoder_position()
            right_pos = self.motor_right_controller.get_encoder_position()
            return left_pos, right_pos
        else:
            # Estimate from motor angular velocity integration
            # This is less accurate than actual encoders
            return 0.0, 0.0  # Not available without controllers

    def get_encoder_velocities(self) -> Tuple[float, float]:
        """Get encoder velocities in rad/s"""
        if self.use_motor_controllers:
            left_vel = self.motor_left_controller.get_encoder_velocity()
            right_vel = self.motor_right_controller.get_encoder_velocity()
            return left_vel, right_vel
        else:
            # Use actual motor velocities
            return self.motor_left.angular_velocity, self.motor_right.angular_velocity

    def update_kinematics(self, dt: float):
        """Update robot kinematics based on wheel velocities"""
        if self.use_physics and self.physics_engine:
            # Use physics engine for realistic dynamics

            # Convert motor torques to wheel forces
            # Force = Torque / wheel_radius
            force_left = self.motor_left.torque / self.wheel_radius
            force_right = self.motor_right.torque / self.wheel_radius

            # Update physics simulation
            self.physics_engine.update_robot_forces(
                "main", force_left, force_right, self.track_width
            )
            self.physics_engine.step(dt)

            # Get updated state from physics engine
            physics_state = self.physics_engine.get_robot_state("main")
            if physics_state:
                self.state.x = physics_state["position"][0]
                self.state.y = physics_state["position"][1]
                self.state.theta = physics_state["angle"]

                # Calculate velocities from physics
                self.state.v = np.sqrt(
                    physics_state["velocity"][0] ** 2 + physics_state["velocity"][1] ** 2
                )
                self.state.omega = physics_state["angular_velocity"]
        else:
            # Original kinematic model (fallback)
            # Convert motor angular velocities to wheel linear velocities
            v_left = self.motor_left.angular_velocity * self.wheel_radius
            v_right = self.motor_right.angular_velocity * self.wheel_radius

            # Differential drive kinematics
            self.state.v = (v_left + v_right) / 2.0  # Linear velocity
            self.state.omega = (v_right - v_left) / self.track_width  # Angular velocity

            # Update pose using discrete integration
            self.state.x += self.state.v * np.cos(self.state.theta) * dt
            self.state.y += self.state.v * np.sin(self.state.theta) * dt
            self.state.theta += self.state.omega * dt

            # Normalize theta to [-pi, pi]
            self.state.theta = np.arctan2(np.sin(self.state.theta), np.cos(self.state.theta))

    def update_motors(self, dt: float):
        """Update motor dynamics with optional PID control"""
        if self.use_motor_controllers:
            # Use PID controllers
            # Get current motor angular velocities
            omega_left = self.motor_left.angular_velocity
            omega_right = self.motor_right.angular_velocity

            # Update controllers (they handle PWM commands internally)
            controlled_pwm_left = self.motor_left_controller.update(omega_left, dt)
            controlled_pwm_right = self.motor_right_controller.update(omega_right, dt)

            # Convert controlled PWM to voltage
            voltage_left = self.pwm_to_voltage(controlled_pwm_left)
            voltage_right = self.pwm_to_voltage(controlled_pwm_right)

            # Update motor states for feedback to next controller update
            self.state.motor_left_pwm = controlled_pwm_left
            self.state.motor_right_pwm = controlled_pwm_right
        else:
            # Direct PWM control (original behavior)
            voltage_left = self.pwm_to_voltage(self.state.motor_left_pwm)
            voltage_right = self.pwm_to_voltage(self.state.motor_right_pwm)

        # Calculate load torque (simplified - resistance from ground friction)
        # In reality, this would depend on terrain, robot weight, etc.
        load_torque = 0.01  # N⋅m (constant friction)

        # Update motors
        omega_left, _ = self.motor_left.update(voltage_left, load_torque, dt)
        omega_right, _ = self.motor_right.update(voltage_right, load_torque, dt)

        # Convert to RPM for state
        self.state.motor_left_rpm = omega_left * 60 / (2 * np.pi)
        self.state.motor_right_rpm = omega_right * 60 / (2 * np.pi)

    def update_sensors(self, environment: Optional[Dict] = None):
        """Update sensor readings with noise"""
        # IMU simulation
        # Accelerometer: measure linear acceleration + gravity
        ax = 0.0  # Assume no linear acceleration for now
        ay = 0.0
        az = -9.81  # Gravity

        # Add noise
        accel_noise = self.rng.normal(0, self.imu_noise["accelerometer"], 3)
        self.state.imu_accel = np.array([ax, ay, az]) + accel_noise

        # Gyroscope: measure angular velocity
        gyro_noise = self.rng.normal(0, self.imu_noise["gyroscope"], 3)
        self.state.imu_gyro = np.array([0, 0, self.state.omega]) + gyro_noise

        # Ultrasonic sensor (placeholder - would need environment collision detection)
        if environment:
            # Simulate distance measurement (simplified)
            self.state.ultrasonic_distance = self._simulate_ultrasonic(environment)
        else:
            self.state.ultrasonic_distance = 2.0  # Default 2m

    def _simulate_ultrasonic(self, environment: Dict) -> float:
        """Simulate ultrasonic sensor reading"""
        # Sensor position and orientation (8cm forward from robot center)
        sensor_x = self.state.x + 0.08 * np.cos(self.state.theta)
        sensor_y = self.state.y + 0.08 * np.sin(self.state.theta)
        sensor_angle = self.state.theta

        max_range = 4.0  # Max sensor range

        if self.use_physics and self.physics_engine:
            # Use physics engine for accurate raycast
            distance = self.physics_engine.raycast_sensor(
                (sensor_x, sensor_y), sensor_angle, max_range
            )
        else:
            # Fallback to simplified collision detection
            min_distance = max_range

            # Check distance to obstacles
            if "obstacles" in environment:
                for obstacle in environment["obstacles"]:
                    if obstacle["type"] == "box":
                        # Simple box collision (simplified)
                        obs_x, obs_y = obstacle["position"]
                        distance = np.sqrt((sensor_x - obs_x) ** 2 + (sensor_y - obs_y) ** 2)
                        if distance < min_distance:
                            min_distance = distance

            distance = min_distance

        # Add noise
        noise = self.rng.normal(0, 0.01)  # 1cm noise
        return max(0.02, distance + noise)  # Minimum 2cm range

    def update(self, dt: Optional[float] = None):
        """Update complete robot state"""
        try:
            if dt is None:
                dt = self.dt

            # Update motor dynamics
            self.update_motors(dt)

            # Update kinematics
            self.update_kinematics(dt)

            # Update sensors
            environment = self.config["simulation"]["environment"]
            self.update_sensors(environment)

            # Update camera frame if camera simulation is enabled
            if self.use_camera_simulation and self.camera_simulation:
                latest_frame = self.camera_simulation.get_latest_frame()
                if latest_frame is not None:
                    self.state.camera_frame = latest_frame

            self.error_handler.report_component_success("robot_model")

        except Exception as e:
            handled = self.error_handler.handle_error(
                "robot_model",
                "update_failed",
                f"Robot update failed: {str(e)}",
                e,
                ErrorSeverity.HIGH,
            )
            if not handled:
                # Emergency stop if update fails critically
                self.state.motor_left_pwm = 0.0
                self.state.motor_right_pwm = 0.0

    def get_state(self) -> RobotState:
        """Get current robot state"""
        return self.state

    def get_pose(self) -> Tuple[float, float, float]:
        """Get robot pose (x, y, theta)"""
        return self.state.x, self.state.y, self.state.theta

    def get_velocity(self) -> Tuple[float, float]:
        """Get robot velocity (linear, angular)"""
        return self.state.v, self.state.omega

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset robot to initial state"""
        self.state = RobotState()
        self.state.x = x
        self.state.y = y
        self.state.theta = theta

        # Reset motors
        self.motor_left = DCMotor(self.config["robot"]["motors"]["left"])
        self.motor_right = DCMotor(self.config["robot"]["motors"]["right"])

        # Reset physics engine state if enabled
        if self.use_physics and self.robot_body:
            self.robot_body.position = (x, y)
            self.robot_body.angle = theta
            self.robot_body.velocity = (0, 0)
            self.robot_body.angular_velocity = 0


if __name__ == "__main__":
    # Test the robot model
    config_path = "../config/robot_config.yaml"
    robot = DifferentialDriveRobot(config_path)

    print("ESP32 Robot Car Simulation Test")
    print("=" * 40)

    # Test motor commands
    robot.set_motor_commands(100, 100)  # Forward

    # Run simulation for 5 seconds
    duration = 5.0
    steps = int(duration / robot.dt)

    for i in range(steps):
        robot.update()

        if i % 100 == 0:  # Print every second
            x, y, theta = robot.get_pose()
            v, omega = robot.get_velocity()
            print(
                f"t={i * robot.dt:.1f}s: pos=({x:.3f}, {y:.3f}), "
                f"θ={np.degrees(theta):.1f}°, v={v:.3f}m/s, ω={np.degrees(omega):.1f}°/s"
            )

    print("\nSimulation complete!")
