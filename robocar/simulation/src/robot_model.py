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
        self.resistance = config['resistance']  # Ohms
        self.inductance = config['inductance']  # H
        self.back_emf_constant = config['back_emf_constant']  # V⋅s/rad
        # Torque constant derived from stall specifications for consistency
        self.torque_constant = config['stall_torque'] / config['stall_current']  # N⋅m/A
        self.max_rpm = config['max_rpm']
        self.stall_torque = config['stall_torque']  # N⋅m
        self.no_load_current = config['no_load_current']  # A
        self.stall_current = config['stall_current']  # A
        
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
        self.current += (target_current - self.current) * (dt / time_constant) * 0.1  # Damping factor for stability
        
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
        self.angular_velocity = np.clip(self.angular_velocity, 
                                      -max_angular_velocity, max_angular_velocity)
        
        self.torque = motor_torque
        
        return self.angular_velocity, self.torque


class DifferentialDriveRobot:
    """Differential drive robot model with realistic motor dynamics"""
    
    def __init__(self, config_path: str):
        """Initialize robot model from configuration file"""
        self.config = self._load_config(config_path)
        self.state = RobotState()
        
        # Robot physical parameters
        robot_config = self.config['robot']
        self.wheelbase = robot_config['dimensions']['wheelbase']
        self.track_width = robot_config['dimensions']['track_width']
        self.wheel_radius = robot_config['dimensions']['wheel_radius']
        self.mass = robot_config['mass']
        self.inertia = robot_config['inertia']
        
        # Initialize motors
        self.motor_left = DCMotor(robot_config['motors']['left'])
        self.motor_right = DCMotor(robot_config['motors']['right'])
        
        # Simulation parameters
        self.dt = self.config['simulation']['timestep']
        
        # Noise parameters for sensors
        self.imu_noise = robot_config['sensors']['imu']['noise']
        
        # Initialize random number generator for sensor noise
        self.rng = np.random.RandomState(42)
        
    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file"""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    def pwm_to_voltage(self, pwm: int) -> float:
        """Convert PWM value to voltage (assuming 3.3V logic, 7.4V motor supply)"""
        # PWM range: -255 to 255
        # Motor supply voltage: 7.4V
        return (pwm / 255.0) * 7.4
    
    def set_motor_commands(self, left_pwm: int, right_pwm: int):
        """Set motor PWM commands"""
        self.state.motor_left_pwm = np.clip(left_pwm, -255, 255)
        self.state.motor_right_pwm = np.clip(right_pwm, -255, 255)
    
    def update_kinematics(self, dt: float):
        """Update robot kinematics based on wheel velocities"""
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
        """Update motor dynamics"""
        # Convert PWM to voltage
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
        accel_noise = self.rng.normal(0, self.imu_noise['accelerometer'], 3)
        self.state.imu_accel = np.array([ax, ay, az]) + accel_noise
        
        # Gyroscope: measure angular velocity
        gyro_noise = self.rng.normal(0, self.imu_noise['gyroscope'], 3)
        self.state.imu_gyro = np.array([0, 0, self.state.omega]) + gyro_noise
        
        # Ultrasonic sensor (placeholder - would need environment collision detection)
        if environment:
            # Simulate distance measurement (simplified)
            self.state.ultrasonic_distance = self._simulate_ultrasonic(environment)
        else:
            self.state.ultrasonic_distance = 2.0  # Default 2m
    
    def _simulate_ultrasonic(self, environment: Dict) -> float:
        """Simulate ultrasonic sensor reading"""
        # Simplified ray casting for ultrasonic sensor
        # In practice, this would use the physics engine for collision detection
        
        # Sensor position and orientation
        sensor_x = self.state.x + 0.08 * np.cos(self.state.theta)  # 8cm forward
        sensor_y = self.state.y + 0.08 * np.sin(self.state.theta)
        sensor_angle = self.state.theta
        
        min_distance = 4.0  # Max range
        
        # Check distance to obstacles
        if 'obstacles' in environment:
            for obstacle in environment['obstacles']:
                if obstacle['type'] == 'box':
                    # Simple box collision (simplified)
                    obs_x, obs_y = obstacle['position']
                    distance = np.sqrt((sensor_x - obs_x)**2 + (sensor_y - obs_y)**2)
                    if distance < min_distance:
                        min_distance = distance
        
        # Add noise
        noise = self.rng.normal(0, 0.01)  # 1cm noise
        return max(0.02, min_distance + noise)  # Minimum 2cm range
    
    def update(self, dt: Optional[float] = None):
        """Update complete robot state"""
        if dt is None:
            dt = self.dt
        
        # Update motor dynamics
        self.update_motors(dt)
        
        # Update kinematics
        self.update_kinematics(dt)
        
        # Update sensors
        environment = self.config['simulation']['environment']
        self.update_sensors(environment)
    
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
        self.motor_left = DCMotor(self.config['robot']['motors']['left'])
        self.motor_right = DCMotor(self.config['robot']['motors']['right'])


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
            print(f"t={i*robot.dt:.1f}s: pos=({x:.3f}, {y:.3f}), "
                  f"θ={np.degrees(theta):.1f}°, v={v:.3f}m/s, ω={np.degrees(omega):.1f}°/s")
    
    print("\nSimulation complete!")