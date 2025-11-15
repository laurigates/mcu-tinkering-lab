"""
Motor Controller with PID Control and Encoder Feedback

This module implements a realistic motor control system with PID control
and encoder feedback simulation, matching the ESP32 motor control architecture.
"""

import numpy as np
import time
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class ControlMode(Enum):
    """Motor control modes"""

    OPEN_LOOP = "open_loop"  # Direct PWM control
    VELOCITY_PID = "velocity"  # Velocity control with PID
    POSITION_PID = "position"  # Position control with PID


@dataclass
class PIDConfig:
    """PID controller configuration"""

    kp: float = 1.0  # Proportional gain
    ki: float = 0.0  # Integral gain
    kd: float = 0.0  # Derivative gain
    output_limit: float = 255.0  # Output limit (PWM range)
    integral_limit: float = 100.0  # Anti-windup limit
    derivative_filter: float = 0.1  # Derivative filter coefficient (0-1)


@dataclass
class EncoderConfig:
    """Encoder configuration"""

    pulses_per_revolution: int = 600  # PPR (typical for TT motor with encoder)
    noise_std: float = 0.1  # Position noise standard deviation (degrees)
    velocity_filter: float = 0.8  # Low-pass filter for velocity (0-1)
    max_velocity_change: float = 500.0  # Maximum velocity change per step (deg/s)


class PIDController:
    """PID Controller implementation with anti-windup and derivative filtering"""

    def __init__(self, config: PIDConfig):
        self.config = config

        # State variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_derivative = 0.0
        self.previous_time = None

        # Reset flag for initialization
        self.initialized = False

    def update(self, setpoint: float, measured_value: float, dt: Optional[float] = None) -> float:
        """
        Update PID controller

        Args:
            setpoint: Desired value
            measured_value: Current measured value
            dt: Time step (auto-calculated if None)

        Returns:
            PID output
        """
        current_time = time.time()

        if dt is None:
            if self.previous_time is not None:
                dt = current_time - self.previous_time
            else:
                dt = 0.01  # Default timestep

        if not self.initialized:
            self.previous_error = setpoint - measured_value
            self.previous_time = current_time
            self.initialized = True
            return 0.0

        # Calculate error
        error = setpoint - measured_value

        # Proportional term
        proportional = self.config.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(
            self.integral, -self.config.integral_limit, self.config.integral_limit
        )
        integral = self.config.ki * self.integral

        # Derivative term with filtering
        if dt > 0:
            raw_derivative = (error - self.previous_error) / dt
            # Low-pass filter for derivative
            filtered_derivative = (
                self.config.derivative_filter * raw_derivative
                + (1 - self.config.derivative_filter) * self.previous_derivative
            )
            derivative = self.config.kd * filtered_derivative
            self.previous_derivative = filtered_derivative
        else:
            derivative = 0.0

        # Calculate output
        output = proportional + integral + derivative

        # Apply output limits
        output = np.clip(output, -self.config.output_limit, self.config.output_limit)

        # Update state
        self.previous_error = error
        self.previous_time = current_time

        return output

    def reset(self):
        """Reset PID controller state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_derivative = 0.0
        self.previous_time = None
        self.initialized = False


class EncoderSimulation:
    """Simulates a quadrature encoder with realistic characteristics"""

    def __init__(self, config: EncoderConfig):
        self.config = config

        # State variables
        self.position = 0.0  # degrees
        self.velocity = 0.0  # degrees/second
        self.previous_velocity = 0.0
        self.pulse_count = 0  # Raw encoder pulses

        # Noise generation
        self.rng = np.random.RandomState(42)

        # Velocity filtering
        self.velocity_filter_state = 0.0

    def update(self, actual_angular_velocity: float, dt: float):
        """
        Update encoder simulation based on actual motor angular velocity

        Args:
            actual_angular_velocity: True motor angular velocity (rad/s)
            dt: Time step (s)
        """
        # Convert rad/s to deg/s
        actual_velocity_deg = np.degrees(actual_angular_velocity)

        # Limit velocity change (simulates encoder resolution and noise)
        velocity_change = actual_velocity_deg - self.previous_velocity
        velocity_change = np.clip(
            velocity_change,
            -self.config.max_velocity_change * dt,
            self.config.max_velocity_change * dt,
        )

        raw_velocity = self.previous_velocity + velocity_change

        # Apply low-pass filter to velocity
        self.velocity = (
            self.config.velocity_filter * self.velocity
            + (1 - self.config.velocity_filter) * raw_velocity
        )

        # Update position
        position_change = self.velocity * dt
        self.position += position_change

        # Add noise to position
        noise = self.rng.normal(0, self.config.noise_std)
        self.position += noise

        # Update pulse count (for debugging/diagnostics)
        pulses_per_degree = self.config.pulses_per_revolution / 360.0
        self.pulse_count = int(self.position * pulses_per_degree)

        # Update previous state
        self.previous_velocity = raw_velocity

    def get_position(self) -> float:
        """Get encoder position in degrees"""
        return self.position

    def get_velocity(self) -> float:
        """Get encoder velocity in degrees/second"""
        return self.velocity

    def get_velocity_rad_s(self) -> float:
        """Get encoder velocity in rad/s"""
        return np.radians(self.velocity)

    def get_pulse_count(self) -> int:
        """Get raw pulse count"""
        return self.pulse_count

    def reset(self, position: float = 0.0):
        """Reset encoder to given position"""
        self.position = position
        self.velocity = 0.0
        self.previous_velocity = 0.0
        self.velocity_filter_state = 0.0
        pulses_per_degree = self.config.pulses_per_revolution / 360.0
        self.pulse_count = int(position * pulses_per_degree)


class MotorController:
    """
    Motor controller with PID control and encoder feedback

    This simulates the ESP32 motor control system with realistic
    PID controllers and encoder feedback loops.
    """

    def __init__(self, motor_config: Dict, control_config: Dict):
        self.motor_config = motor_config

        # Control mode
        self.control_mode = ControlMode(control_config.get("mode", "open_loop"))

        # PID controllers for velocity and position
        velocity_pid_config = PIDConfig(**control_config.get("velocity_pid", {}))
        position_pid_config = PIDConfig(**control_config.get("position_pid", {}))

        self.velocity_pid = PIDController(velocity_pid_config)
        self.position_pid = PIDController(position_pid_config)

        # Encoder simulation
        encoder_config = EncoderConfig(**control_config.get("encoder", {}))
        self.encoder = EncoderSimulation(encoder_config)

        # Control state
        self.velocity_setpoint = 0.0  # rad/s
        self.position_setpoint = 0.0  # degrees
        self.pwm_command = 0.0  # Direct PWM (-255 to 255)

        # Output
        self.output_pwm = 0.0

        # Feedforward (for better tracking)
        self.velocity_feedforward = control_config.get("velocity_feedforward", 0.0)

    def set_pwm_command(self, pwm: float):
        """Set direct PWM command (open loop mode)"""
        self.pwm_command = np.clip(pwm, -255, 255)
        self.control_mode = ControlMode.OPEN_LOOP

    def set_velocity_setpoint(self, velocity_rad_s: float):
        """Set velocity setpoint in rad/s"""
        self.velocity_setpoint = velocity_rad_s
        self.control_mode = ControlMode.VELOCITY_PID

    def set_position_setpoint(self, position_degrees: float):
        """Set position setpoint in degrees"""
        self.position_setpoint = position_degrees
        self.control_mode = ControlMode.POSITION_PID

    def update(self, actual_motor_velocity: float, dt: float) -> float:
        """
        Update motor controller

        Args:
            actual_motor_velocity: Actual motor angular velocity (rad/s)
            dt: Time step (s)

        Returns:
            PWM output (-255 to 255)
        """
        # Update encoder simulation
        self.encoder.update(actual_motor_velocity, dt)

        # Control based on mode
        if self.control_mode == ControlMode.OPEN_LOOP:
            self.output_pwm = self.pwm_command

        elif self.control_mode == ControlMode.VELOCITY_PID:
            # Velocity control with PID
            measured_velocity = self.encoder.get_velocity_rad_s()

            # PID output
            pid_output = self.velocity_pid.update(self.velocity_setpoint, measured_velocity, dt)

            # Add feedforward term
            feedforward = self.velocity_setpoint * self.velocity_feedforward

            self.output_pwm = pid_output + feedforward

        elif self.control_mode == ControlMode.POSITION_PID:
            # Position control with cascaded PID
            measured_position = self.encoder.get_position()

            # Position PID gives velocity command
            velocity_command = self.position_pid.update(
                self.position_setpoint, measured_position, dt
            )

            # Velocity PID gives PWM output
            measured_velocity = self.encoder.get_velocity_rad_s()
            pid_output = self.velocity_pid.update(
                np.radians(velocity_command), measured_velocity, dt
            )

            self.output_pwm = pid_output

        # Clamp output
        self.output_pwm = np.clip(self.output_pwm, -255, 255)

        return self.output_pwm

    def get_encoder_position(self) -> float:
        """Get encoder position in degrees"""
        return self.encoder.get_position()

    def get_encoder_velocity(self) -> float:
        """Get encoder velocity in rad/s"""
        return self.encoder.get_velocity_rad_s()

    def get_encoder_velocity_rpm(self) -> float:
        """Get encoder velocity in RPM"""
        return self.encoder.get_velocity_rad_s() * 60 / (2 * np.pi)

    def get_control_mode(self) -> ControlMode:
        """Get current control mode"""
        return self.control_mode

    def reset(self):
        """Reset controller state"""
        self.velocity_pid.reset()
        self.position_pid.reset()
        self.encoder.reset()
        self.output_pwm = 0.0
        self.pwm_command = 0.0
        self.velocity_setpoint = 0.0
        self.position_setpoint = 0.0


def main():
    """Test motor controller"""

    # Example configuration
    motor_config = {
        "max_rpm": 200,
        "stall_torque": 0.5,
        "resistance": 3.0,
    }

    control_config = {
        "mode": "velocity",
        "velocity_pid": {
            "kp": 2.0,
            "ki": 0.5,
            "kd": 0.1,
            "output_limit": 255.0,
        },
        "position_pid": {
            "kp": 1.0,
            "ki": 0.0,
            "kd": 0.05,
            "output_limit": 50.0,  # Output is velocity command
        },
        "encoder": {
            "pulses_per_revolution": 600,
            "noise_std": 0.1,
        },
        "velocity_feedforward": 10.0,  # PWM per rad/s
    }

    # Create controller
    controller = MotorController(motor_config, control_config)

    # Test velocity control
    print("Testing Motor Controller with PID")
    print("=" * 40)

    # Set velocity command
    target_velocity = 5.0  # rad/s
    controller.set_velocity_setpoint(target_velocity)

    # Simulate motor response
    actual_velocity = 0.0
    dt = 0.01

    print(f"Target velocity: {target_velocity:.2f} rad/s")
    print("Time\tPWM\tActual\tMeasured\tPosition")

    for i in range(300):  # 3 seconds
        t = i * dt

        # Update controller
        pwm_output = controller.update(actual_velocity, dt)

        # Simple motor model (first-order system)
        # In real simulation this would be the actual DC motor
        motor_time_constant = 0.1  # seconds
        target_from_pwm = pwm_output / 255.0 * 10.0  # Max 10 rad/s at full PWM
        actual_velocity += (target_from_pwm - actual_velocity) * dt / motor_time_constant

        # Print status every 50ms
        if i % 5 == 0:
            measured_velocity = controller.get_encoder_velocity()
            position = controller.get_encoder_position()
            print(
                f"{t:.2f}\t{pwm_output:.1f}\t{actual_velocity:.2f}\t{measured_velocity:.2f}\t{position:.1f}"
            )

    print("\nTesting position control...")

    # Switch to position control
    target_position = 180.0  # degrees
    controller.set_position_setpoint(target_position)

    print(f"Target position: {target_position:.1f} degrees")
    print("Time\tPWM\tActual\tMeasured\tPosition")

    for i in range(500):  # 5 seconds
        t = i * dt

        # Update controller
        pwm_output = controller.update(actual_velocity, dt)

        # Motor model
        target_from_pwm = pwm_output / 255.0 * 10.0
        actual_velocity += (target_from_pwm - actual_velocity) * dt / motor_time_constant

        # Print status every 100ms
        if i % 10 == 0:
            measured_velocity = controller.get_encoder_velocity()
            position = controller.get_encoder_position()
            print(
                f"{t:.2f}\t{pwm_output:.1f}\t{actual_velocity:.2f}\t{measured_velocity:.2f}\t{position:.1f}"
            )


if __name__ == "__main__":
    main()
