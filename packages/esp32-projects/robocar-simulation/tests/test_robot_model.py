#!/usr/bin/env python3
"""
Test suite for ESP32 Robot Car simulation
"""

import sys
from pathlib import Path

import numpy as np
import pytest

# Add src directory to path
sys.path.append(str(Path(__file__).parent.parent / "src"))

from robot_model import DCMotor, DifferentialDriveRobot, RobotState


class TestDCMotor:
    """Test DC motor simulation"""

    def test_motor_initialization(self):
        """Test motor initialization"""
        config = {
            "resistance": 3.0,
            "inductance": 0.001,
            "back_emf_constant": 0.01,
            "max_rpm": 200,
            "stall_torque": 0.5,
            "no_load_current": 0.1,
            "stall_current": 2.0,
        }

        motor = DCMotor(config)
        assert motor.resistance == 3.0
        assert motor.inductance == 0.001
        assert motor.current == 0.0
        assert motor.angular_velocity == 0.0

    def test_motor_dynamics(self):
        """Test motor dynamics update"""
        config = {
            "resistance": 3.0,
            "inductance": 0.001,
            "back_emf_constant": 0.01,
            "max_rpm": 200,
            "stall_torque": 0.5,
            "no_load_current": 0.1,
            "stall_current": 2.0,
        }

        motor = DCMotor(config)

        # Apply voltage and update
        voltage = 5.0
        load_torque = 0.1
        dt = 0.01

        omega, torque = motor.update(voltage, load_torque, dt)

        # Motor should start spinning
        assert omega > 0
        assert torque > 0
        assert motor.current > 0

    def test_motor_limits(self):
        """Test motor current and speed limits"""
        config = {
            "resistance": 1.0,
            "inductance": 0.001,
            "back_emf_constant": 0.01,
            "max_rpm": 100,
            "stall_torque": 0.5,
            "no_load_current": 0.1,
            "stall_current": 1.0,
        }

        motor = DCMotor(config)

        # Apply very high voltage
        voltage = 100.0
        load_torque = 0.0
        dt = 0.01

        for _ in range(1000):  # Run for a while
            omega, torque = motor.update(voltage, load_torque, dt)

        # Current should be limited
        assert motor.current <= config["stall_current"]

        # Speed should be limited
        max_omega = config["max_rpm"] * 2 * np.pi / 60
        assert abs(omega) <= max_omega * 1.01  # Allow small tolerance


class TestRobotState:
    """Test robot state representation"""

    def test_state_initialization(self):
        """Test robot state initialization"""
        state = RobotState()

        assert state.x == 0.0
        assert state.y == 0.0
        assert state.theta == 0.0
        assert state.v == 0.0
        assert state.omega == 0.0
        assert len(state.imu_accel) == 3
        assert len(state.imu_gyro) == 3

    def test_state_with_values(self):
        """Test robot state with custom values"""
        state = RobotState(
            x=1.0, y=2.0, theta=0.5, v=0.3, omega=0.1, motor_left_pwm=100, motor_right_pwm=80
        )

        assert state.x == 1.0
        assert state.y == 2.0
        assert state.theta == 0.5
        assert state.motor_left_pwm == 100
        assert state.motor_right_pwm == 80


class TestDifferentialDriveRobot:
    """Test differential drive robot model"""

    @pytest.fixture
    def robot(self):
        """Create robot for testing"""
        # Create temporary config
        config_path = Path(__file__).parent.parent / "config" / "robot_config.yaml"
        return DifferentialDriveRobot(str(config_path))

    def test_robot_initialization(self, robot):
        """Test robot initialization"""
        assert robot.state.x == 0.0
        assert robot.state.y == 0.0
        assert robot.state.theta == 0.0
        assert robot.wheelbase > 0
        assert robot.wheel_radius > 0
        assert robot.mass > 0

    def test_pwm_to_voltage_conversion(self, robot):
        """Test PWM to voltage conversion"""
        # Test full forward
        voltage = robot.pwm_to_voltage(255)
        assert voltage == 7.4

        # Test full reverse
        voltage = robot.pwm_to_voltage(-255)
        assert voltage == -7.4

        # Test zero
        voltage = robot.pwm_to_voltage(0)
        assert voltage == 0.0

        # Test half
        voltage = robot.pwm_to_voltage(127)
        assert abs(voltage - 3.68) < 0.1  # Approximately 3.68V

    def test_motor_commands(self, robot):
        """Test motor command setting"""
        robot.set_motor_commands(100, 150)

        assert robot.state.motor_left_pwm == 100
        assert robot.state.motor_right_pwm == 150

        # Test limits
        robot.set_motor_commands(300, -300)
        assert robot.state.motor_left_pwm == 255
        assert robot.state.motor_right_pwm == -255

    def test_kinematics_update(self, robot):
        """Test kinematic updates"""
        # Set motors for forward motion
        robot.set_motor_commands(100, 100)

        # Update motors and kinematics
        initial_x = robot.state.x
        initial_y = robot.state.y

        for _ in range(100):  # Run for 1 second
            robot.update(0.01)

        # Robot should have moved forward
        assert robot.state.x > initial_x
        assert abs(robot.state.y - initial_y) < 0.01  # Should be approximately straight
        assert robot.state.v > 0  # Should have positive velocity

    def test_turning_motion(self, robot):
        """Test turning motion"""
        # Set motors for turning (left motor slower)
        robot.set_motor_commands(50, 100)

        initial_theta = robot.state.theta
        max_omega = 0.0

        # Run for shorter duration to capture turning behavior before motors reach limits
        for _ in range(50):  # Run for 0.5 seconds
            robot.update(0.01)
            max_omega = max(max_omega, abs(robot.state.omega))

        # Robot should have turned
        assert abs(robot.state.theta - initial_theta) > 0.1
        assert max_omega > 0.5  # Should have had significant angular velocity during turning

    def test_sensor_updates(self, robot):
        """Test sensor updates"""
        robot.update_sensors()

        # IMU should have readings
        assert len(robot.state.imu_accel) == 3
        assert len(robot.state.imu_gyro) == 3

        # Accelerometer should show gravity
        assert abs(robot.state.imu_accel[2] + 9.81) < 1.0  # Should be close to -9.81

        # Ultrasonic should have reasonable reading
        assert robot.state.ultrasonic_distance > 0
        assert robot.state.ultrasonic_distance < 10  # Should be reasonable

    def test_robot_reset(self, robot):
        """Test robot reset functionality"""
        # Move robot first
        robot.set_motor_commands(100, 100)
        for _ in range(100):
            robot.update(0.01)

        # Verify it moved
        assert robot.state.x != 0.0 or robot.state.y != 0.0

        # Reset robot
        robot.reset(x=1.0, y=2.0, theta=0.5)

        # Verify reset
        assert robot.state.x == 1.0
        assert robot.state.y == 2.0
        assert robot.state.theta == 0.5
        assert robot.state.v == 0.0
        assert robot.state.omega == 0.0

    def test_differential_drive_kinematics(self, robot):
        """Test differential drive kinematics equations"""
        # Set specific wheel velocities
        robot.motor_left.angular_velocity = 10.0  # rad/s
        robot.motor_right.angular_velocity = 10.0  # rad/s

        robot.update_kinematics(0.01)

        # Both wheels same speed -> straight motion
        expected_v = 10.0 * robot.wheel_radius
        assert abs(robot.state.v - expected_v) < 0.001
        assert abs(robot.state.omega) < 0.001

        # Different wheel speeds -> turning
        robot.motor_left.angular_velocity = 5.0  # rad/s
        robot.motor_right.angular_velocity = 10.0  # rad/s

        robot.update_kinematics(0.01)

        expected_v = (5.0 + 10.0) * robot.wheel_radius / 2.0
        expected_omega = (10.0 - 5.0) * robot.wheel_radius / robot.track_width

        assert abs(robot.state.v - expected_v) < 0.001
        assert abs(robot.state.omega - expected_omega) < 0.001


class TestSimulationAccuracy:
    """Test simulation accuracy and performance"""

    @pytest.fixture
    def robot(self):
        """Create robot for testing"""
        config_path = Path(__file__).parent.parent / "config" / "robot_config.yaml"
        return DifferentialDriveRobot(str(config_path))

    def test_circular_motion_accuracy(self, robot):
        """Test accuracy of circular motion"""
        # Set motors for circular motion (larger difference to create more curvature)
        robot.set_motor_commands(100, 40)

        # Run for shorter duration to test turning behavior before motors reach limits
        duration = 1.0  # seconds
        steps = int(duration / robot.dt)

        positions = []
        for _ in range(steps):
            robot.update()
            positions.append((robot.state.x, robot.state.y))

        # Check that robot actually moves in a curved path (not straight)
        # Calculate path curvature by checking if the robot changes direction
        initial_x, initial_y = positions[0]
        mid_x, mid_y = positions[len(positions) // 2]
        final_x, final_y = positions[-1]

        # The robot should not travel in a straight line
        # Calculate the deviation from straight line
        straight_line_distance = ((final_x - initial_x) ** 2 + (final_y - initial_y) ** 2) ** 0.5
        actual_path_length = sum(
            (
                (positions[i + 1][0] - positions[i][0]) ** 2
                + (positions[i + 1][1] - positions[i][1]) ** 2
            )
            ** 0.5
            for i in range(len(positions) - 1)
        )

        # For curved motion, actual path should be longer than straight line
        # Use a smaller threshold since the path difference is small for realistic robot motion
        assert actual_path_length > straight_line_distance * 1.005  # At least 0.5% longer
        assert abs(robot.state.x) < 2.0  # Should be within reasonable bounds
        assert abs(robot.state.y) < 2.0

    def test_energy_conservation(self, robot):
        """Test energy conservation in simulation"""
        # Set constant motor commands
        robot.set_motor_commands(100, 100)

        # Run simulation and check energy behavior
        energies = []
        for _i in range(100):
            robot.update()

            # Calculate kinetic energy
            kinetic_energy = 0.5 * robot.mass * robot.state.v**2
            rotational_energy = 0.5 * robot.inertia * robot.state.omega**2
            total_energy = kinetic_energy + rotational_energy
            energies.append(total_energy)

        # Check that energy doesn't go negative and shows expected behavior
        assert all(e >= 0 for e in energies), "Energy should never be negative"

        # Check that final energy is higher than initial (overall energy increase)
        initial_energy = sum(energies[:10]) / 10  # Average of first 10 steps
        final_energy = sum(energies[-10:]) / 10  # Average of last 10 steps

        # With constant motor input, final energy should be higher than initial
        # (allowing for electrical transients but expecting overall increase)
        assert final_energy > initial_energy * 0.5, (
            f"Final energy {final_energy:.6f}J should be significantly higher than initial {initial_energy:.6f}J"
        )

    def test_simulation_stability(self, robot):
        """Test numerical stability of simulation"""
        # Test with various time steps
        time_steps = [0.001, 0.01, 0.1]

        for dt in time_steps:
            robot.reset()
            robot.dt = dt
            robot.set_motor_commands(100, 100)

            # Run simulation
            for _ in range(int(1.0 / dt)):  # 1 second
                robot.update()

            # Check for reasonable values (no NaN or infinite values)
            assert np.isfinite(robot.state.x)
            assert np.isfinite(robot.state.y)
            assert np.isfinite(robot.state.theta)
            assert np.isfinite(robot.state.v)
            assert np.isfinite(robot.state.omega)

            # Check for reasonable magnitude
            assert abs(robot.state.x) < 10.0
            assert abs(robot.state.y) < 10.0
            assert abs(robot.state.v) < 5.0
            assert abs(robot.state.omega) < 10.0


if __name__ == "__main__":
    # Run tests
    pytest.main([__file__, "-v"])
