#!/usr/bin/env python3
"""
Test PID Motor Control Integration

This test verifies that the PID motor controllers are working correctly
with encoder feedback and can maintain velocity and position setpoints.
"""

import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from robot_model import DifferentialDriveRobot
from motor_controller import MotorController, PIDConfig, EncoderConfig


def test_motor_controller_standalone():
    """Test motor controller independently"""
    print("🔧 Testing standalone motor controller...")
    
    try:
        # Motor configuration
        motor_config = {
            'max_rpm': 200,
            'stall_torque': 0.5,
            'resistance': 3.0,
        }
        
        # Control configuration
        control_config = {
            'mode': 'velocity',
            'velocity_pid': {
                'kp': 2.0,
                'ki': 0.5,
                'kd': 0.1,
                'output_limit': 255.0,
            },
            'encoder': {
                'pulses_per_revolution': 600,
                'noise_std': 0.1,
            },
            'velocity_feedforward': 10.0,
        }
        
        # Create controller
        controller = MotorController(motor_config, control_config)
        
        # Test velocity setpoint
        target_velocity = 5.0  # rad/s
        controller.set_velocity_setpoint(target_velocity)
        
        # Simple motor simulation
        actual_velocity = 0.0
        dt = 0.01
        motor_time_constant = 0.1
        
        # Run for 2 seconds
        for i in range(200):
            pwm_output = controller.update(actual_velocity, dt)
            
            # Simple first-order motor model
            target_from_pwm = pwm_output / 255.0 * 10.0  # Max 10 rad/s
            actual_velocity += (target_from_pwm - actual_velocity) * dt / motor_time_constant
        
        # Check if we're close to target
        measured_velocity = controller.get_encoder_velocity()
        error = abs(measured_velocity - target_velocity)
        
        assert error < 0.5, f"Velocity error too large: {error:.2f} rad/s"
        print(f"  ✅ Velocity control: target={target_velocity:.2f}, measured={measured_velocity:.2f}")
        
        # Test position control
        controller.set_position_setpoint(180.0)  # 180 degrees
        
        for i in range(300):  # 3 seconds
            pwm_output = controller.update(actual_velocity, dt)
            target_from_pwm = pwm_output / 255.0 * 10.0
            actual_velocity += (target_from_pwm - actual_velocity) * dt / motor_time_constant
        
        position = controller.get_encoder_position()
        position_error = abs(position - 180.0)
        
        assert position_error < 10.0, f"Position error too large: {position_error:.1f} degrees"
        print(f"  ✅ Position control: target=180.0°, measured={position:.1f}°")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Standalone motor controller failed: {e}")
        return False


def test_robot_pid_integration():
    """Test PID motor control integration with robot"""
    print("🔧 Testing robot PID integration...")
    
    try:
        # Create robot with PID enabled
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)
        
        # Check if PID controllers are enabled
        assert robot.use_motor_controllers, "Motor controllers should be enabled"
        assert robot.motor_left_controller is not None, "Left motor controller should exist"
        assert robot.motor_right_controller is not None, "Right motor controller should exist"
        
        print("  ✅ PID controllers initialized")
        
        # Test velocity control
        left_velocity = 3.0   # rad/s
        right_velocity = 2.5  # rad/s (slight turn)
        
        robot.set_velocity_commands(left_velocity, right_velocity)
        
        # Run simulation for a while
        for i in range(200):  # 2 seconds
            robot.update()
            time.sleep(0.001)  # Small delay to prevent overwhelming
        
        # Check encoder readings
        left_vel, right_vel = robot.get_encoder_velocities()
        left_pos, right_pos = robot.get_encoder_positions()
        
        left_error = abs(left_vel - left_velocity)
        right_error = abs(right_vel - right_velocity)
        
        print(f"  📊 Left motor: target={left_velocity:.2f}, measured={left_vel:.2f}, error={left_error:.2f}")
        print(f"  📊 Right motor: target={right_velocity:.2f}, measured={right_vel:.2f}, error={right_error:.2f}")
        print(f"  📊 Encoder positions: left={left_pos:.1f}°, right={right_pos:.1f}°")
        
        # Allow for some settling time and control error
        assert left_error < 1.0, f"Left velocity error too large: {left_error:.2f}"
        assert right_error < 1.0, f"Right velocity error too large: {right_error:.2f}"
        
        print("  ✅ Velocity control working")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Robot PID integration failed: {e}")
        return False


def test_position_control():
    """Test position control functionality"""
    print("🔧 Testing position control...")
    
    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)
        
        # Test position commands
        target_left = 360.0   # One full rotation
        target_right = 180.0  # Half rotation
        
        robot.set_position_commands(target_left, target_right)
        
        # Run simulation
        for i in range(500):  # 5 seconds
            robot.update()
            time.sleep(0.001)
        
        # Check final positions
        left_pos, right_pos = robot.get_encoder_positions()
        
        left_error = abs(left_pos - target_left)
        right_error = abs(right_pos - target_right)
        
        print(f"  📊 Left position: target={target_left:.1f}°, measured={left_pos:.1f}°, error={left_error:.1f}°")
        print(f"  📊 Right position: target={target_right:.1f}°, measured={right_pos:.1f}°, error={right_error:.1f}°")
        
        # Position control should be fairly accurate
        assert left_error < 20.0, f"Left position error too large: {left_error:.1f}°"
        assert right_error < 20.0, f"Right position error too large: {right_error:.1f}°"
        
        print("  ✅ Position control working")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Position control failed: {e}")
        return False


def test_encoder_simulation():
    """Test encoder simulation accuracy"""
    print("🔧 Testing encoder simulation...")
    
    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)
        
        # Test encoder tracking during movement
        initial_pos_left, initial_pos_right = robot.get_encoder_positions()
        
        # Set constant velocity
        robot.set_velocity_commands(2.0, 2.0)  # 2 rad/s both wheels
        
        # Track for 3 seconds
        positions = []
        velocities = []
        times = []
        
        start_time = time.time()
        for i in range(300):
            robot.update()
            
            left_pos, right_pos = robot.get_encoder_positions()
            left_vel, right_vel = robot.get_encoder_velocities()
            
            current_time = time.time() - start_time
            
            positions.append([left_pos, right_pos])
            velocities.append([left_vel, right_vel])
            times.append(current_time)
            
            time.sleep(0.001)
        
        # Check velocity consistency
        final_velocities = velocities[-10:]  # Last 10 readings
        avg_left_vel = np.mean([v[0] for v in final_velocities])
        avg_right_vel = np.mean([v[1] for v in final_velocities])
        
        vel_std_left = np.std([v[0] for v in final_velocities])
        vel_std_right = np.std([v[1] for v in final_velocities])
        
        print(f"  📊 Average velocities: left={avg_left_vel:.2f}, right={avg_right_vel:.2f}")
        print(f"  📊 Velocity std dev: left={vel_std_left:.3f}, right={vel_std_right:.3f}")
        
        # Velocity should be close to setpoint and relatively stable
        assert abs(avg_left_vel - 2.0) < 0.5, f"Left velocity not tracking: {avg_left_vel:.2f}"
        assert abs(avg_right_vel - 2.0) < 0.5, f"Right velocity not tracking: {avg_right_vel:.2f}"
        assert vel_std_left < 0.2, f"Left velocity too noisy: {vel_std_left:.3f}"
        assert vel_std_right < 0.2, f"Right velocity too noisy: {vel_std_right:.3f}"
        
        print("  ✅ Encoder simulation working accurately")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Encoder simulation failed: {e}")
        return False


def test_control_mode_switching():
    """Test switching between control modes"""
    print("🔧 Testing control mode switching...")
    
    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)
        
        # Start with PWM control
        robot.set_motor_commands(100, 100)
        
        for i in range(50):
            robot.update()
        
        pwm_positions = robot.get_encoder_positions()
        print(f"  📊 After PWM control: positions={pwm_positions}")
        
        # Switch to velocity control
        robot.set_velocity_commands(1.0, 1.0)
        
        for i in range(100):
            robot.update()
        
        vel_positions = robot.get_encoder_positions()
        velocities = robot.get_encoder_velocities()
        print(f"  📊 After velocity control: positions={vel_positions}, velocities={velocities}")
        
        # Switch to position control
        robot.set_position_commands(180.0, 180.0)
        
        for i in range(200):
            robot.update()
        
        final_positions = robot.get_encoder_positions()
        print(f"  📊 After position control: positions={final_positions}")
        
        # Position should be close to setpoint
        pos_error_left = abs(final_positions[0] - 180.0)
        pos_error_right = abs(final_positions[1] - 180.0)
        
        assert pos_error_left < 30.0, f"Position error after mode switch: {pos_error_left:.1f}°"
        assert pos_error_right < 30.0, f"Position error after mode switch: {pos_error_right:.1f}°"
        
        print("  ✅ Control mode switching working")
        
        return True
        
    except Exception as e:
        print(f"  ❌ Control mode switching failed: {e}")
        return False


def run_all_tests():
    """Run all PID motor control tests"""
    print("🎛️  PID Motor Control Test Suite")
    print("=" * 50)
    
    tests = [
        test_motor_controller_standalone,
        test_robot_pid_integration,
        test_position_control,
        test_encoder_simulation,
        test_control_mode_switching,
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        try:
            if test():
                passed += 1
            print()
        except Exception as e:
            print(f"  💥 Test crashed: {e}")
            print()
    
    print("📋 Test Results")
    print("-" * 20)
    print(f"Passed: {passed}/{total}")
    print(f"Failed: {total - passed}/{total}")
    
    if passed == total:
        print("🎉 All PID motor control tests passed!")
        return True
    else:
        print("⚠️  Some PID motor control tests failed")
        return False


if __name__ == "__main__":
    print("PID Motor Control Test")
    print("Press Ctrl+C to stop at any time")
    
    try:
        success = run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 Tests interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 Test suite crashed: {e}")
        sys.exit(1)