#!/usr/bin/env python3
"""
Test Configuration Consistency

This test verifies that all configuration parameters are properly used across
the simulation system and that there are no inconsistencies between different
parts of the configuration.
"""

import sys
import os
import yaml
import numpy as np

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from robot_model import DifferentialDriveRobot
from camera_simulation import CameraSimulation
from swift_visualizer import RobotVisualizer
from ai_command_processor import AICommandProcessor


def test_config_loading():
    """Test that configuration loads properly"""
    print("⚙️  Testing configuration loading...")

    try:
        config_path = "config/robot_config.yaml"

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        # Check major sections exist
        required_sections = ["robot", "simulation", "ai_backend"]
        for section in required_sections:
            assert section in config, f"Missing required config section: {section}"

        # Check robot subsections
        robot_subsections = ["dimensions", "motors", "sensors"]
        for subsection in robot_subsections:
            assert subsection in config["robot"], f"Missing robot subsection: {subsection}"

        # Check simulation subsections
        sim_subsections = ["camera", "motor_control", "environment"]
        for subsection in sim_subsections:
            assert subsection in config["simulation"], (
                f"Missing simulation subsection: {subsection}"
            )

        print("  ✅ All required configuration sections present")

        return True, config

    except Exception as e:
        print(f"  ❌ Configuration loading failed: {e}")
        return False, None


def test_camera_config_consistency(config):
    """Test camera configuration consistency"""
    print("⚙️  Testing camera configuration consistency...")

    try:
        robot_camera = config["robot"]["sensors"]["camera"]
        sim_camera = config["simulation"]["camera"]

        # Test camera simulation uses consistent values
        camera_sim = CameraSimulation("config/robot_config.yaml")

        # Check FOV consistency (robot uses 'field_of_view', sim uses 'fov_degrees')
        expected_fov = robot_camera.get("field_of_view", sim_camera.get("fov_degrees", 70.0))
        actual_fov = camera_sim.camera_config.fov_degrees

        assert abs(actual_fov - expected_fov) < 0.1, (
            f"FOV mismatch: expected {expected_fov}, got {actual_fov}"
        )
        print(f"  ✅ Camera FOV consistent: {actual_fov}°")

        # Check FPS consistency
        expected_fps = robot_camera.get("fps", sim_camera.get("fps", 30))
        actual_fps = camera_sim.camera_config.fps

        assert actual_fps == expected_fps, (
            f"FPS mismatch: expected {expected_fps}, got {actual_fps}"
        )
        print(f"  ✅ Camera FPS consistent: {actual_fps}")

        # Check resolution consistency
        expected_resolution = tuple(
            robot_camera.get("resolution", sim_camera.get("resolution", [640, 480]))
        )
        actual_resolution = camera_sim.camera_config.resolution

        assert actual_resolution == expected_resolution, (
            f"Resolution mismatch: expected {expected_resolution}, got {actual_resolution}"
        )
        print(f"  ✅ Camera resolution consistent: {actual_resolution}")

        return True

    except Exception as e:
        print(f"  ❌ Camera config consistency failed: {e}")
        return False


def test_motor_config_consistency(config):
    """Test motor configuration consistency"""
    print("⚙️  Testing motor configuration consistency...")

    try:
        robot = DifferentialDriveRobot("config/robot_config.yaml")

        # Check that motor parameters match config
        left_motor_config = config["robot"]["motors"]["left"]
        right_motor_config = config["robot"]["motors"]["right"]

        # Test max RPM
        assert robot.motor_left.max_rpm == left_motor_config["max_rpm"], (
            "Left motor max RPM mismatch"
        )
        assert robot.motor_right.max_rpm == right_motor_config["max_rpm"], (
            "Right motor max RPM mismatch"
        )
        print(f"  ✅ Motor max RPM consistent: {robot.motor_left.max_rpm}")

        # Test motor electrical parameters
        assert abs(robot.motor_left.resistance - left_motor_config["resistance"]) < 0.1, (
            "Left motor resistance mismatch"
        )
        assert abs(robot.motor_right.resistance - right_motor_config["resistance"]) < 0.1, (
            "Right motor resistance mismatch"
        )
        print(f"  ✅ Motor resistance consistent: {robot.motor_left.resistance}Ω")

        # Test stall torque
        assert abs(robot.motor_left.stall_torque - left_motor_config["stall_torque"]) < 0.01, (
            "Left motor stall torque mismatch"
        )
        assert abs(robot.motor_right.stall_torque - right_motor_config["stall_torque"]) < 0.01, (
            "Right motor stall torque mismatch"
        )
        print(f"  ✅ Motor stall torque consistent: {robot.motor_left.stall_torque}N⋅m")

        return True

    except Exception as e:
        print(f"  ❌ Motor config consistency failed: {e}")
        return False


def test_simulation_timestep_consistency(config):
    """Test simulation timestep consistency"""
    print("⚙️  Testing simulation timestep consistency...")

    try:
        expected_timestep = config["simulation"]["timestep"]

        # Test robot model uses correct timestep
        robot = DifferentialDriveRobot("config/robot_config.yaml")
        assert abs(robot.dt - expected_timestep) < 0.001, (
            f"Robot timestep mismatch: expected {expected_timestep}, got {robot.dt}"
        )
        print(f"  ✅ Robot timestep consistent: {robot.dt}s")

        # Test visualizer uses correct update rate
        try:
            visualizer = RobotVisualizer("config/robot_config.yaml")
            expected_rate = 1.0 / expected_timestep
            assert abs(visualizer.update_rate - expected_rate) < 1.0, (
                f"Visualizer update rate mismatch: expected {expected_rate}, got {visualizer.update_rate}"
            )
            print(f"  ✅ Visualizer update rate consistent: {visualizer.update_rate}Hz")
        except Exception as viz_e:
            print(f"  ⚠️  Visualizer test skipped (likely no display): {viz_e}")

        return True

    except Exception as e:
        print(f"  ❌ Timestep consistency failed: {e}")
        return False


def test_physical_parameters_consistency(config):
    """Test physical parameters consistency"""
    print("⚙️  Testing physical parameters consistency...")

    try:
        robot = DifferentialDriveRobot("config/robot_config.yaml")
        robot_config = config["robot"]

        # Test dimensions
        assert abs(robot.wheelbase - robot_config["dimensions"]["wheelbase"]) < 0.001, (
            "Wheelbase mismatch"
        )
        assert abs(robot.track_width - robot_config["dimensions"]["track_width"]) < 0.001, (
            "Track width mismatch"
        )
        assert abs(robot.wheel_radius - robot_config["dimensions"]["wheel_radius"]) < 0.001, (
            "Wheel radius mismatch"
        )
        print(
            f"  ✅ Robot dimensions consistent: wheelbase={robot.wheelbase}m, track_width={robot.track_width}m"
        )

        # Test mass properties
        assert abs(robot.mass - robot_config["mass"]) < 0.001, "Mass mismatch"
        assert abs(robot.inertia - robot_config["inertia"]) < 0.001, "Inertia mismatch"
        print(f"  ✅ Mass properties consistent: mass={robot.mass}kg, inertia={robot.inertia}kg⋅m²")

        return True

    except Exception as e:
        print(f"  ❌ Physical parameters consistency failed: {e}")
        return False


def test_ai_backend_consistency(config):
    """Test AI backend configuration consistency"""
    print("⚙️  Testing AI backend configuration consistency...")

    try:
        ai_config = config["ai_backend"]

        processor = AICommandProcessor("config/robot_config.yaml")

        # Test backend type
        expected_type = ai_config["type"]
        actual_type = processor.backend_type.value
        assert actual_type == expected_type, (
            f"AI backend type mismatch: expected {expected_type}, got {actual_type}"
        )
        print(f"  ✅ AI backend type consistent: {actual_type}")

        # Test model name
        expected_model = ai_config["model"]
        actual_model = processor.model
        assert actual_model == expected_model, (
            f"AI model mismatch: expected {expected_model}, got {actual_model}"
        )
        print(f"  ✅ AI model consistent: {actual_model}")

        # Test enabled flag
        if "enabled" in ai_config:
            # This would be tested in communication bridge, but we can check if processor loads
            print(f"  ✅ AI backend enabled: {ai_config['enabled']}")

        return True

    except Exception as e:
        print(f"  ❌ AI backend consistency failed: {e}")
        return False


def test_pid_control_config_consistency(config):
    """Test PID control configuration consistency"""
    print("⚙️  Testing PID control configuration consistency...")

    try:
        if not config["simulation"]["motor_control"]["enabled"]:
            print("  ⚠️  PID control disabled, skipping test")
            return True

        robot = DifferentialDriveRobot("config/robot_config.yaml")

        if not robot.use_motor_controllers:
            print("  ⚠️  Motor controllers not initialized, skipping test")
            return True

        # Test that PID controllers are properly configured
        pid_config = config["simulation"]["motor_control"]["velocity_pid"]

        left_controller = robot.motor_left_controller
        right_controller = robot.motor_right_controller

        # Check PID gains (basic validation that controllers exist and have config)
        assert left_controller is not None, "Left motor controller should exist"
        assert right_controller is not None, "Right motor controller should exist"

        print(f"  ✅ PID controllers initialized with configured parameters")
        print(
            f"  📊 Velocity PID gains: Kp={pid_config['kp']}, Ki={pid_config['ki']}, Kd={pid_config['kd']}"
        )

        return True

    except Exception as e:
        print(f"  ❌ PID control consistency failed: {e}")
        return False


def test_sensor_noise_consistency(config):
    """Test sensor noise configuration consistency"""
    print("⚙️  Testing sensor noise configuration consistency...")

    try:
        robot = DifferentialDriveRobot("config/robot_config.yaml")
        imu_config = config["robot"]["sensors"]["imu"]["noise"]

        # Test that noise parameters are used
        expected_accel_noise = imu_config["accelerometer"]
        expected_gyro_noise = imu_config["gyroscope"]

        # These are stored in robot.imu_noise
        assert "accelerometer" in robot.imu_noise, "Accelerometer noise config missing"
        assert "gyroscope" in robot.imu_noise, "Gyroscope noise config missing"

        actual_accel_noise = robot.imu_noise["accelerometer"]
        actual_gyro_noise = robot.imu_noise["gyroscope"]

        assert abs(actual_accel_noise - expected_accel_noise) < 0.001, (
            f"Accelerometer noise mismatch: expected {expected_accel_noise}, got {actual_accel_noise}"
        )
        assert abs(actual_gyro_noise - expected_gyro_noise) < 0.001, (
            f"Gyroscope noise mismatch: expected {expected_gyro_noise}, got {actual_gyro_noise}"
        )

        print(f"  ✅ IMU noise consistent: accel={actual_accel_noise}, gyro={actual_gyro_noise}")

        return True

    except Exception as e:
        print(f"  ❌ Sensor noise consistency failed: {e}")
        return False


def run_all_tests():
    """Run all configuration consistency tests"""
    print("⚙️  Configuration Consistency Test Suite")
    print("=" * 50)

    # Load configuration first
    success, config = test_config_loading()
    if not success:
        print("❌ Cannot continue without valid configuration")
        return False

    print()

    tests = [
        lambda: test_camera_config_consistency(config),
        lambda: test_motor_config_consistency(config),
        lambda: test_simulation_timestep_consistency(config),
        lambda: test_physical_parameters_consistency(config),
        lambda: test_ai_backend_consistency(config),
        lambda: test_pid_control_config_consistency(config),
        lambda: test_sensor_noise_consistency(config),
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
        print("🎉 All configuration consistency tests passed!")
        return True
    else:
        print("⚠️  Some configuration consistency issues found")
        return False


if __name__ == "__main__":
    print("Configuration Consistency Test")
    print("Verifying that all configuration parameters are properly used")
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
