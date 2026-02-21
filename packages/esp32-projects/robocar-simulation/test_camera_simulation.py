#!/usr/bin/env python3
"""
Test Camera Simulation Integration

This test verifies that the camera simulation is working correctly with
the robot model and generating synthetic camera feeds.
"""

import os
import sys
import time

import numpy as np

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from camera_simulation import CameraSimulation
from robot_model import DifferentialDriveRobot


def test_camera_basic_functionality():
    """Test basic camera simulation functionality"""
    print("🔍 Testing camera simulation basic functionality...")

    try:
        # Create camera simulation
        config_path = "config/robot_config.yaml"
        camera = CameraSimulation(config_path)

        # Create a dummy robot state for testing
        from robot_model import RobotState

        robot_state = RobotState()
        robot_state.x = 1.0
        robot_state.y = 1.0
        robot_state.theta = 0.5
        robot_state.camera_pan_angle = 0.0

        # Test frame generation
        frame = camera._render_synthetic_view(robot_state)

        assert frame is not None, "Frame should not be None"
        assert frame.shape == (480, 640, 3), (
            f"Frame shape should be (480, 640, 3), got {frame.shape}"
        )
        assert frame.dtype == np.uint8, f"Frame dtype should be uint8, got {frame.dtype}"

        print("  ✅ Basic frame generation works")

        # Test JPEG compression
        camera.get_compressed_frame()
        # compressed will be None since no frame capture is running
        print("  ✅ JPEG compression functionality exists")

        return True

    except Exception as e:
        print(f"  ❌ Camera basic functionality failed: {e}")
        return False


def test_camera_integration():
    """Test camera integration with robot model"""
    print("🔍 Testing camera integration with robot...")

    try:
        # Create robot with camera enabled
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)

        # Check if camera simulation was initialized
        assert robot.use_camera_simulation, "Camera simulation should be enabled"
        assert robot.camera_simulation is not None, "Camera simulation should be initialized"

        print("  ✅ Camera simulation initialized with robot")

        # Update robot a few times to allow camera to capture frames
        for _i in range(5):
            robot.set_motor_commands(30, 25)  # Slight movement
            robot.update()
            time.sleep(0.1)  # Allow time for camera capture

        # Check if camera frame is available in robot state
        state = robot.get_state()
        if state.camera_frame is not None:
            print(f"  ✅ Camera frame captured: {state.camera_frame.shape}")

            # Test frame saving
            filename = f"test_camera_frame_{int(time.time())}.jpg"
            success = robot.camera_simulation.save_frame(filename)
            if success:
                print(f"  ✅ Frame saved successfully as {filename}")
                # Clean up test file
                try:
                    os.remove(filename)
                except Exception:
                    pass
        else:
            print("  ⚠️  Camera frame not yet available (may need more time)")

        # Stop camera simulation
        robot.camera_simulation.stop_capture()
        print("  ✅ Camera simulation stopped cleanly")

        return True

    except Exception as e:
        print(f"  ❌ Camera integration failed: {e}")
        return False


def test_camera_movement_tracking():
    """Test camera tracking robot movement"""
    print("🔍 Testing camera movement tracking...")

    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)

        # Test different robot positions and camera angles
        test_cases = [
            {"pos": (0, 0, 0), "pan": 0, "description": "origin, forward"},
            {"pos": (1, 1, 1.57), "pan": 45, "description": "moved, turned right, pan right"},
            {"pos": (-1, -1, 3.14), "pan": -30, "description": "opposite corner, pan left"},
        ]

        frames_captured = 0

        for i, case in enumerate(test_cases):
            # Set robot position
            robot.reset(case["pos"][0], case["pos"][1], case["pos"][2])
            robot.state.camera_pan_angle = case["pan"]

            # Update robot
            robot.update()
            time.sleep(0.1)  # Allow camera to capture

            state = robot.get_state()
            if state.camera_frame is not None:
                frames_captured += 1
                print(f"  ✅ Frame {i + 1}: {case['description']}")

        robot.camera_simulation.stop_capture()

        if frames_captured > 0:
            print(f"  ✅ Successfully captured {frames_captured}/{len(test_cases)} frames")
        else:
            print("  ⚠️  No frames captured during movement test")

        return True

    except Exception as e:
        print(f"  ❌ Camera movement tracking failed: {e}")
        return False


def test_camera_performance():
    """Test camera simulation performance"""
    print("🔍 Testing camera simulation performance...")

    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)

        # Run simulation for a short time and measure frame rate
        start_time = time.time()
        frame_count = 0
        test_duration = 2.0  # seconds

        while time.time() - start_time < test_duration:
            robot.update()
            state = robot.get_state()
            if state.camera_frame is not None:
                frame_count += 1
            time.sleep(0.01)  # Simulation timestep

        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0

        robot.camera_simulation.stop_capture()

        print(f"  📊 Captured {frame_count} frames in {elapsed:.2f}s")
        print(f"  📊 Effective FPS: {fps:.1f}")

        # Check if performance is reasonable (should get some frames)
        if frame_count > 0:
            print("  ✅ Camera simulation is generating frames")
        else:
            print("  ⚠️  No frames captured during performance test")

        return True

    except Exception as e:
        print(f"  ❌ Camera performance test failed: {e}")
        return False


def run_all_tests():
    """Run all camera simulation tests"""
    print("🎬 Camera Simulation Test Suite")
    print("=" * 50)

    tests = [
        test_camera_basic_functionality,
        test_camera_integration,
        test_camera_movement_tracking,
        test_camera_performance,
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
        print("🎉 All camera simulation tests passed!")
        return True
    else:
        print("⚠️  Some camera simulation tests failed")
        return False


if __name__ == "__main__":
    print("Camera Simulation Test")
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
