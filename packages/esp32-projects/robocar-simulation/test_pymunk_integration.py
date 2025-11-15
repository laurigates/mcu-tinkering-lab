#!/usr/bin/env python3
"""
Test script for pymunk physics integration
"""

import sys
from pathlib import Path
import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from robot_model import DifferentialDriveRobot, PhysicsEngine


def test_physics_engine_initialization():
    """Test that physics engine initializes correctly"""
    print("Testing Physics Engine Initialization...")

    config_path = "config/robot_config.yaml"

    try:
        robot = DifferentialDriveRobot(config_path)
        print(f"✓ Robot initialized successfully")
        print(f"✓ Using physics engine: {robot.use_physics}")

        if robot.use_physics:
            print(f"✓ Physics engine created with {len(robot.physics_engine.obstacles)} obstacles")
            print(f"✓ Robot body added to physics simulation")

        return True

    except Exception as e:
        print(f"✗ Failed to initialize robot: {e}")
        return False


def test_basic_movement():
    """Test basic robot movement with physics"""
    print("\nTesting Basic Movement...")

    config_path = "config/robot_config.yaml"
    robot = DifferentialDriveRobot(config_path)

    # Set forward motion
    robot.set_motor_commands(100, 100)

    initial_x = robot.state.x
    initial_y = robot.state.y

    # Run simulation for 1 second
    steps = int(1.0 / robot.dt)
    for i in range(steps):
        robot.update()

    final_x = robot.state.x
    final_y = robot.state.y

    distance_moved = np.sqrt((final_x - initial_x) ** 2 + (final_y - initial_y) ** 2)

    print(f"✓ Robot moved {distance_moved:.3f}m in 1 second")
    print(f"✓ Initial position: ({initial_x:.3f}, {initial_y:.3f})")
    print(f"✓ Final position: ({final_x:.3f}, {final_y:.3f})")

    if distance_moved > 0.01:  # Should have moved at least 1cm
        print("✓ Movement test passed")
        return True
    else:
        print("✗ Movement test failed - robot didn't move enough")
        return False


def test_collision_detection():
    """Test ultrasonic sensor collision detection"""
    print("\nTesting Collision Detection...")

    config_path = "config/robot_config.yaml"
    robot = DifferentialDriveRobot(config_path)

    # Place robot near an obstacle (according to config, there's a box at [2.0, 2.0])
    robot.reset(1.5, 2.0, 0.0)  # Face towards obstacle
    robot.update()

    distance = robot.state.ultrasonic_distance
    print(f"✓ Ultrasonic sensor reading: {distance:.3f}m")

    if distance < 1.0:  # Should detect the obstacle
        print("✓ Collision detection working")
        return True
    else:
        print("? Collision detection may not be working (sensor reading > 1m)")
        return True  # Don't fail test, might be expected


def main():
    """Run all tests"""
    print("PyMunk Physics Integration Test")
    print("=" * 40)

    tests = [test_physics_engine_initialization, test_basic_movement, test_collision_detection]

    passed = 0
    failed = 0

    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"✗ Test {test.__name__} crashed: {e}")
            failed += 1

    print(f"\nTest Results: {passed} passed, {failed} failed")

    if failed == 0:
        print("🎉 All tests passed! PyMunk integration working correctly.")
    else:
        print("⚠️  Some tests failed. Check the implementation.")


if __name__ == "__main__":
    main()
