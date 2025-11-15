#!/usr/bin/env python3
"""
Simple Swift Test Script

This script tests Swift visualization modes independently to identify
issues before integrating with the main simulation.
"""

import asyncio
import sys
import time
import threading
from pathlib import Path

try:
    import roboticstoolbox as rtb
    import swift
    import trimesh
    from spatialmath import SE3
    from swift import Swift

    print("✓ All Swift dependencies imported successfully")
except ImportError as e:
    print(f"✗ Import error: {e}")
    sys.exit(1)


def test_browser_mode():
    """Test Swift browser mode independently"""
    print("\n=== Testing Swift Browser Mode ===")

    async def browser_test():
        try:
            # Create Swift instance
            env = Swift()
            print("✓ Swift instance created")

            # Launch in browser mode
            print("Launching Swift in browser mode...")
            env.launch(realtime=True, headless=False, browser="default")
            print("✓ Swift browser mode launched")
            print("✓ Check http://localhost:52000 in your browser")

            # Add a simple test object
            test_box = trimesh.primitives.Box(extents=[0.1, 0.1, 0.1])
            test_box.visual.face_colors = [255, 0, 0, 255]  # Red
            env.add(test_box)
            print("✓ Test object added")

            # Run for 10 seconds
            print("Running for 10 seconds...")
            start_time = time.time()
            while time.time() - start_time < 10.0:
                # Rotate the box
                angle = (time.time() - start_time) * 2.0  # 2 rad/s
                transform = SE3.Rz(angle).A
                test_box.apply_transform(
                    test_box.transform
                    @ transform
                    @ SE3.Rz(-angle / 2).A  # Counter-rotate to prevent drift
                )
                env.step(0.1)
                await asyncio.sleep(0.1)

            print("✓ Browser mode test completed successfully")
            return True

        except Exception as e:
            print(f"✗ Browser mode test failed: {e}")
            import traceback

            traceback.print_exc()
            return False

    # Run the async test
    try:
        result = asyncio.run(browser_test())
        return result
    except Exception as e:
        print(f"✗ Asyncio error in browser mode: {e}")
        return False


def test_visual_mode():
    """Test Swift visual (GUI) mode independently"""
    print("\n=== Testing Swift Visual/GUI Mode ===")

    async def visual_test():
        try:
            # Create Swift instance
            env = Swift()
            print("✓ Swift instance created")

            # Launch in visual mode
            print("Launching Swift in visual mode...")
            env.launch(realtime=True, headless=False)
            print("✓ Swift visual mode launched")
            print("✓ GUI window should appear")

            # Add a simple test object
            test_cylinder = trimesh.primitives.Cylinder(radius=0.05, height=0.2)
            test_cylinder.visual.face_colors = [0, 255, 0, 255]  # Green
            env.add(test_cylinder)
            print("✓ Test object added")

            # Run for 10 seconds
            print("Running for 10 seconds...")
            start_time = time.time()
            while time.time() - start_time < 10.0:
                # Move the cylinder up and down
                z = 0.1 + 0.05 * (1 + time.sin((time.time() - start_time) * 3.0))
                transform = SE3(0, 0, z).A
                test_cylinder.apply_transform(SE3.inv(test_cylinder.transform) @ transform)
                env.step(0.1)
                await asyncio.sleep(0.1)

            print("✓ Visual mode test completed successfully")
            return True

        except Exception as e:
            print(f"✗ Visual mode test failed: {e}")
            import traceback

            traceback.print_exc()
            return False

    # Run the async test
    try:
        result = asyncio.run(visual_test())
        return result
    except Exception as e:
        print(f"✗ Asyncio error in visual mode: {e}")
        return False


def test_headless_mode():
    """Test Swift headless mode (should always work)"""
    print("\n=== Testing Swift Headless Mode ===")

    async def headless_test():
        try:
            # Create Swift instance
            env = Swift()
            print("✓ Swift instance created")

            # Launch in headless mode
            print("Launching Swift in headless mode...")
            env.launch(realtime=True, headless=True)
            print("✓ Swift headless mode launched")

            # Add a simple test object
            test_sphere = trimesh.primitives.Sphere(radius=0.05)
            test_sphere.visual.face_colors = [0, 0, 255, 255]  # Blue
            env.add(test_sphere)
            print("✓ Test object added")

            # Run for 5 seconds
            print("Running for 5 seconds...")
            for i in range(50):  # 5 seconds at 10 Hz
                env.step(0.1)
                await asyncio.sleep(0.1)

            print("✓ Headless mode test completed successfully")
            return True

        except Exception as e:
            print(f"✗ Headless mode test failed: {e}")
            import traceback

            traceback.print_exc()
            return False

    # Run the async test
    try:
        result = asyncio.run(headless_test())
        return result
    except Exception as e:
        print(f"✗ Asyncio error in headless mode: {e}")
        return False


def main():
    """Run all Swift tests"""
    print("Swift Visualization Test Suite")
    print("=" * 50)

    results = {"headless": False, "browser": False, "visual": False}

    # Test headless mode first (should always work)
    results["headless"] = test_headless_mode()

    # Test browser mode
    if results["headless"]:
        print("\nHeadless mode works, testing browser mode...")
        results["browser"] = test_browser_mode()
    else:
        print("\nSkipping browser test due to headless failure")

    # Test visual mode
    if results["headless"]:
        print("\nHeadless mode works, testing visual mode...")
        results["visual"] = test_visual_mode()
    else:
        print("\nSkipping visual test due to headless failure")

    # Summary
    print("\n" + "=" * 50)
    print("TEST RESULTS SUMMARY:")
    print("=" * 50)

    for mode, success in results.items():
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"{mode.upper():>10}: {status}")

    if results["browser"] and results["visual"]:
        print("\n🎉 All visualization modes working!")
        print("The simulation GUI and browser issues should now be resolved.")
    elif results["headless"]:
        print("\n⚠️  Headless mode works, but GUI modes have issues.")
        print("Check display settings and dependencies.")
    else:
        print("\n❌ Swift has fundamental issues.")
        print("Check installation: pip install --upgrade swift-sim robotics-toolbox-python")

    return all(results.values())


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
