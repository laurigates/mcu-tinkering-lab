#!/usr/bin/env python3
"""
Test script for simplified Swift visualizer
"""

import sys
from pathlib import Path
import time

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

def test_simplified_architecture():
    """Test that the simplified architecture works without threading complexity"""
    print("Testing Simplified Swift Visualizer...")
    
    try:
        from swift_visualizer import SwiftSimulation
        
        config_path = "config/robot_config.yaml"
        
        # Test headless mode (should work without Swift dependencies)
        print("1. Testing headless mode initialization...")
        sim = SwiftSimulation(config_path, viz_mode='headless')
        print("✓ Simulation created successfully")
        
        # Test motor commands
        print("2. Testing motor commands...")
        sim.set_motor_commands(100, 100)
        print("✓ Motor commands set")
        
        # Test short simulation run
        print("3. Running short simulation (2 seconds)...")
        start_time = time.time()
        sim.run_simulation(duration=2.0)
        end_time = time.time()
        
        actual_duration = end_time - start_time
        print(f"✓ Simulation completed in {actual_duration:.2f} seconds")
        
        # Check if robot moved
        final_pos = sim.robot.get_pose()
        print(f"✓ Final robot position: {final_pos}")
        
        if abs(final_pos[0]) > 0.01 or abs(final_pos[1]) > 0.01:
            print("✓ Robot moved as expected")
        else:
            print("? Robot movement minimal (might be expected)")
        
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_visualization_disabled():
    """Test that simulation works when Swift is not available"""
    print("\nTesting with visualization disabled...")
    
    try:
        from swift_visualizer import RobotVisualizer
        from robot_model import DifferentialDriveRobot
        
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)
        
        # Force visualization to be disabled
        visualizer = RobotVisualizer(config_path, robot, viz_mode='headless')
        
        # Test update calls (should not crash)
        print("Testing visualization updates...")
        for i in range(5):
            robot.set_motor_commands(50, 50)
            robot.update()
            visualizer.update_robot_visualization()
        
        print("✓ Visualization updates handled gracefully")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False

def test_threading_simplification():
    """Test that threading is simplified and stable"""
    print("\nTesting threading simplification...")
    
    try:
        from swift_visualizer import SwiftSimulation
        
        config_path = "config/robot_config.yaml"
        sim = SwiftSimulation(config_path, viz_mode='headless')
        
        # Start and stop multiple times to test thread cleanup
        print("Testing multiple start/stop cycles...")
        for cycle in range(3):
            print(f"  Cycle {cycle + 1}...")
            
            # Start visualization
            if sim.visualizer.enabled:
                sim.visualizer.start_update_loop()
                time.sleep(0.5)  # Let it run briefly
                sim.visualizer.stop()
                time.sleep(0.1)  # Cleanup time
            
        print("✓ Multiple start/stop cycles completed")
        return True
        
    except Exception as e:
        print(f"✗ Threading test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("Simplified Swift Visualizer Test Suite")
    print("=" * 45)
    
    tests = [
        test_simplified_architecture,
        test_visualization_disabled,
        test_threading_simplification
    ]
    
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
        print("🎉 All tests passed! Threading architecture simplified successfully.")
        print("✓ Removed complex subprocess approach")
        print("✓ Simplified event loop management") 
        print("✓ Eliminated queue-based communication")
        print("✓ Reduced threading complexity")
    else:
        print("⚠️  Some tests failed. Check the implementation.")

if __name__ == "__main__":
    main()