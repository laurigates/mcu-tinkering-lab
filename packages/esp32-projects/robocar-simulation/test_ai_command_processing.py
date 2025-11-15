#!/usr/bin/env python3
"""
Test AI Command Processing Integration

This test verifies that the AI command processing system is working correctly
with both Claude and Ollama backends, and can handle natural language commands.
"""

import sys
import os
import time
import asyncio
import json
import numpy as np

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from robot_model import DifferentialDriveRobot
from ai_command_processor import AICommandProcessor, RobotContext, execute_ai_command
from communication_bridge import ESP32CommunicationBridge


def test_ai_processor_initialization():
    """Test AI processor initialization"""
    print("🧠 Testing AI processor initialization...")

    try:
        config_path = "config/robot_config.yaml"
        processor = AICommandProcessor(config_path)

        assert processor.backend_type is not None, "Backend type should be set"
        assert processor.system_prompt is not None, "System prompt should be created"
        assert processor.command_history == [], "Command history should be empty initially"

        print(f"  ✅ Backend type: {processor.backend_type.value}")
        print(f"  ✅ Model: {processor.model}")
        print(f"  ✅ System prompt created ({len(processor.system_prompt)} chars)")

        return True

    except Exception as e:
        print(f"  ❌ AI processor initialization failed: {e}")
        return False


def test_command_parsing():
    """Test AI response parsing"""
    print("🧠 Testing command parsing...")

    try:
        config_path = "config/robot_config.yaml"
        processor = AICommandProcessor(config_path)

        # Test valid JSON response
        valid_response = """
        {
          "action": "move",
          "parameters": {"left_pwm": 100, "right_pwm": 100},
          "confidence": 0.9,
          "reasoning": "Moving forward as requested"
        }
        """

        command = processor._parse_ai_response(valid_response)

        assert command.action == "move", f"Action should be 'move', got '{command.action}'"
        assert command.parameters["left_pwm"] == 100, "Left PWM should be 100"
        assert command.confidence == 0.9, f"Confidence should be 0.9, got {command.confidence}"
        assert "Moving forward" in command.reasoning, "Reasoning should contain explanation"

        print("  ✅ Valid JSON parsing works")

        # Test JSON embedded in text
        text_response = """
        I'll help you move the robot forward. Here's my command:
        
        {
          "action": "forward",
          "parameters": {"distance_meters": 1.0, "speed": 0.5},
          "confidence": 0.8,
          "reasoning": "Moving forward 1 meter at moderate speed"
        }
        
        This should move the robot safely forward.
        """

        command2 = processor._parse_ai_response(text_response)
        assert command2.action == "forward", "Should extract JSON from text"
        print("  ✅ JSON extraction from text works")

        # Test invalid response handling
        invalid_response = "This is not JSON at all!"
        command3 = processor._parse_ai_response(invalid_response)
        assert command3.action == "stop", "Should default to stop for invalid JSON"
        assert command3.confidence == 0.0, "Should have zero confidence for invalid response"
        print("  ✅ Invalid response handling works")

        return True

    except Exception as e:
        print(f"  ❌ Command parsing failed: {e}")
        return False


def test_command_execution():
    """Test AI command execution on robot"""
    print("🧠 Testing command execution...")

    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)

        # Test basic movement command
        from ai_command_processor import AICommand

        move_command = AICommand(
            action="move",
            parameters={"left_pwm": 150, "right_pwm": 100},
            confidence=0.9,
            reasoning="Test movement",
            timestamp=time.time(),
        )

        success = execute_ai_command(move_command, robot)
        assert success, "Move command should execute successfully"

        state = robot.get_state()
        assert state.motor_left_pwm == 150, "Left motor PWM should be set"
        assert state.motor_right_pwm == 100, "Right motor PWM should be set"

        print("  ✅ Move command execution works")

        # Test stop command
        stop_command = AICommand(
            action="stop",
            parameters={},
            confidence=1.0,
            reasoning="Emergency stop",
            timestamp=time.time(),
        )

        success = execute_ai_command(stop_command, robot)
        assert success, "Stop command should execute successfully"

        state = robot.get_state()
        assert state.motor_left_pwm == 0, "Left motor should be stopped"
        assert state.motor_right_pwm == 0, "Right motor should be stopped"

        print("  ✅ Stop command execution works")

        # Test camera pan command
        pan_command = AICommand(
            action="pan_camera",
            parameters={"angle_degrees": 45.0},
            confidence=0.8,
            reasoning="Look right",
            timestamp=time.time(),
        )

        success = execute_ai_command(pan_command, robot)
        assert success, "Pan command should execute successfully"

        state = robot.get_state()
        assert state.camera_pan_angle == 45.0, "Camera should be panned to 45 degrees"

        print("  ✅ Camera pan command execution works")

        return True

    except Exception as e:
        print(f"  ❌ Command execution failed: {e}")
        return False


def test_robot_context_creation():
    """Test robot context creation for AI processing"""
    print("🧠 Testing robot context creation...")

    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)

        # Move robot to create some state
        robot.set_motor_commands(100, 80)
        for _ in range(10):  # Update multiple times to allow state to change
            robot.update()

        # Manually set position for testing (simulating robot movement)
        robot.state.x = 1.5
        robot.state.y = 2.0
        robot.state.theta = 0.785  # 45 degrees
        robot.state.camera_pan_angle = 30.0

        # Get robot state and create context
        state = robot.get_state()
        encoder_pos = robot.get_encoder_positions()

        context = RobotContext(
            position=(state.x, state.y, state.theta),
            velocity=(state.v, state.omega),
            sensor_data={
                "ultrasonic_distance": state.ultrasonic_distance,
                "imu_accel": state.imu_accel.tolist(),
                "imu_gyro": state.imu_gyro.tolist(),
            },
            camera_frame=state.camera_frame,
            encoder_positions=encoder_pos,
            last_commands=["move", "turn"],
        )

        # Verify context
        assert context.position[0] == 1.5, "X position should be set"
        assert context.position[1] == 2.0, "Y position should be set"
        assert abs(context.position[2] - 0.785) < 0.1, "Theta should be approximately 45 degrees"
        assert len(context.sensor_data) > 0, "Sensor data should be populated"
        assert len(context.last_commands) == 2, "Command history should be included"

        print("  ✅ Robot context creation works")
        print(
            f"  📊 Position: ({context.position[0]:.1f}, {context.position[1]:.1f}, {np.degrees(context.position[2]):.1f}°)"
        )
        print(f"  📊 Ultrasonic: {context.sensor_data['ultrasonic_distance']:.2f}m")
        print(f"  📊 Last commands: {context.last_commands}")

        return True

    except Exception as e:
        print(f"  ❌ Robot context creation failed: {e}")
        return False


async def test_mock_ai_processing():
    """Test AI processing with mock responses (without actual API calls)"""
    print("🧠 Testing mock AI processing...")

    try:
        config_path = "config/robot_config.yaml"
        processor = AICommandProcessor(config_path)
        robot = DifferentialDriveRobot(config_path)

        # Create robot context
        state = robot.get_state()
        encoder_pos = robot.get_encoder_positions()

        context = RobotContext(
            position=(state.x, state.y, state.theta),
            velocity=(state.v, state.omega),
            sensor_data={"ultrasonic_distance": state.ultrasonic_distance},
            camera_frame=state.camera_frame,
            encoder_positions=encoder_pos,
            last_commands=[],
        )

        # Test context formatting
        context_text = processor._format_robot_context(context)
        assert "Position:" in context_text, "Context should include position"
        assert "Velocity:" in context_text, "Context should include velocity"
        assert "Ultrasonic" in context_text, "Context should include sensor data"

        print("  ✅ Context formatting works")
        print(f"  📊 Context preview: {context_text[:100]}...")

        # Test parsing mock AI responses
        mock_responses = [
            '{"action": "move", "parameters": {"left_pwm": 120, "right_pwm": 120}, "confidence": 0.9, "reasoning": "Moving forward"}',
            'I want to turn the robot left: {"action": "turn", "parameters": {"angle_degrees": -90, "speed": 0.5}, "confidence": 0.8, "reasoning": "Turning left 90 degrees"}',
            '{"action": "stop", "parameters": {}, "confidence": 1.0, "reasoning": "Emergency stop requested"}',
        ]

        for i, mock_response in enumerate(mock_responses):
            command = processor._parse_ai_response(mock_response)
            processor.add_to_history(command)

            print(
                f"  ✅ Mock response {i + 1}: {command.action} (confidence: {command.confidence:.1f})"
            )

        # Check command history
        history = processor.get_command_history()
        assert len(history) == 3, "Should have 3 commands in history"

        print("  ✅ Command history works")

        await processor.close()

        return True

    except Exception as e:
        print(f"  ❌ Mock AI processing failed: {e}")
        return False


def test_integration_with_communication_bridge():
    """Test AI integration with communication bridge"""
    print("🧠 Testing communication bridge integration...")

    try:
        config_path = "config/robot_config.yaml"
        robot = DifferentialDriveRobot(config_path)
        bridge = ESP32CommunicationBridge(config_path, robot)

        # Check if AI processor is initialized
        assert bridge.use_ai_processing, "AI processing should be enabled"
        assert bridge.ai_processor is not None, "AI processor should be initialized"

        print("  ✅ AI processor integrated with bridge")
        print(f"  📊 Backend type: {bridge.ai_processor.backend_type.value}")

        # Test AI command handler exists
        from communication_bridge import MessageType

        assert MessageType.AI_COMMAND in bridge.message_handlers, (
            "AI command handler should be registered"
        )

        print("  ✅ AI command handler registered")

        bridge.stop()

        return True

    except Exception as e:
        print(f"  ❌ Communication bridge integration failed: {e}")
        return False


async def run_all_tests():
    """Run all AI command processing tests"""
    print("🧠 AI Command Processing Test Suite")
    print("=" * 50)

    tests = [
        test_ai_processor_initialization,
        test_command_parsing,
        test_command_execution,
        test_robot_context_creation,
        test_mock_ai_processing,
        test_integration_with_communication_bridge,
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        try:
            if asyncio.iscoroutinefunction(test):
                result = await test()
            else:
                result = test()

            if result:
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
        print("🎉 All AI command processing tests passed!")
        return True
    else:
        print("⚠️  Some AI command processing tests failed")
        return False


if __name__ == "__main__":
    print("AI Command Processing Test")
    print("Note: This test runs in mock mode (no actual API calls)")
    print("Press Ctrl+C to stop at any time")

    try:
        success = asyncio.run(run_all_tests())
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 Tests interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 Test suite crashed: {e}")
        sys.exit(1)
