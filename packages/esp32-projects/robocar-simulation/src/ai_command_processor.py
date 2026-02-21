"""
AI Command Processor

This module provides AI-powered command processing for the robot simulation,
supporting both Claude and Ollama backends for natural language robot control.
"""

import asyncio
import base64
import json
import logging
import os
import re
import time
import traceback
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import cv2
import httpx
import numpy as np
import yaml

logger = logging.getLogger(__name__)


class AIBackendType(Enum):
    """Supported AI backend types"""

    CLAUDE = "claude"
    OLLAMA = "ollama"


@dataclass
class AICommand:
    """Structured AI command output"""

    action: str  # move, turn, stop, pan_camera, etc.
    parameters: dict[str, Any]  # Action-specific parameters
    confidence: float  # AI confidence in command (0-1)
    reasoning: str  # AI explanation of decision
    timestamp: float  # When command was generated


@dataclass
class RobotContext:
    """Robot context for AI processing"""

    position: tuple[float, float, float]  # x, y, theta
    velocity: tuple[float, float]  # linear, angular
    sensor_data: dict[str, Any]  # Ultrasonic, IMU, etc.
    camera_frame: np.ndarray | None  # Current camera image
    encoder_positions: tuple[float, float]  # Left, right encoder positions
    last_commands: list[str]  # Recent command history


class AICommandProcessor:
    """
    AI-powered command processor for robot control

    Supports natural language commands via Claude API or local Ollama
    """

    def __init__(self, config_path: str):
        self.config = self._load_config(config_path)
        self.ai_config = self.config.get("ai_backend", {})

        # Backend configuration
        self.backend_type = AIBackendType(self.ai_config.get("type", "claude"))
        self.api_key = os.getenv("CLAUDE_API_KEY") or self.ai_config.get("api_key", "")
        self.model = self.ai_config.get("model", "claude-3-sonnet-20240229")

        # Ollama configuration
        self.ollama_host = self.ai_config.get("ollama", {}).get("host", "localhost")
        self.ollama_port = self.ai_config.get("ollama", {}).get("port", 11434)
        self.ollama_model = self.ai_config.get("ollama", {}).get("model", "llava")

        # HTTP client for API calls
        self.http_client = httpx.AsyncClient(timeout=30.0)

        # Command history
        self.command_history: list[AICommand] = []
        self.max_history = 10

        # System prompt for AI
        self.system_prompt = self._create_system_prompt()

    def _load_config(self, config_path: str) -> dict[str, Any]:
        """Load configuration from YAML file"""
        config_file = Path(config_path)
        if not config_file.exists():
            config_file = Path(__file__).parent.parent / "config" / "robot_config.yaml"

        with open(config_file) as f:
            return yaml.safe_load(f)

    def _create_system_prompt(self) -> str:
        """Create system prompt for AI backend"""
        return """You are an AI assistant controlling a differential drive robot car with the following capabilities:

ROBOT HARDWARE:
- Differential drive with left/right motors (PWM control -255 to 255)
- ESP32-CAM with pan servo (angle -90 to +90 degrees)
- Ultrasonic distance sensor (range 0.02-4.0 meters)
- IMU with accelerometer and gyroscope
- Wheel encoders for position feedback

AVAILABLE ACTIONS:
1. "move" - Set motor PWM values
   Parameters: {"left_pwm": int, "right_pwm": int, "duration_ms": int}

2. "move_velocity" - Set wheel velocities (if PID enabled)
   Parameters: {"left_velocity": float, "right_velocity": float, "duration_ms": int}

3. "turn" - Turn robot by angle
   Parameters: {"angle_degrees": float, "speed": float}

4. "forward" - Move forward
   Parameters: {"distance_meters": float, "speed": float}

5. "backward" - Move backward
   Parameters: {"distance_meters": float, "speed": float}

6. "stop" - Stop all movement
   Parameters: {}

7. "pan_camera" - Pan camera servo
   Parameters: {"angle_degrees": float}

8. "scan_area" - Scan area with camera and ultrasonic
   Parameters: {"start_angle": float, "end_angle": float, "step_size": float}

9. "follow_wall" - Follow wall using ultrasonic
   Parameters: {"target_distance": float, "speed": float}

10. "explore" - Autonomous exploration
    Parameters: {"duration_seconds": float}

RESPONSE FORMAT:
You must respond with a JSON object containing:
{
  "action": "action_name",
  "parameters": {...},
  "confidence": 0.8,
  "reasoning": "Explanation of why you chose this action"
}

GUIDELINES:
- Keep movements safe and controlled
- Use camera and sensors to assess environment
- Explain your reasoning clearly
- If unsure, ask for clarification or choose safe defaults
- Consider robot's current state when planning actions
"""

    def _encode_image_base64(self, image: np.ndarray) -> str:
        """Encode OpenCV image to base64 string"""
        if image is None:
            return ""

        # Encode as JPEG
        _, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Convert to base64
        image_base64 = base64.b64encode(buffer).decode("utf-8")
        return image_base64

    async def process_command(self, user_input: str, robot_context: RobotContext) -> AICommand:
        """
        Process natural language command using AI backend

        Args:
            user_input: Natural language command from user
            robot_context: Current robot state and sensor data

        Returns:
            Structured AI command
        """
        try:
            if self.backend_type == AIBackendType.CLAUDE:
                return await self._process_with_claude(user_input, robot_context)
            elif self.backend_type == AIBackendType.OLLAMA:
                return await self._process_with_ollama(user_input, robot_context)
            else:
                raise ValueError(f"Unsupported AI backend: {self.backend_type}")

        except Exception as e:
            logger.error(
                "AI processing failed for backend %s: %s\n%s",
                self.backend_type,
                e,
                traceback.format_exc(),
            )
            # Return safe default command
            return AICommand(
                action="stop",
                parameters={},
                confidence=0.0,
                reasoning=f"AI processing failed: {e}",
                timestamp=time.time(),
            )

    async def _process_with_claude(self, user_input: str, robot_context: RobotContext) -> AICommand:
        """Process command using Claude API"""
        # Prepare context information
        context_text = self._format_robot_context(robot_context)

        # Prepare messages
        messages = [
            {"role": "system", "content": self.system_prompt},
            {
                "role": "user",
                "content": f"Current robot state:\n{context_text}\n\nUser command: {user_input}\n\nPlease respond with a JSON action command.",
            },
        ]

        # Add image if available
        if robot_context.camera_frame is not None:
            image_base64 = self._encode_image_base64(robot_context.camera_frame)
            if image_base64:
                messages[-1]["content"] += "\n\nCurrent camera view is attached as image."
                # Note: Claude API image handling would need proper implementation

        # Make API call
        headers = {
            "Content-Type": "application/json",
            "x-api-key": self.api_key,
            "anthropic-version": "2023-06-01",
        }

        payload = {"model": self.model, "max_tokens": 1000, "messages": messages}

        response = await self.http_client.post(
            "https://api.anthropic.com/v1/messages", headers=headers, json=payload
        )

        if response.status_code != 200:
            raise Exception(f"Claude API error: {response.status_code} - {response.text}")

        result = response.json()
        ai_response = result["content"][0]["text"]

        # Parse JSON response
        command = self._parse_ai_response(ai_response)
        command.timestamp = time.time()

        return command

    async def _process_with_ollama(self, user_input: str, robot_context: RobotContext) -> AICommand:
        """Process command using local Ollama"""
        # Prepare context
        context_text = self._format_robot_context(robot_context)

        # Prepare prompt
        prompt = f"{self.system_prompt}\n\nCurrent robot state:\n{context_text}\n\nUser command: {user_input}\n\nPlease respond with a JSON action command."

        # Prepare request
        payload = {
            "model": self.ollama_model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": 0.7,
                "top_p": 0.9,
            },
        }

        # Add image if available (for vision models like llava)
        if robot_context.camera_frame is not None and "llava" in self.ollama_model.lower():
            image_base64 = self._encode_image_base64(robot_context.camera_frame)
            if image_base64:
                payload["images"] = [image_base64]

        # Make API call to Ollama
        ollama_url = f"http://{self.ollama_host}:{self.ollama_port}/api/generate"

        response = await self.http_client.post(ollama_url, json=payload)

        if response.status_code != 200:
            raise Exception(f"Ollama API error: {response.status_code} - {response.text}")

        result = response.json()
        ai_response = result.get("response", "")

        # Parse JSON response
        command = self._parse_ai_response(ai_response)
        command.timestamp = time.time()

        return command

    def _format_robot_context(self, context: RobotContext) -> str:
        """Format robot context for AI prompt"""
        pos_x, pos_y, theta = context.position
        linear_vel, angular_vel = context.velocity
        left_pos, right_pos = context.encoder_positions

        context_text = f"""Position: ({pos_x:.2f}, {pos_y:.2f}) meters, heading: {np.degrees(theta):.1f}°
Velocity: {linear_vel:.2f} m/s linear, {np.degrees(angular_vel):.1f}°/s angular
Encoder positions: left={left_pos:.1f}°, right={right_pos:.1f}°
Ultrasonic distance: {context.sensor_data.get("ultrasonic_distance", 0):.2f} meters
Camera available: {"Yes" if context.camera_frame is not None else "No"}
Recent commands: {", ".join(context.last_commands[-3:]) if context.last_commands else "None"}"""

        return context_text

    def _parse_ai_response(self, response: str) -> AICommand:
        """Parse AI response into structured command"""
        try:
            # Extract JSON from response (AI might include explanatory text)
            json_match = re.search(r"\{.*\}", response, re.DOTALL)
            if json_match:
                json_str = json_match.group()
                command_data = json.loads(json_str)
            else:
                # Try parsing entire response as JSON
                command_data = json.loads(response)

            # Validate required fields
            action = command_data.get("action", "stop")
            parameters = command_data.get("parameters", {})
            confidence = command_data.get("confidence", 0.5)
            reasoning = command_data.get("reasoning", "No reasoning provided")

            # Validate confidence is in range
            try:
                confidence = max(0.0, min(1.0, float(confidence)))
            except (TypeError, ValueError):
                confidence = 0.5

            return AICommand(
                action=action,
                parameters=parameters,
                confidence=confidence,
                reasoning=reasoning,
                timestamp=time.time(),
            )

        except (json.JSONDecodeError, ValueError, KeyError) as e:
            logger.error(
                "Failed to parse AI response: %s\nResponse was: %s\n%s",
                e,
                response,
                traceback.format_exc(),
            )

            return AICommand(
                action="stop",
                parameters={},
                confidence=0.0,
                reasoning=f"Failed to parse AI response: {e}",
                timestamp=time.time(),
            )

    def add_to_history(self, command: AICommand):
        """Add command to history"""
        self.command_history.append(command)
        if len(self.command_history) > self.max_history:
            self.command_history.pop(0)

    def get_command_history(self) -> list[AICommand]:
        """Get command history"""
        return self.command_history.copy()

    async def close(self):
        """Clean up resources"""
        await self.http_client.aclose()


def execute_ai_command(command: AICommand, robot) -> bool:
    """
    Execute AI command on robot

    Args:
        command: AI command to execute
        robot: Robot instance

    Returns:
        True if command was executed successfully
    """
    try:
        action = command.action
        params = command.parameters

        if action == "move":
            left_pwm = params.get("left_pwm", 0)
            right_pwm = params.get("right_pwm", 0)
            robot.set_motor_commands(left_pwm, right_pwm)

        elif action == "move_velocity":
            left_vel = params.get("left_velocity", 0.0)
            right_vel = params.get("right_velocity", 0.0)
            robot.set_velocity_commands(left_vel, right_vel)

        elif action == "stop":
            robot.set_motor_commands(0, 0)

        elif action == "pan_camera":
            angle = params.get("angle_degrees", 0.0)
            robot.state.camera_pan_angle = np.clip(angle, -90, 90)

        elif action == "forward":
            params.get("distance_meters", 1.0)
            speed = params.get("speed", 0.5)
            # Convert to approximate PWM for fixed duration
            pwm = int(speed * 100)
            robot.set_motor_commands(pwm, pwm)

        elif action == "backward":
            params.get("distance_meters", 1.0)
            speed = params.get("speed", 0.5)
            pwm = int(speed * 100)
            robot.set_motor_commands(-pwm, -pwm)

        elif action == "turn":
            angle = params.get("angle_degrees", 90.0)
            speed = params.get("speed", 0.5)
            pwm = int(speed * 100)
            # Positive angle = turn right
            if angle > 0:
                robot.set_motor_commands(pwm, -pwm)
            else:
                robot.set_motor_commands(-pwm, pwm)

        else:
            print(f"Unknown action: {action}")
            return False

        return True

    except Exception as e:
        print(f"Failed to execute command {command.action}: {e}")
        return False


async def main():
    """Test AI command processor"""
    from robot_model import DifferentialDriveRobot

    # Create processor and robot
    config_path = "../config/robot_config.yaml"
    processor = AICommandProcessor(config_path)
    robot = DifferentialDriveRobot(config_path)

    print(f"AI Command Processor Test ({processor.backend_type.value})")
    print("=" * 50)

    # Test commands
    test_commands = [
        "Move forward slowly",
        "Turn right 90 degrees",
        "Stop the robot",
        "Pan camera to look left",
        "Scan the area in front",
    ]

    for user_command in test_commands:
        print(f"\nUser: {user_command}")

        # Get robot context
        state = robot.get_state()
        encoder_pos = robot.get_encoder_positions()

        context = RobotContext(
            position=(state.x, state.y, state.theta),
            velocity=(state.v, state.omega),
            sensor_data={"ultrasonic_distance": state.ultrasonic_distance},
            camera_frame=state.camera_frame,
            encoder_positions=encoder_pos,
            last_commands=[cmd.action for cmd in processor.command_history[-3:]],
        )

        # Process command
        try:
            ai_command = await processor.process_command(user_command, context)

            print(f"AI Action: {ai_command.action}")
            print(f"Parameters: {ai_command.parameters}")
            print(f"Confidence: {ai_command.confidence:.2f}")
            print(f"Reasoning: {ai_command.reasoning}")

            # Execute command
            success = execute_ai_command(ai_command, robot)
            print(f"Execution: {'✅' if success else '❌'}")

            processor.add_to_history(ai_command)

        except Exception as e:
            print(f"❌ Command processing failed: {e}")

        # Update robot
        robot.update()

    await processor.close()


if __name__ == "__main__":
    asyncio.run(main())
