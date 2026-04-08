"""Shared test fixtures for robocar simulation tests."""

import sys
from pathlib import Path

import pytest

# Add src directory to path so all test modules can import project code
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from robot_model import DifferentialDriveRobot  # noqa: E402


@pytest.fixture
def robot():
    """Create a DifferentialDriveRobot for testing using the default config.

    Subsystems (WiFi, OTA, camera) are NOT started because
    DifferentialDriveRobot no longer activates them in __init__.
    """
    config_path = Path(__file__).resolve().parent.parent / "config" / "robot_config.yaml"
    robot = DifferentialDriveRobot(str(config_path))
    yield robot
    robot.reset()
