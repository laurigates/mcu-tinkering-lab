"""Shared test fixtures for robocar simulation tests."""

from pathlib import Path

import pytest

from robocar_sim.robot_model import DifferentialDriveRobot


@pytest.fixture
def robot():
    """Create a DifferentialDriveRobot for testing using the default config.

    The camera is not started: DifferentialDriveRobot activates it only
    through enable_camera().
    """
    config_path = Path(__file__).resolve().parent.parent / "config" / "robot_config.yaml"
    robot = DifferentialDriveRobot(str(config_path))
    yield robot
    robot.reset()
