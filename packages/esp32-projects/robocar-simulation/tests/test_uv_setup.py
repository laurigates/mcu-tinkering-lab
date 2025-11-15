#!/usr/bin/env python3
"""
Test file to verify uv setup and basic imports work correctly.
"""

import pytest
import sys
import asyncio
from pathlib import Path

# Add src directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def test_python_version():
    """Test that we're running Python 3.11+"""
    assert sys.version_info >= (3, 11), f"Python 3.11+ required, got {sys.version}"


def test_basic_imports():
    """Test that core Python libraries can be imported"""
    import numpy as np
    import yaml
    import asyncio
    import json

    assert np.__version__
    print(f"✓ NumPy {np.__version__}")


def test_simulation_imports():
    """Test that simulation-specific libraries can be imported"""
    try:
        import cv2

        print(f"✓ OpenCV {cv2.__version__}")
    except ImportError:
        pytest.skip("OpenCV not available")

    try:
        import websockets

        print(f"✓ WebSockets {websockets.__version__}")
    except ImportError:
        pytest.skip("WebSockets not available")

    try:
        import serial

        print(f"✓ PySerial {serial.__version__}")
    except ImportError:
        pytest.skip("PySerial not available")


def test_genesis_import():
    """Test that Genesis framework can be imported"""
    try:
        import genesis

        print(f"✓ Genesis available")
    except ImportError:
        pytest.skip("Genesis not available - this is expected if not installed yet")


def test_torch_import():
    """Test that PyTorch can be imported"""
    try:
        import torch

        print(f"✓ PyTorch {torch.__version__}")
        if torch.cuda.is_available():
            print(f"✓ CUDA available: {torch.cuda.get_device_name()}")
        else:
            print("ℹ CPU-only PyTorch (no CUDA)")
    except ImportError:
        pytest.skip("PyTorch not available - this is expected if not installed yet")


def test_robot_model_import():
    """Test that our robot model can be imported"""
    try:
        from robot_model import DifferentialDriveRobot

        print("✓ Robot model imports successfully")
    except ImportError as e:
        pytest.skip(f"Robot model not available: {e}")


def test_config_file_exists():
    """Test that configuration file exists"""
    config_path = Path(__file__).parent.parent / "config" / "robot_config.yaml"
    assert config_path.exists(), f"Configuration file not found: {config_path}"

    import yaml

    with open(config_path) as f:
        config = yaml.safe_load(f)

    assert "robot" in config, "Robot configuration section missing"
    print("✓ Configuration file valid")


@pytest.mark.asyncio
async def test_asyncio_functionality():
    """Test that asyncio works correctly"""

    async def test_coroutine():
        await asyncio.sleep(0.001)
        return "success"

    result = await test_coroutine()
    assert result == "success"
    print("✓ Asyncio functionality working")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
