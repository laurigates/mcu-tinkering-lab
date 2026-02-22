#!/usr/bin/env python3
"""
Setup script to initialize uv environment for ESP32 Robot Simulation
"""

import subprocess
import sys
from pathlib import Path


def run_command(cmd, cwd=None, check=True):
    """Run a command and return the result"""
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, check=check, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr)
    return result


def main():
    """Initialize uv environment and install dependencies"""
    simulation_dir = Path(__file__).parent
    print(f"Setting up uv environment in: {simulation_dir}")

    # Check if uv is installed
    try:
        run_command(["uv", "--version"])
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: uv is not installed. Please install uv first:")
        print("curl -LsSf https://astral.sh/uv/install.sh | sh")
        sys.exit(1)

    # Initialize uv project (this will create .venv and install dependencies)
    print("\n=== Initializing uv project ===")
    try:
        # Create virtual environment with Python 3.11
        run_command(["uv", "venv", "--python", "3.11"], cwd=simulation_dir)

        # Sync dependencies from pyproject.toml
        run_command(["uv", "sync"], cwd=simulation_dir)

        # Install development dependencies
        run_command(["uv", "sync", "--extra", "dev"], cwd=simulation_dir)

        print("\n=== Setup Complete! ===")
        print("To activate the virtual environment:")
        print("  source .venv/bin/activate")
        print("\nTo run the simulation:")
        print("  uv run python src/main.py")
        print("\nTo run tests:")
        print("  uv run pytest")
        print("\nTo run linting:")
        print("  uv run ruff check .")
        print("  uv run ruff format .")

    except subprocess.CalledProcessError as e:
        print(f"Error during setup: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
