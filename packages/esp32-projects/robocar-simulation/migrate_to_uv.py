#!/usr/bin/env python3
"""
Migration script to help transition from pip/requirements.txt to uv/pyproject.toml

This script helps existing users migrate their environment to use uv.
"""

import shutil
import subprocess
import sys
from pathlib import Path


def run_command(cmd, cwd=None, check=True):
    """Run a command and return the result"""
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, check=check, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout)
    if result.stderr and result.stderr.strip():
        print(f"Warning: {result.stderr.strip()}")
    return result


def main():
    """Migrate from pip to uv"""
    simulation_dir = Path(__file__).parent
    print(f"Migrating ESP32 Robot Simulation to uv in: {simulation_dir}")

    # Check current environment
    print("\n=== Checking Current Environment ===")

    # Check if we're in a virtual environment
    in_venv = hasattr(sys, "real_prefix") or (
        hasattr(sys, "base_prefix") and sys.base_prefix != sys.prefix
    )
    if in_venv:
        print(f"Currently in virtual environment: {sys.prefix}")
        print("Recommendation: Exit current virtual environment before proceeding")
        choice = input("Continue anyway? (y/N): ").lower()
        if choice != "y":
            print("Exiting. Please deactivate your current virtual environment and run again.")
            sys.exit(0)

    # Check for existing virtual environments
    old_venvs = []
    for venv_name in ["venv", "env", ".env", "virtualenv"]:
        venv_path = simulation_dir / venv_name
        if venv_path.exists():
            old_venvs.append(venv_path)

    if old_venvs:
        print(f"\nFound existing virtual environments: {[str(p) for p in old_venvs]}")
        print("These will not be modified, but you can remove them after migration")

    # Check if uv is installed
    try:
        run_command(["uv", "--version"])
        print("✓ uv is installed")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ uv is not installed. Installing uv...")
        try:
            # Try to install uv
            run_command(["curl", "-LsSf", "https://astral.sh/uv/install.sh"])
            run_command(["sh"])
        except Exception:
            print("Failed to install uv automatically. Please install manually:")
            print("curl -LsSf https://astral.sh/uv/install.sh | sh")
            sys.exit(1)

    # Check for pyproject.toml
    pyproject_path = simulation_dir / "pyproject.toml"
    if not pyproject_path.exists():
        print("❌ pyproject.toml not found. Please ensure it exists.")
        sys.exit(1)

    print("✓ pyproject.toml found")

    # Create .python-version if it doesn't exist
    python_version_path = simulation_dir / ".python-version"
    if not python_version_path.exists():
        with open(python_version_path, "w") as f:
            f.write("3.11\n")
        print("✓ Created .python-version file")

    # Setup uv environment
    print("\n=== Setting up uv environment ===")

    try:
        # Remove existing .venv if it exists
        uv_venv_path = simulation_dir / ".venv"
        if uv_venv_path.exists():
            print("Removing existing .venv directory...")
            shutil.rmtree(uv_venv_path)

        # Create new virtual environment
        run_command(["uv", "venv", "--python", "3.11"], cwd=simulation_dir)
        print("✓ Created new virtual environment with Python 3.11")

        # Install dependencies
        run_command(["uv", "sync"], cwd=simulation_dir)
        print("✓ Installed core dependencies")

        # Install development dependencies
        run_command(["uv", "sync", "--extra", "dev"], cwd=simulation_dir)
        print("✓ Installed development dependencies")

        print("\n=== Migration Complete! ===")
        print("\n🎉 Successfully migrated to uv!")
        print("\nNext steps:")
        print("1. Activate the new environment:")
        print("   source .venv/bin/activate")
        print("\n2. Test the setup:")
        print("   uv run python src/main.py --demo-only")
        print("\n3. Run tests:")
        print("   uv run pytest")
        print("\n4. Optional cleanup:")
        if old_venvs:
            print(f"   Remove old virtual environments: {[str(p) for p in old_venvs]}")

        requirements_txt = simulation_dir / "requirements.txt"
        if requirements_txt.exists():
            print("   Archive or remove requirements.txt (dependencies now in pyproject.toml)")

        print("\n5. New workflow commands:")
        print("   uv add <package>           # Add new dependency")
        print("   uv add --dev <package>     # Add dev dependency")
        print("   uv run <command>           # Run command in environment")
        print("   uv sync                    # Update dependencies")

    except subprocess.CalledProcessError as e:
        print(f"❌ Error during migration: {e}")
        print("\nTroubleshooting:")
        print("1. Ensure Python 3.11 is available on your system")
        print("2. Check that uv is properly installed and in PATH")
        print("3. Verify pyproject.toml is valid")
        sys.exit(1)


if __name__ == "__main__":
    main()
