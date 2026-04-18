---
name: sim
description: Run robocar physics simulation environment
argument-hint: "[options]"
allowed-tools: Bash(python:*), Bash(cd:*), Bash(uv:*), Read
---

# Robocar Physics Simulation

Run the robocar physics simulation: $ARGUMENTS

## Steps
1. Navigate to simulation directory: `packages/robocar/simulation/`
2. Check if virtual environment exists, create if needed using `uv`
3. Install dependencies if needed: `uv sync`
4. Run the simulation: `uv run python src/main.py` or the appropriate entry point

## Additional Arguments

Pass any additional arguments to the simulation: $ARGUMENTS

For example:
- `--headless` for headless mode
- `--record` to record session
- `--scenario <name>` to load specific scenario

Report any errors or missing dependencies clearly.
