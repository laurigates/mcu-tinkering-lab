---
allowed-tools: Bash(python:*), Bash(cd:*), Bash(uv:*), Read
description: Run robocar physics simulation environment
---

## Context
Simulation directory contents:
!`ls packages/esp32-projects/robocar-simulation/`

## Task
Run the robocar physics simulation.

### Steps
1. Navigate to simulation directory: `packages/esp32-projects/robocar-simulation/`
2. Check if virtual environment exists, create if needed using `uv`
3. Install dependencies if needed: `uv pip install -r requirements.txt`
4. Run the simulation: `python src/main.py` or the appropriate entry point

### Additional Arguments
Pass any additional arguments to the simulation: $ARGUMENTS

For example:
- `--headless` for headless mode
- `--record` to record session
- `--scenario <name>` to load specific scenario

Report any errors or missing dependencies clearly.
