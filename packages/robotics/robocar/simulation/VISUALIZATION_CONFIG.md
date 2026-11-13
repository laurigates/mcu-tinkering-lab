# 3D Visualization Configuration

The viewer modes of the legacy demo (`robocar_sim.main`, see the
[README](README.md)). The demo needs the `viz` extra: run the commands below
from this directory after `uv sync --extra viz`, or use the
`just sim::run`, `run-visual` and `run-browser` recipes, which pass
`--extra viz` themselves. The plant (`robocar_sim.plant`) has no viewer; its
demo draws a plot of the path (`just sim::plant-demo`).

## Visualization Modes

### 1. Headless Mode (Default)
- **Description**: No visual interface, runs simulation in background
- **Usage**: `uv run python -m robocar_sim.main --headless` or `uv run python -m robocar_sim.main --viz-mode headless`
- **Best for**: Server environments, performance testing, continuous integration

### 2. Visual Mode (GUI Window)
- **Description**: Opens a GUI window with 3D visualization
- **Usage**: `uv run python -m robocar_sim.main --visual` or `uv run python -m robocar_sim.main --viz-mode visual`
- **Best for**: Local development, interactive debugging

### 3. Browser Mode (Web Interface)
- **Description**: Opens browser-based 3D visualization at http://localhost:52000
- **Usage**: `uv run python -m robocar_sim.main --browser` or `uv run python -m robocar_sim.main --viz-mode browser`
- **Best for**: Remote access, sharing visualization, web integration

## Configuration Methods

### Command Line Arguments
```bash
# Explicit mode selection
uv run python -m robocar_sim.main --viz-mode headless
uv run python -m robocar_sim.main --viz-mode visual
uv run python -m robocar_sim.main --viz-mode browser

# Shorthand flags
uv run python -m robocar_sim.main --headless     # or --no-viz
uv run python -m robocar_sim.main --visual       # or --gui
uv run python -m robocar_sim.main --browser

# Combined with other options
uv run python -m robocar_sim.main --browser --config custom_config.yaml
```

### Environment Variable
```bash
# Set visualization mode via environment
export GENESIS_VIZ_MODE=visual
uv run python -m robocar_sim.main

# Or inline
GENESIS_VIZ_MODE=browser uv run python -m robocar_sim.main
```

### Priority Order
1. Command line `--viz-mode` argument (highest priority)
2. Command line shorthand flags (`--visual`, `--browser`, `--headless`)
3. `GENESIS_VIZ_MODE` environment variable
4. Default: `headless` mode

## Examples

```bash
# Default headless mode
uv run python -m robocar_sim.main

# GUI window visualization
uv run python -m robocar_sim.main --visual

# Browser-based visualization
uv run python -m robocar_sim.main --browser

# Environment variable control
GENESIS_VIZ_MODE=visual uv run python -m robocar_sim.main

```

## Troubleshooting

### Visual Mode Issues
- **GUI not opening**: Try browser mode instead: `--browser`
- **Display errors**: Ensure X11 forwarding (SSH) or local display access
- **Performance issues**: Use headless mode: `--headless`

### Browser Mode Issues
- **Port conflicts**: Check if localhost:52000 is available
- **Browser not opening**: Manually navigate to http://localhost:52000
- **Connection errors**: Try visual mode instead: `--visual`

### Fallback Behavior
The system automatically falls back to headless mode if the requested visualization mode fails to initialize.
