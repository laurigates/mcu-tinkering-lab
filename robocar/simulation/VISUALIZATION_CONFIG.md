# 3D Visualization Configuration

The ESP32 Robot Car Simulation now supports configurable 3D visualization modes instead of requiring code changes.

## Visualization Modes

### 1. Headless Mode (Default)
- **Description**: No visual interface, runs simulation in background
- **Usage**: `python main.py --headless` or `python main.py --viz-mode headless`
- **Best for**: Server environments, performance testing, continuous integration

### 2. Visual Mode (GUI Window)
- **Description**: Opens a GUI window with 3D visualization
- **Usage**: `python main.py --visual` or `python main.py --viz-mode visual`  
- **Best for**: Local development, interactive debugging

### 3. Browser Mode (Web Interface)
- **Description**: Opens browser-based 3D visualization at http://localhost:52000
- **Usage**: `python main.py --browser` or `python main.py --viz-mode browser`
- **Best for**: Remote access, sharing visualization, web integration

## Configuration Methods

### Command Line Arguments
```bash
# Explicit mode selection
python main.py --viz-mode headless
python main.py --viz-mode visual  
python main.py --viz-mode browser

# Shorthand flags
python main.py --headless     # or --no-viz
python main.py --visual       # or --gui
python main.py --browser

# Combined with other options
python main.py --visual --serial /dev/ttyUSB0
python main.py --browser --config custom_config.yaml
```

### Environment Variable
```bash
# Set visualization mode via environment
export SWIFT_VIZ_MODE=visual
python main.py

# Or inline
SWIFT_VIZ_MODE=browser python main.py
```

### Priority Order
1. Command line `--viz-mode` argument (highest priority)
2. Command line shorthand flags (`--visual`, `--browser`, `--headless`)  
3. `SWIFT_VIZ_MODE` environment variable
4. Default: `headless` mode

## Examples

```bash
# Default headless mode
python main.py

# GUI window visualization
python main.py --visual

# Browser-based visualization
python main.py --browser

# Environment variable control
SWIFT_VIZ_MODE=visual python main.py

# Combined with ESP32 connection
python main.py --visual --serial /dev/ttyUSB0

# Demo mode with browser visualization
python main.py --browser --demo-only
```

## Migration from Previous Version

**Before** (required code edit):
```python
# Edit swift_visualizer.py line 432:
self.env.launch(realtime=True, headless=False)  # Manual edit needed
```

**After** (configurable):
```bash
python main.py --visual  # No code changes needed
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