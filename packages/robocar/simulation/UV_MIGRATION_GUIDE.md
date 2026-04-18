# UV Migration Guide

This document describes the migration from pip/requirements.txt to uv/pyproject.toml for the ESP32 Robot Simulation project.

## What Changed

### 🆕 New Files
- `pyproject.toml` - Modern Python project configuration with dependencies
- `.python-version` - Specifies Python 3.11 requirement
- `setup_uv.py` - One-command setup script
- `migrate_to_uv.py` - Migration helper for existing users
- `.gitignore` - Updated to handle uv files and caches
- `tests/test_uv_setup.py` - Verification tests for uv setup

### 📝 Updated Files
- `README.md` - Updated installation and usage instructions for uv
- All command examples now use `uv run` prefix

### 🗑️ Deprecated Files
- `requirements.txt` - Dependencies now in pyproject.toml (keep for reference)

## Key Benefits of UV

### ⚡ Performance
- **10-100x faster** than pip for dependency resolution
- **Parallel downloads** and installations
- **Smart caching** reduces repeated downloads

### 🛡️ Reliability
- **Lock file** ensures reproducible installations
- **Conflict resolution** prevents dependency issues
- **Virtual environment isolation** built-in

### 🔧 Developer Experience
- **Single tool** for project management
- **Integrated commands** (run, add, sync)
- **Modern pyproject.toml** configuration

## Migration Steps

### For New Users
```bash
cd robocar/simulation
python setup_uv.py
```

### For Existing Users
```bash
cd robocar/simulation
python migrate_to_uv.py
```

### Manual Migration
```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Create virtual environment
uv venv --python 3.11

# Install dependencies
uv sync --extra dev
```

## New Workflow Commands

### Basic Usage
```bash
# Run simulation
uv run python src/main.py

# Run tests
uv run pytest

# Run with coverage
uv run pytest --cov=src
```

### Development
```bash
# Add new dependency
uv add requests

# Add development dependency
uv add --dev black

# Update dependencies
uv sync

# Check for outdated packages
uv list --outdated
```

### Code Quality
```bash
# Lint code
uv run ruff check .

# Format code
uv run ruff format .

# Type checking
uv run mypy .

# Run all quality checks
uv run ruff check . && uv run ruff format . && uv run mypy . && uv run pytest
```

## Configuration Details

### pyproject.toml Structure
```toml
[project]
name = "esp32-robot-simulation"
dependencies = [
    "genesis-world>=0.2.0",  # Replaces robotics-toolbox-python
    "torch>=2.0.0",          # Required by Genesis
    # ... other core dependencies
]

[project.optional-dependencies]
dev = ["pytest", "ruff", "mypy"]     # Development tools
gpu = ["torch[cuda]>=2.0.0"]         # GPU acceleration
```

### Tool Configuration
- **ruff**: Fast linting and formatting (replaces black, isort, flake8)
- **mypy**: Type checking configuration
- **pytest**: Test discovery and execution
- **coverage**: Test coverage reporting

## Compatibility Notes

### Python Version
- **Required**: Python 3.11+
- **Tested**: Python 3.11, 3.12
- **Genesis compatibility**: Requires modern Python features

### Dependencies
- **Genesis**: Replaces Swift-sim (better Python 3.12+ support)
- **PyTorch**: Now explicit dependency (required by Genesis)
- **Ruff**: Modern linting/formatting tool

### Environment Variables
```bash
# UV respects these environment variables
UV_PYTHON=3.11              # Force Python version
UV_CACHE_DIR=~/.cache/uv    # Cache location
UV_INDEX_URL=...            # Custom package index
```

## Troubleshooting

### Common Issues

**uv not found**:
```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc  # or restart terminal
```

**Python 3.11 not found**:
```bash
# macOS
brew install python@3.11

# Ubuntu/Debian
sudo apt install python3.11 python3.11-venv

# Verify
python3.11 --version
```

**Genesis import error**:
```bash
# Verify installation
uv run python -c "import genesis; print('Genesis OK')"

# Reinstall if needed
uv sync --reinstall
```

**Permission errors (Linux)**:
```bash
# Fix serial port permissions
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Performance Optimization

**Slow dependency resolution**:
```bash
# Use UV cache
export UV_CACHE_DIR=~/.cache/uv

# Parallel installs (default)
export UV_CONCURRENT_INSTALLS=10
```

**Large download sizes**:
```bash
# CPU-only PyTorch (smaller)
uv add "torch>=2.0.0+cpu" --index-url https://download.pytorch.org/whl/cpu

# Or use requirements in pyproject.toml:
# torch = {version = ">=2.0.0", source = "pytorch-cpu"}
```

## Testing the Migration

### Verification Steps
```bash
# 1. Test imports
uv run python -c "import genesis, torch, cv2; print('All imports OK')"

# 2. Run basic tests
uv run pytest tests/test_uv_setup.py -v

# 3. Test simulation startup
uv run python src/main.py --demo-only

# 4. Check code quality tools
uv run ruff check .
uv run mypy .
```

### Expected Output
- ✅ All tests pass
- ✅ No linting errors
- ✅ Type checking succeeds
- ✅ Simulation starts without errors

## Rollback Procedure

If you need to rollback to the old setup:

```bash
# 1. Deactivate uv environment
deactivate

# 2. Remove uv files
rm -rf .venv uv.lock

# 3. Restore old environment
python3.11 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Support

### Getting Help
1. **Check logs**: Look for error messages in terminal output
2. **Test environment**: Run `uv run pytest tests/test_uv_setup.py`
3. **Verify tools**: Check `uv --version`, `python --version`
4. **Clean install**: Remove `.venv` and run setup again

### Reporting Issues
Include this information:
- Operating system and version
- Python version (`python --version`)
- UV version (`uv --version`)
- Error messages and full traceback
- Output of `uv run pytest tests/test_uv_setup.py -v`

## Future Improvements

### Planned Features
- **Pre-commit hooks**: Automatic linting on git commit
- **CI/CD integration**: GitHub Actions with uv
- **Docker support**: Containerized development environment
- **Documentation**: Auto-generated API docs with Sphinx

### Contributing
When adding new dependencies:
1. Use `uv add <package>` for runtime dependencies
2. Use `uv add --dev <package>` for development tools
3. Update tests to verify new functionality
4. Document any new requirements or setup steps
