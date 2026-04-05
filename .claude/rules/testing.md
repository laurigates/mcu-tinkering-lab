# Testing Requirements

## Python Simulation Tests

```bash
cd packages/esp32-projects/robocar-simulation
uv run pytest tests/ --cov
```

- Maintain test coverage for simulation code
- Run tests before committing Python changes

## C/C++ Firmware

- Static analysis via `cppcheck` (run in CI)
- Format checking via `clang-format --dry-run`
- Host-based unit tests planned (not yet implemented)

## CI Verification

All PRs must pass:
- ESP32 firmware builds (matrix across projects)
- Python test suite with coverage
- Pre-commit hooks (format, lint, secret scanning)
