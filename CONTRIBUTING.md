# Contributing to MCU Tinkering Lab

Thank you for your interest in contributing to MCU Tinkering Lab! This document provides guidelines and instructions for contributing to this embedded systems monorepo.

## Table of Contents

- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Code Style Guidelines](#code-style-guidelines)
- [Build Commands](#build-commands)
- [Testing Requirements](#testing-requirements)
- [Commit Message Convention](#commit-message-convention)
- [Pull Request Process](#pull-request-process)
- [Adding New Projects](#adding-new-projects)
- [Documentation Guidelines](#documentation-guidelines)
- [CI/CD](#cicd)
- [Troubleshooting](#troubleshooting)

## Getting Started

### Prerequisites

- **Docker**, or Podman with `CONTAINER_CMD=podman`. ESP-IDF builds run in the
  `espressif/idf:v5.4` container; no local ESP-IDF install is needed.
- **[just](https://github.com/casey/just)**
- **[uv](https://github.com/astral-sh/uv)** and Python 3.11+ for the simulation
  and Python tooling
- **Git** configured with your name and email

### Setup Development Environment

```bash
just setup-all            # Docker images, dev tools, pre-commit hooks
just check-environment    # verify Docker and serial port setup
```

Builds and `menuconfig` run in the container. Flashing and the serial monitor
run on the host, because USB passthrough into containers is unreliable on
macOS. To reach serial devices from inside the container anyway, uncomment the
`devices` and `privileged` entries in `docker-compose.yml`.

### Fork and Clone

1. Fork the repository on GitHub
2. Clone your fork:
   ```bash
   git clone https://github.com/YOUR-USERNAME/mcu-tinkering-lab.git
   cd mcu-tinkering-lab
   ```
3. Add upstream remote:
   ```bash
   git remote add upstream https://github.com/laurigates/mcu-tinkering-lab.git
   ```

## Development Workflow

### 1. Create a Feature Branch

```bash
# Update your main branch
git checkout main
git pull upstream main

# Create a feature branch
git checkout -b feat/your-feature-name

# Or for bug fixes
git checkout -b fix/bug-description
```

### 2. Make Your Changes

- Write clean, readable code
- Follow the code style guidelines (see below)
- Add tests for new functionality
- Update documentation as needed

### 3. Test Your Changes

```bash
# Format code
just format

# Run linters
just lint

# Check formatting (non-destructive)
just format-check

# Build and test the affected projects
just <module>::build
just <module>::test     # where the project has host tests
```

### 4. Commit Your Changes

```bash
# Stage your changes
git add .

# Pre-commit hooks will run automatically
# Commit with conventional commit message
git commit -m "feat: Add WiFi reconnection logic for ESP32"
```

### 5. Push and Create Pull Request

```bash
# Push to your fork
git push origin feat/your-feature-name

# Create PR on GitHub
# Fill out the PR template
```

## Code Style Guidelines

### C/C++ Code Style

We use **clang-format** with Google style (4-space indent, 100 column limit).

**Formatting:**
```bash
# Format all C/C++ files
just format-c

# Check formatting without modifying
just format-check-c
```

**Style Rules:**
- Use 4 spaces for indentation (no tabs)
- Maximum line length: 100 characters
- Braces on same line for functions, separate for control structures
- Pointer alignment: `char *ptr` (pointer on right)
- Use meaningful variable names
- Comment complex logic

**Example:**
```c
#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "MY_MODULE";

// Brief description of function
esp_err_t initialize_wifi(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "Invalid WiFi credentials");
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize WiFi with provided credentials
    esp_err_t ret = esp_wifi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}
```

### Python Code Style

We use **ruff** for linting and formatting (PEP 8 compatible, 100 character line limit).

**Formatting:**
```bash
# Format all Python files
just format-python

# Lint Python code
just lint-python
```

**Style Rules:**
- Follow PEP 8
- Use type hints
- Docstrings for all public functions/classes
- Maximum line length: 100 characters

**Example:**
```python
from typing import Optional

def calculate_motor_speed(duty_cycle: int, max_speed: int = 255) -> int:
    """
    Calculate motor speed based on duty cycle percentage.

    Args:
        duty_cycle: PWM duty cycle percentage (0-100)
        max_speed: Maximum speed value (default: 255)

    Returns:
        Calculated motor speed value

    Raises:
        ValueError: If duty_cycle is out of range
    """
    if not 0 <= duty_cycle <= 100:
        raise ValueError(f"Duty cycle must be 0-100, got {duty_cycle}")

    return int((duty_cycle / 100.0) * max_speed)
```

### Pre-commit Hooks

Pre-commit hooks automatically enforce code quality:

```bash
# Install hooks (one-time setup)
pre-commit install

# Run manually on all files
pre-commit run --all-files
```

**Checks performed:**
- C/C++ formatting (clang-format) and Python formatting and linting (ruff, ty)
- Secret scanning (gitleaks) and credential-file blocking
- Flash recipes against partition tables (`tools/check-flash-recipes.py`)
- Generated wiring tables and build guides against `hardware.toml` and
  `pin_config.h`
- Trailing whitespace, end of file, YAML validity, build artifacts

## Build Commands

Each project is a `just` module. `just list-projects` lists them,
`just --list <module>` shows one project's recipes.

```bash
just <module>::build          # containerized build
just <module>::flash          # flash from the host (PORT=/dev/... overrides detection)
just <module>::monitor        # serial monitor
just <module>::menuconfig     # containerized menuconfig

just build-all                # robocar main + camera only
just clean-all                # clean every project build

just lint                     # cppcheck + ruff
just format                   # clang-format + ruff format
just format-check             # check only, no changes

just docker-dev               # interactive ESP-IDF shell
just docker-clean             # remove containers and volumes
```

## Testing Requirements

- Add tests for new hardware-independent logic.
- Run the affected suites before opening a PR.

| Suite | Command |
|---|---|
| robocar-unified host tests | `just robocar-unified::test` |
| kids-audio-toy host tests | `just kids-audio::test` |
| balancebot host tests | `just balancebot::test` |
| Robocar simulation | `cd packages/robocar/simulation && uv sync && uv run pytest tests/ --cov` |
| All pre-commit hooks | `pre-commit run --all-files` |

### ESP32 Host-Based Tests

Host tests compile hardware-independent firmware modules with the native
compiler and run them without a board. Move the logic into a `*_core.{c,h}`
with no ESP-IDF headers, compile that same file into the firmware and into a
plain-assert test `main()`, and expose it as a `just test` recipe.
`packages/audio/kids-audio-toy` is the worked example; the full pattern is in
`.claude/rules/testing.md`.

Host tests cover cases a bench cannot stage, such as the 32-bit millisecond
counter wrapping at day 49.

## Commit Message Convention

We follow **Conventional Commits** for clear, semantic versioning-compatible commit messages.

### Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Types

- **feat**: New feature
- **fix**: Bug fix
- **docs**: Documentation only changes
- **style**: Code style changes (formatting, no logic change)
- **refactor**: Code refactoring (no feature change, no bug fix)
- **perf**: Performance improvements
- **test**: Adding or updating tests
- **chore**: Build process or auxiliary tool changes
- **ci**: CI/CD pipeline changes

### Examples

```bash
# Feature
git commit -m "feat(robocar-main): Add WiFi reconnection logic"

# Bug fix
git commit -m "fix(robocar-camera): Fix memory leak in image capture"

# Documentation
git commit -m "docs(readme): Update Docker setup instructions"

# Breaking change
git commit -m "feat(i2c)!: Change I2C protocol format

BREAKING CHANGE: I2C protocol now uses CRC8 instead of CRC16"
```

### Scope

Optional scope to specify which part of the codebase is affected:
- `robocar-main`, `robocar-camera`, `robocar-simulation`
- `esp32-webserver`, `llm-telegram`
- `makefile`, `ci`, `docker`
- `tests`, `docs`

## Pull Request Process

### 1. Before Creating PR

- ✅ All tests pass
- ✅ Code is formatted (`just format`)
- ✅ Linters pass (`just lint`)
- ✅ Documentation updated
- ✅ Commit messages follow convention
- ✅ Branch is up-to-date with main

### 2. PR Title and Description

**Title format:**
```
<type>(<scope>): <description>
```

**Description template:**
```markdown
## Summary
Brief description of changes

## Changes
- Change 1
- Change 2
- Change 3

## Testing
- [ ] Manual testing performed
- [ ] Unit tests added/updated
- [ ] CI pipeline passes

## Screenshots (if applicable)
[Add screenshots here]

## Breaking Changes
[Describe any breaking changes]

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Documentation updated
- [ ] No new warnings introduced
```

### 3. Code Review

- Address all review comments
- Be responsive and respectful
- Make requested changes in new commits (don't force-push during review)
- Request re-review after changes

### 4. Merging

- Squash commits if requested by maintainers
- Ensure CI passes
- Wait for maintainer approval
- Maintainer will merge the PR

## Adding New Projects

### Using the Scaffolding Tool

```bash
# Create new ESP32 project
./tools/scaffold/new-esp32-project.sh

# Follow the prompts
```

### Manual Project Creation

1. **Create project directory:**
   ```bash
   mkdir -p packages/<domain>/my-new-project
   cd packages/<domain>/my-new-project
   ```

2. **Create CMakeLists.txt:**
   ```cmake
   cmake_minimum_required(VERSION 3.5)
   include($ENV{IDF_PATH}/tools/cmake/project.cmake)
   project(my-new-project)
   ```

3. **Create main component:**
   ```bash
   mkdir -p main
   # Create main/CMakeLists.txt and main/main.c
   ```

4. **Add to CI pipeline:**
   Add an entry to `.github/project-matrix.json` with your project's `system` (`esp32`), `project`, `path`, and `target` (plus `fetch_bluepad32: true` if it vendors bluepad32). The single `build.yml` workflow discovers it automatically and builds it on push/PR whenever its files change — no per-project workflow file needed.

5. **Register the module in the root justfile:**
   `mod <name> 'packages/<domain>/<name>'`, then run
   `python3 tools/check-flash-recipes.py` to check the flash recipe against the
   partition table.

6. **Enable web flasher (optional):**
   To include your project in the [Web Flasher](https://laurigates.github.io/mcu-tinkering-lab/),
   add a `flasher.json` file to your project root (see [#152](https://github.com/laurigates/mcu-tinkering-lab/issues/152) for the planned convention):
   ```json
   {
     "name": "My New Project",
     "description": "Brief description of the project",
     "chipFamily": "ESP32",
     "board": "ESP32-DevKitC"
   }
   ```

## Documentation Guidelines

### Project Documentation

Every project should have a `README.md` with:

- **Title and brief description**
- **Features list**
- **Hardware requirements**
- **Building instructions**
- **Flashing instructions**
- **Configuration guide**
- **License information**

### Code Documentation

**C/C++ Comments:**
```c
/**
 * @brief Initialize the motor control system
 *
 * @param motor_count Number of motors to initialize
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_init(uint8_t motor_count);
```

**Python Docstrings:**
```python
def process_image(img: np.ndarray, threshold: int = 128) -> np.ndarray:
    """
    Process image with threshold filter.

    Args:
        img: Input image as numpy array
        threshold: Threshold value (0-255)

    Returns:
        Processed image
    """
    pass
```

### Architecture Documentation

For significant architectural changes, update relevant documentation in `docs/` or project-specific `README.md`.

## CI/CD

| Workflow | Purpose |
|---|---|
| `build.yml` | Builds every changed firmware project (ESP-IDF, ESPHome, Pico SDK), discovered from `.github/project-matrix.json` |
| `test.yml` | Pre-commit, pytest, cppcheck, format check |
| `build-firmware.yml` | On release: builds firmware, attaches binaries, deploys the web flasher, and fetches every published file to verify it |
| `release-please.yml` | Release PRs from conventional commits |
| `hardware-check.yml`, `build-guide-check.yml`, `schematics-check.yml` | Generated docs and schematics match their sources |
| `refresh-idf-locks.yml` | Monthly refresh of tracked `dependencies.lock` files |

## Troubleshooting

**Serial port permission denied (Linux).** Add the user to `dialout` and log in
again: `sudo usermod -a -G dialout $USER`.

**Port not detected.** `just list-devices` lists connected boards and USB-serial
adapters. Set `PORT=/dev/...` explicitly when more than one is connected.

**Out of disk space.** `just clean-all`, then `just docker-clean`.

## Questions or Issues?

- **Questions:** Open a [Discussion](https://github.com/laurigates/mcu-tinkering-lab/discussions)
- **Bug Reports:** Open an [Issue](https://github.com/laurigates/mcu-tinkering-lab/issues)
- **Feature Requests:** Open an [Issue](https://github.com/laurigates/mcu-tinkering-lab/issues) with the `enhancement` label
