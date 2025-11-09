# Contributing to MCU Tinkering Lab

Thank you for your interest in contributing to MCU Tinkering Lab! This document provides guidelines and instructions for contributing to this embedded systems monorepo.

## Table of Contents

- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Code Style Guidelines](#code-style-guidelines)
- [Testing Requirements](#testing-requirements)
- [Commit Message Convention](#commit-message-convention)
- [Pull Request Process](#pull-request-process)
- [Adding New Projects](#adding-new-projects)
- [Documentation Guidelines](#documentation-guidelines)

## Getting Started

### Prerequisites

Before you begin, ensure you have:

- **ESP-IDF v5.4+** installed (for ESP32 projects)
- **Python 3.11+** with `pip` and `uv`
- **Docker** (optional but recommended)
- **Git** configured with your name and email
- **Code editor** (VS Code, CLion, or your preference)

### Setup Development Environment

#### Option 1: Docker (Recommended)

```bash
# Clone the repository
git clone https://github.com/laurigates/mcu-tinkering-lab.git
cd mcu-tinkering-lab

# Build Docker images
make docker-build

# Start development shell
make docker-dev
```

#### Option 2: Native Setup

```bash
# Install development tools
make install-dev-tools

# This will install:
# - pre-commit hooks
# - Python tools (ruff, mypy, pytest, uv)
# - Instructions for clang-format and cppcheck
```

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
make format

# Run linters
make lint

# Check formatting (non-destructive)
make format-check

# Build affected projects
make build-all

# Run tests (when available)
make test-all
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
make format-c

# Check formatting without modifying
make format-check-c
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
make format-python

# Lint Python code
make lint-python
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
- ✅ C/C++ formatting (clang-format)
- ✅ Python formatting (ruff)
- ✅ Python linting (ruff)
- ✅ Trailing whitespace removal
- ✅ End-of-file fixing
- ✅ YAML validation
- ✅ Credential file detection
- ✅ Build artifact detection

## Testing Requirements

### Unit Tests (When Adding New Code)

- Write unit tests for new functions
- Aim for >70% code coverage
- Test edge cases and error conditions

### ESP32 Host-Based Tests (Future)

```c
// Example test structure
#include "unity.h"
#include "motor_control.h"

void test_motor_speed_calculation(void) {
    TEST_ASSERT_EQUAL(128, calculate_motor_speed(50, 255));
    TEST_ASSERT_EQUAL(255, calculate_motor_speed(100, 255));
    TEST_ASSERT_EQUAL(0, calculate_motor_speed(0, 255));
}

void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_motor_speed_calculation);
    UNITY_END();
}
```

### Python Tests

```python
import pytest
from robot_model import RobotModel

def test_robot_initialization():
    robot = RobotModel(width=0.15, height=0.20)
    assert robot.width == 0.15
    assert robot.height == 0.20

def test_motor_command_invalid_range():
    robot = RobotModel()
    with pytest.raises(ValueError):
        robot.set_motor_speed(-10, 50)
```

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
- ✅ Code is formatted (`make format`)
- ✅ Linters pass (`make lint`)
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
   mkdir -p packages/esp32-projects/my-new-project
   cd packages/esp32-projects/my-new-project
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
   Edit `.github/workflows/esp32-build.yml` to include your project.

5. **Update root Makefile:**
   Add build/flash targets for your project.

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

## Questions or Issues?

- **Questions:** Open a [Discussion](https://github.com/laurigates/mcu-tinkering-lab/discussions)
- **Bug Reports:** Open an [Issue](https://github.com/laurigates/mcu-tinkering-lab/issues)
- **Feature Requests:** Open an [Issue](https://github.com/laurigates/mcu-tinkering-lab/issues) with the `enhancement` label

---

**Thank you for contributing to MCU Tinkering Lab! 🚀🤖**
