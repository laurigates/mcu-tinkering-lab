---
name: esp-idf-setup
description: Set up ESP-IDF development environment, create new projects, and verify configuration
argument-hint: "[setup|new <name> <platform>|check]"
user-invocable: true
allowed-tools: Bash(docker:*), Bash(just:*), Bash(mkdir:*), Bash(ls:*), Write, Edit, Read, Glob
---

# ESP-IDF Project Setup Guide

## When to Use This Skill

Apply this skill when the user:
- Wants to set up the build environment for the first time (`setup`)
- Needs to create a new ESP32 project (`new <name> <platform>`)
- Wants to verify the development environment (`check`)
- Wants to configure CMakeLists.txt or component dependencies
- Needs help with sdkconfig or menuconfig
- Wants to add a project to the monorepo

Based on `$ARGUMENTS`:
- Empty or `setup` → Environment setup
- `new <name> <platform>` → Create new project
- `check` → Verify environment

## Environment Setup (Docker-Based)

All ESP-IDF builds run inside Docker containers. No local ESP-IDF installation needed.

### Quick Setup

```bash
# Full environment: Docker images + dev tools
just setup-all

# Or individually:
just docker-build        # Build ESP-IDF container image
just install-dev-tools   # Install host-side linters/formatters
```

### Host-Side Tools

These run natively (not in container):
- `esptool` — for flashing firmware (`pip install esptool`)
- `clang-format` — C/C++ formatting (`brew install clang-format`)
- `cppcheck` — C/C++ linting (`brew install cppcheck`)
- `ruff` — Python linting (`pip install ruff`)
- `pre-commit` — git hooks (`pip install pre-commit`)

### Interactive Container Shell

```bash
just docker-dev          # Root-level interactive shell
just webserver::shell    # Project-specific shell
```

## Verify Environment

Run the environment check:

```bash
just check-environment
```

Also check:

1. Available serial ports:
   ```bash
   ls -la /dev/cu.usbserial-* /dev/ttyUSB* 2>/dev/null || echo "No USB serial devices found"
   ```

2. Docker availability:
   ```bash
   docker --version
   ```

3. Project structure:
   ```bash
   just --list
   ```

Provide a summary: Docker status, serial ports, projects found, missing dependencies.

If issues found:
- Missing Docker → "Install Docker to build ESP-IDF projects"
- No serial ports → "Connect your ESP32 device via USB"
- Missing tools → "Run `just install-dev-tools`"

## Creating New Projects

### Supported Platforms
- `esp32` — ESP-IDF based ESP32 project
- `esp32-cam` — ESP32-CAM specific project
- `arduino` — Arduino platform project
- `stm32` — STM32 platform project

### ESP32/ESP32-CAM Structure

```
packages/esp32-projects/$name/
├── justfile
├── CMakeLists.txt
├── sdkconfig.defaults
├── version.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c
└── README.md
```

### Justfile (import shared config)

```just
# Project Name
# Run `just --list` to see available recipes

set positional-arguments

import '../../../tools/esp32.just'

project_dir := "packages/esp32-projects/project-name"
port := env("PORT", _detected_serial)  # or _detected_s3 for ESP32-S3
target := "esp32"                       # or "esp32s3"

default:
    @just --list

[group: "build"]
build:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building project-name..."
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        bash -c "idf.py set-target {{target}} && idf.py build"
```

### Root CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(project-name)
```

### Main Component CMakeLists.txt

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_wifi
)
```

### sdkconfig.defaults

```ini
CONFIG_IDF_TARGET="esp32"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_PARTITION_TABLE_SINGLE_APP=y
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
```

### Arduino Structure

```
packages/arduino-projects/$name/
├── src/
│   └── main.cpp
├── include/
├── lib/
├── platformio.ini
└── README.md
```

### STM32 Structure

```
packages/stm32-projects/$name/
├── src/
│   └── main.c
├── include/
├── Makefile
└── README.md
```

## Adding to the Monorepo

1. Place project in `packages/esp32-projects/<name>/`
2. Create justfile with `import '../../../tools/esp32.just'`
3. Register as module in root justfile: `mod name 'packages/esp32-projects/<name>'`
4. Test: `just name::build`

## Component Management

### IDF Component Manager

Create `idf_component.yml` in component directory:
```yaml
dependencies:
  espressif/led_strip: "^2.0.0"
```

### Local Components

Place in project's `components/` directory.

## Common Configuration Tasks

### Menuconfig (runs in container)

```bash
just webserver::menuconfig
just xbox::menuconfig
```

## Steps for New Project

1. Create directory structure for the specified platform
2. Generate template files with proper boilerplate
3. Add project to root justfile as `mod`
4. Print next steps for the user

## Best Practices

1. **Use sdkconfig.defaults** — Don't commit generated sdkconfig
2. **Import tools/esp32.just** — Don't duplicate container_cmd, require-port, etc.
3. **Minimal dependencies** — Only include REQUIRES you actually use
4. **Document GPIO usage** — Create WIRING.md with `/wiring-doc`
