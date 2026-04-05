---
name: esp-idf-setup
description: Set up ESP-IDF development environment via Docker and create new projects
---

# ESP-IDF Project Setup Guide

## When to Use This Skill

Apply this skill when the user:
- Wants to set up the build environment for the first time
- Needs to create a new ESP32 project
- Wants to configure CMakeLists.txt or component dependencies
- Needs help with sdkconfig or menuconfig
- Wants to add a project to the monorepo

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

### Verify

```bash
just check-environment   # Check Docker, serial ports, project structure
just webserver::build    # Test a container build
```

## Creating New Projects

### Minimal Project Structure

```
new-project/
├── justfile
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c
└── sdkconfig.defaults
```

### Justfile (import shared config)

```just
# New Project Name
# Run `just --list` to see available recipes

set positional-arguments

import '../../../tools/esp32.just'

project_dir := "packages/esp32-projects/new-project"
port := env("PORT", _detected_serial)  # or _detected_s3 for ESP32-S3
target := "esp32"                       # or "esp32s3"

default:
    @just --list

[group: "build"]
build:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building new-project..."
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        bash -c "idf.py set-target {{target}} && idf.py build"
```

### Root CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(new-project)
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

## Adding to the Monorepo

1. Place project in `packages/esp32-projects/new-project-name/`
2. Create justfile with `import '../../../tools/esp32.just'` (see template above)
3. Register as module in root justfile: `mod new-project 'packages/esp32-projects/new-project-name'`
4. Test: `just new-project::build`

## Component Management

### Using IDF Component Manager

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

### Interactive Shell (for debugging)

```bash
just webserver::shell
just docker-dev          # Root-level shell
```

## Best Practices

1. **Use sdkconfig.defaults** — Don't commit generated sdkconfig
2. **Import tools/esp32.just** — Don't duplicate container_cmd, require-port, etc.
3. **Minimal dependencies** — Only include REQUIRES you actually use
4. **Document GPIO usage** — Create a pinout table in README
