---
name: project-readme
description: Generate a standardized README.md for an ESP32 project by analyzing source code, justfile, and hardware configuration
argument-hint: "<project-path>"
user-invocable: true
allowed-tools: Read, Write, Edit, Grep, Glob, Agent
---

## Task

Generate or update a `README.md` for the project at `$1` (relative path under `packages/esp32-projects/`).

## Process

1. **Read the project source** to understand its purpose and capabilities:
   - `main/*.c`, `main/*.h` — functionality, peripherals, protocols
   - `main/CMakeLists.txt` — component dependencies
   - `sdkconfig.defaults` — target chip, enabled features
   - `justfile` — available build/flash/monitor recipes
   - `CLAUDE.md` — existing project description (if present)
   - `WIRING.md` — hardware connections (if present)
   - `Kconfig.projbuild` — configurable options

2. **Identify the board and key features** (WiFi, Bluetooth, camera, sensors, etc.)

3. **Generate README.md** following the template below

## Template

```markdown
# <Project Name>

<One paragraph description: what this project does, what hardware it targets, and its key capability.>

## Hardware

- **Board**: <board name and variant>
- **Peripherals**: <list of connected hardware>

For detailed wiring instructions, see [WIRING.md](WIRING.md).

## Quick Start

### Prerequisites

- Docker (for containerized builds)
- USB cable for flashing

### Build and Flash

```bash
just <module>::build     # Build firmware
just <module>::flash     # Flash to device
just <module>::monitor   # Serial monitor
just <module>::develop   # Build + flash + monitor
```

## Configuration

<Describe any required configuration: credentials.h, sdkconfig options, Kconfig choices.>

## Architecture

<Brief description of how the firmware is structured: main loop, tasks, state machine, communication protocols.>

## Project Structure

```
<project-name>/
├── main/
│   ├── main.c          — <brief description>
│   └── ...
├── justfile            — Build recipes
├── sdkconfig.defaults  — ESP-IDF configuration
└── CMakeLists.txt      — CMake project definition
```
```

## Style Rules

- **Concise** — target 50-100 lines, not 300
- **Actionable** — lead with Quick Start so users can build immediately
- **No generic ESP32 tutorials** — only project-specific information
- **Link to WIRING.md** for hardware details rather than duplicating pin tables
- **Link to CLAUDE.md** if it exists for developer-focused details
- **Use actual justfile recipe names** — read the justfile to get the correct module name and recipes
- **Include credential setup** if the project uses WiFi/API keys
