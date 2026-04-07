---
name: project-claude-md
description: Generate a CLAUDE.md for a project with build commands, hardware constraints, architecture, and development tasks
argument-hint: "<project-path>"
user-invocable: true
allowed-tools: Read, Write, Edit, Grep, Glob, Agent
---

## Task

Generate or update a `CLAUDE.md` for the project at `$1` (relative path under `packages/esp32-projects/`).

## Process

1. **Read the project source** to understand its internals:
   - `main/*.c`, `main/*.h` — all source files, pin defines, task structure
   - `main/CMakeLists.txt` — REQUIRES list
   - `sdkconfig.defaults` — target, features, stack sizes
   - `justfile` — build/flash/monitor recipes and their exact names
   - `README.md` — existing description
   - `WIRING.md` — hardware details
   - `Kconfig.projbuild` — configurable options
   - `components/` — local components
   - `idf_component.yml` or `managed_components/` — external dependencies

2. **Generate CLAUDE.md** following the template below

## Template

Model after the best existing examples: `audiobook-player/CLAUDE.md` (181 lines) and `xbox-switch-bridge/CLAUDE.md` (125 lines).

```markdown
# CLAUDE.md

This file provides guidance to Claude Code when working with this project.

## Project Overview

<What this project does, target hardware, current status.>

## Build Commands

### Using justfile

```bash
just build       # Build firmware (containerized)
just flash       # Flash to device
just monitor     # Serial monitor
just develop     # Build + flash + monitor
just clean       # Remove build artifacts
```

<Include ALL recipes from the project's justfile with descriptions.>

## Critical Hardware Notes

### GPIO Pin Constraints

<Board-specific pin reservations, strapping pins, bus contention issues.>
<Only include if there ARE constraints — omit for simple projects.>

### Power Considerations

<Only if non-trivial: battery, deep sleep, current requirements.>

## Architecture

<How the firmware is structured: main loop vs FreeRTOS tasks, state machine, communication flow.>
<Include component/module descriptions for multi-file projects.>

## Common Development Tasks

### <Task 1>
<Steps for the most common development workflow.>

### Troubleshooting
<Only include if there are non-obvious failure modes.>
```

## Style Rules

- **Target 80-150 lines** — enough detail for Claude to work effectively, not a tutorial
- **Build commands must be accurate** — read the actual justfile, don't guess recipe names
- **Pin constraints only when real** — omit the section for simple projects
- **Architecture section scales with complexity** — 2 lines for a single-file project, a component table for complex ones
- **No generic ESP-IDF advice** — only project-specific information
- **Include credential handling** if the project uses WiFi/API keys (where to create credentials.h, what values to set)
