---
name: safe-build-operations
description: Safely execute build, flash, and monitor operations for ESP32/MCU projects. Prevents dangerous operations and validates commands before execution.
allowed-tools: Bash(just:*), Bash(docker:*), Bash(esptool*), Bash(ls:*), Bash(pwd:*), Read, Glob, Grep
---

# Safe Build Operations Skill

This skill enables safe execution of build system operations with appropriate safeguards.

## When to Use
- Building firmware projects
- Flashing devices
- Monitoring serial output
- Cleaning build artifacts
- Running development workflows

## Permitted Operations

### Build Commands (containerized)
```bash
just build-all
just robocar::build-all
just robocar::build-main
just robocar::build-cam
just webserver::build
just i2s-audio::build
just telegram::build
just kids-audio::build
just xbox::build
just wifitest::build
```

### Flash Commands (native — ports auto-detected)
```bash
just robocar::flash-main
just robocar::flash-cam
just webserver::flash
just i2s-audio::flash
just telegram::flash
just kids-audio::flash
just xbox::flash
just wifitest::flash
```

### Monitor Commands (native)
```bash
just robocar::monitor-main
just robocar::monitor-cam
just webserver::monitor
just i2s-audio::monitor
just telegram::monitor
just xbox::monitor
```

### Clean Commands
```bash
just clean-all
just robocar::clean-all
just webserver::clean
just telegram::clean
```

### Development Workflows (build + flash + monitor)
```bash
just robocar::develop-main
just robocar::develop-cam
just webserver::develop
just telegram::develop
```

### Code Quality
```bash
just lint
just lint-c
just lint-python
just format
just format-check
```

## Safety Checks

### Before Flashing ESP32-CAM
1. Remind user about GPIO0 → GND requirement
2. Port is auto-detected from `tools/esp32.just` — verify with `just <module>::info`
3. Override with: `PORT=/dev/... just <module>::flash`

### Before Clean Operations
1. Confirm user intent (data loss prevention)
2. List what will be cleaned

### Port Auto-Detection
- USB-serial adapters (CH340/CP2102): auto-detected via `/dev/cu.usbserial-*`
- ESP32-S3 native USB: auto-detected by Espressif VID `0x303a`
- Override with `PORT` env var

## Prohibited Operations

This skill does NOT allow:
- Arbitrary shell commands
- File deletion outside build directories
- Network operations
- Git operations (use dedicated commands)
- System configuration changes

## Usage Examples

### Build a project
"Build the robocar main controller"
→ `just robocar::build-main`

### Flash with specific port
"Flash the camera to /dev/ttyUSB0"
→ Remind about GPIO0, then `PORT=/dev/ttyUSB0 just robocar::flash-cam`

### Full development workflow
"I want to work on the telegram bot"
→ `just telegram::develop`

### Check code quality
"Run the linters"
→ `just lint`
