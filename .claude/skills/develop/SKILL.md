---
name: develop
description: Full development cycle - build, flash, and monitor
argument-hint: "[project-name]"
allowed-tools: Bash(just:*), Bash(docker:*)
---

# Development Workflow

Execute the full development cycle: build (containerized), flash (native), and monitor (native).

## Parse Arguments

$ARGUMENTS may contain:
- Project name

## Available Projects

- `robocar` or `robocar-unified` - Robocar firmware (XIAO ESP32-S3 Sense)
- `webserver` - ESP32-CAM webserver
- `telegram` - LLM Telegram bot development
- `i2s-audio` or `audio` - ESP32-CAM audio
- `kids-audio` - Kids audio toy

## Development Commands

Based on project name:

- `robocar` or `robocar-unified`: `just robocar-unified::build && just robocar-unified::flash-monitor`
- `webserver`: `just webserver::develop`
- `telegram`: `just telegram::develop`
- `i2s-audio`: `just i2s-audio::develop`
- `kids-audio`: `just kids-audio::develop`

Ports are auto-detected. Override with: `PORT=/dev/... just <module>::develop`

## For ESP32-CAM Projects

Before running the develop command for ESP32-CAM projects:
1. Remind user: "Connect GPIO0 to GND for programming mode"
2. After flash success, the monitor will start automatically
3. Note: User may need to disconnect GPIO0 and reset to see normal operation

## Monitoring

The development command ends with serial monitor via pyserial.
- Exit monitor with Ctrl-C
- The monitor will show boot messages, logs, and any debug output
