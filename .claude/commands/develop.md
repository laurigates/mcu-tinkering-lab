---
description: Full development cycle - build, flash, and monitor
argument-hint: "[project-name] [port]"
allowed-tools: Bash(make:*)
---

# Development Workflow

Execute the full development cycle: build, flash, and monitor.

## Parse Arguments

$ARGUMENTS may contain:
- Project name (first word)
- Serial port (second word)

## Available Projects

- `robocar-main` or `main` - Main controller development
- `robocar-cam` or `cam` - Camera module development
- `llm-telegram` or `telegram` - LLM Telegram bot development

## Development Commands

Based on project name, run the develop target:

- `robocar-main` or `main`:
  ```bash
  make robocar-develop-main PORT=/dev/xxx
  ```

- `robocar-cam` or `cam`:
  ```bash
  make robocar-develop-cam PORT=/dev/xxx
  ```

- `llm-telegram` or `telegram`:
  ```bash
  make llm-telegram-develop PORT=/dev/xxx
  ```

## For ESP32-CAM Projects

Before running the develop command for ESP32-CAM projects:
1. Remind user: "Connect GPIO0 to GND for programming mode"
2. After flash success, the monitor will start automatically
3. Note: User may need to disconnect GPIO0 and reset to see normal operation

## Monitoring

The development command ends with serial monitor (idf.py monitor).
- Exit monitor with Ctrl+]
- The monitor will show boot messages, logs, and any debug output
