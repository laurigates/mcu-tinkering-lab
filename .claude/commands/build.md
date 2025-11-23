---
allowed-tools: Bash(make:*), Bash(idf.py:*), Read, Glob
argument-hint: [project-name]
description: Build an ESP32/MCU project with context-aware selection
---

## Context
Current directory structure:
!`ls -la packages/esp32-projects/`

## Task
Build the specified project: $ARGUMENTS

If no project is specified, analyze the user's recent context or ask which project to build.

### Available Projects
- `robocar-main` or `main` - Robocar main controller (Heltec WiFi LoRa 32)
- `robocar-camera` or `cam` - Robocar camera module (ESP32-CAM)
- `robocar-all` or `all` - Both robocar controllers
- `webserver` - ESP32-CAM webserver
- `audio` - ESP32-CAM I2S audio
- `telegram` or `llm` - ESP32-CAM LLM Telegram bot

### Build Commands
Use the root Makefile targets:
- `make robocar-build-main` for main controller
- `make robocar-build-cam` for camera
- `make robocar-build-all` for both
- `make esp32-webserver-build` for webserver
- `make esp32-audio-build` for audio
- `make llm-telegram-build` for telegram bot

Report build success/failure and any warnings or errors encountered.
