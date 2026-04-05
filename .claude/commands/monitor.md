---
allowed-tools: Bash(just:*), Bash(uvx:*)
argument-hint: "[project-name] [port]"
description: Start serial monitor for MCU device
---

## Task
Start serial monitor for the specified project: $ARGUMENTS

## Available Projects
- `main` - Monitor robocar main controller: `just robocar::monitor-main`
- `cam` - Monitor robocar camera: `just robocar::monitor-cam`
- `webserver` - Monitor ESP32-CAM webserver: `just webserver::monitor`
- `audio` - Monitor ESP32-CAM audio: `just i2s-audio::monitor`
- `telegram` - Monitor LLM Telegram bot: `just telegram::monitor`
- `xbox` - Monitor xbox-switch-bridge: `just xbox::monitor`

Ports are auto-detected. Override with: `PORT=/dev/... just <module>::monitor`

Note: Press Ctrl-C to exit the monitor.
