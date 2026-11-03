---
name: build
description: Build an ESP32 project (containerized)
argument-hint: "[project-name]"
allowed-tools: Bash(just:*), Bash(docker:*)
---

# Build ESP32 Project

Build an ESP32 project in a Docker container and analyze the output for errors.

## Available Projects

If $ARGUMENTS is empty, show the user the available build targets:

- `robocar` - Robocar firmware (XIAO ESP32-S3 Sense, robocar-unified)
- `robocar-bringup` - Robocar hardware self-test
- `webserver` - ESP32-CAM webserver
- `i2s-audio` - ESP32-CAM I2S audio
- `telegram` - ESP32-CAM LLM Telegram bot
- `kids-audio` - Kids audio toy
- `xbox` - Xbox-Switch bridge
- `wifitest` - WiFi AP test

## Build Commands

Based on $ARGUMENTS, run the appropriate build command:

- `robocar` or `robocar-unified`: `just robocar-unified::build`
- `robocar-bringup` or `bringup`: `just robocar-bringup::build`
- `webserver`: `just webserver::build`
- `i2s-audio` or `audio`: `just i2s-audio::build`
- `telegram`: `just telegram::build`
- `kids-audio`: `just kids-audio::build`
- `xbox`: `just xbox::build`
- `wifitest`: `just wifitest::build`

If no argument provided, build all: `just build-all`

## Error Analysis

After the build completes:
1. Report build success or failure
2. If errors occurred, analyze them and suggest fixes
3. Look for common issues:
   - Missing includes (check CMakeLists.txt dependencies)
   - Linker errors (check component registration)
   - Type mismatches (check header definitions)
4. Report memory usage from the build output
