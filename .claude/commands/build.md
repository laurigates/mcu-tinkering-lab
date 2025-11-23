---
description: Build an ESP32 project with error analysis
argument-hint: "[project-name]"
allowed-tools: Bash(make:*)
---

# Build ESP32 Project

Build an ESP32 project and analyze the output for errors.

## Available Projects

If $ARGUMENTS is empty, show the user the available build targets:

- `robocar-main` - Main controller (Heltec WiFi LoRa 32)
- `robocar-cam` - Camera module (ESP32-CAM)
- `robocar-all` - Both robocar controllers
- `esp32-webserver` - ESP32-CAM webserver
- `esp32-audio` - ESP32-CAM I2S audio
- `llm-telegram` - ESP32-CAM LLM Telegram bot

## Build Commands

Based on $ARGUMENTS, run the appropriate build command:

- `robocar-main` or `main`: `make robocar-build-main`
- `robocar-cam` or `cam`: `make robocar-build-cam`
- `robocar-all` or `all`: `make robocar-build-all`
- `esp32-webserver` or `webserver`: `make esp32-webserver-build`
- `esp32-audio` or `audio`: `make esp32-audio-build`
- `llm-telegram` or `telegram`: `make llm-telegram-build`

If no argument provided, build all: `make build-all`

## Error Analysis

After the build completes:
1. Report build success or failure
2. If errors occurred, analyze them and suggest fixes
3. Look for common issues:
   - Missing includes (check CMakeLists.txt dependencies)
   - Linker errors (check component registration)
   - Type mismatches (check header definitions)
4. Report memory usage from the build output
