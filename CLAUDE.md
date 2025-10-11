# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is an embedded systems monorepo containing projects for various microcontroller platforms (ESP32, Arduino, STM32). The repository is organized by platform and functionality.

**Repository Structure:**

- **`packages/`** - Individual projects organized by platform
  - **`esp32-projects/`** - ESP32-based projects including:
    - `robocar-main/` - Main controller for robocar (Heltec WiFi LoRa 32)
    - `robocar-camera/` - Camera module for robocar (ESP32-CAM)
    - `robocar-simulation/` - Python physics simulation environment
    - `robocar-docs/` - Robocar documentation and coordination Makefile
    - `esp32cam-llm-telegram/` - Telegram bot with LLM integration
    - `esp32-cam-webserver/` - Web server for video streaming
    - `esp32-cam-i2s-audio/` - Audio processing project
  - **`arduino-projects/`** - Arduino platform projects
  - **`stm32-projects/`** - STM32 platform projects
  - **`shared-libs/`** - Shared libraries across platforms
- **`docs/`** - Global documentation (currently empty)
- **`tools/`** - Build scripts and utilities (currently empty)

## Build System and Commands

Run `make` to see the available commands.

