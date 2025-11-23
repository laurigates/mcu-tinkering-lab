---
allowed-tools: Bash(make:*), Bash(idf.py:*)
argument-hint: [project-name] [port]
description: Flash firmware to MCU device (remember GPIO0->GND for ESP32-CAM)
---

## Task
Flash firmware for the specified project: $ARGUMENTS

## Important Notes
- ESP32-CAM modules require GPIO0 connected to GND during programming
- After successful flash, disconnect GPIO0 from GND
- Default port: /dev/cu.usbserial-0001 (can be overridden with PORT=)

## Available Projects
- `main` - Flash robocar main controller: `make robocar-flash-main PORT=$2`
- `cam` - Flash robocar camera: `make robocar-flash-cam PORT=$2`
- `all` - Flash both robocar controllers: `make robocar-flash-all PORT=$2`
- `webserver` - Flash ESP32-CAM webserver: `make esp32-webserver-flash PORT=$2`
- `audio` - Flash ESP32-CAM audio: `make esp32-audio-flash PORT=$2`
- `telegram` - Flash LLM Telegram bot: `make llm-telegram-flash PORT=$2`

If the user specified a port as $2, include it in the make command with PORT=$2.
Otherwise use the default port.

Remind the user about GPIO0->GND requirement for ESP32-CAM based projects before flashing.
