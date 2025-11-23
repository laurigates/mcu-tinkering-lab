---
allowed-tools: Bash(make:*), Bash(idf.py:*)
argument-hint: [project-name] [port]
description: Start serial monitor for MCU device
---

## Task
Start serial monitor for the specified project: $ARGUMENTS

## Available Projects
- `main` - Monitor robocar main controller: `make robocar-monitor-main PORT=$2`
- `cam` - Monitor robocar camera: `make robocar-monitor-cam PORT=$2`
- `webserver` - Monitor ESP32-CAM webserver: `make esp32-webserver-monitor PORT=$2`
- `audio` - Monitor ESP32-CAM audio: `make esp32-audio-monitor PORT=$2`
- `telegram` - Monitor LLM Telegram bot: `make llm-telegram-monitor PORT=$2`

Default port: /dev/cu.usbserial-0001

Note: Press Ctrl+] to exit the monitor.
