---
id: PRD-002
title: ESP32-CAM Vision and AI Backend
status: draft
created: 2026-03-05
---

# PRD-002: ESP32-CAM Vision and AI Backend

## Problem Statement

The vision system must capture images, submit them to an AI service for scene analysis,
translate the AI response into robot movement commands, and report telemetry — all on an
ESP32-CAM with 4 MB PSRAM and no dedicated NPU. The AI service may be a cloud API or a
local model, and operators must be able to switch backends without hardware changes.

## Goals

- Capture OV2640 images and forward them for AI analysis at a configurable rate.
- Support Claude API and Ollama as interchangeable AI backends selected at compile time.
- Publish real-time telemetry (log level, heap, WiFi RSSI) to an MQTT broker.
- Discover the MQTT broker automatically via mDNS when no static IP is configured.
- Generate and transmit movement commands to the main controller over UART.

## Non-Goals

- On-device neural inference (no NPU; offloaded to external service).
- Video streaming (handled by the separate `esp32-cam-webserver` project).

## Requirements

### Image Capture and Encoding

| Req | Description |
|-----|-------------|
| R1 | Capture JPEG images from OV2640 at configurable intervals |
| R2 | Base64-encode images for API transmission |
| R3 | Apply rate limiting to stay within API quotas |

### AI Backend Interface

| Req | Description |
|-----|-------------|
| R4 | Compile-time backend selection via `CONFIG_AI_BACKEND_CLAUDE` / `CONFIG_AI_BACKEND_OLLAMA` macros in `config.h` |
| R5 | Claude backend: POST to Anthropic Vision API; model configurable (default `claude-3-haiku-20240307`) |
| R6 | Ollama backend: POST to local server URL; model configurable (default `llava-phi3`) |
| R7 | Parse AI text response and map to movement command strings |
| R8 | Credentials stored in `credentials.h` (excluded from version control) |

### MQTT Telemetry

| Req | Description |
|-----|-------------|
| R9 | Publish JSON log messages to `robocar/logs` topic |
| R10 | Publish status messages to `robocar/status` topic |
| R11 | Buffer messages locally when broker is unreachable |
| R12 | Auto-discover broker via mDNS service type `_mqtt._tcp` |
| R13 | Include timestamp, log level, heap free, and WiFi RSSI in each message |

### Command Output

| Req | Description |
|-----|-------------|
| R14 | Transmit text commands over UART to main controller at 115200 baud |
| R15 | Use GPIO14 (TX) and GPIO15 (RX) — fixed for PSRAM compatibility |

## Success Criteria

- AI analysis completes and a movement command is sent within 3 s of image capture.
- Switching from Claude to Ollama requires only a `config.h` edit and rebuild; no
  wiring or runtime configuration changes.
- MQTT messages appear on broker within 1 s; offline buffer drains on reconnect.
