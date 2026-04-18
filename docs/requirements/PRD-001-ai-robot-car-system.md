---
id: PRD-001
title: AI-Powered Robot Car System
status: active
created: 2026-03-05
---

# PRD-001: AI-Powered Robot Car System

## Problem Statement

Building an autonomous robot car requires coordinating motor control, visual perception,
and AI inference across hardware with tight resource constraints. A single microcontroller
cannot simultaneously handle high-resolution camera I/O, network communication to AI
services, and precise real-time motor control. Splitting responsibilities across two
dedicated ESP32 boards allows each to focus on its domain without contention.

## Goals

- Enable autonomous navigation driven by AI vision without human remote control.
- Provide rich operator feedback via OLED display, RGB LEDs, audio cues, and MQTT
  telemetry.
- Support both cloud AI (Claude API) and on-premise AI (Ollama) with zero hardware
  changes.
- Allow development and testing of movement logic without physical hardware via a
  physics simulation.

## Non-Goals

- Voice command recognition (listed as future enhancement).
- SLAM or map-based navigation (future enhancement).
- Multi-robot coordination (future enhancement).

## Requirements

### Main Controller (Heltec WiFi LoRa 32 V1)

| Req | Description |
|-----|-------------|
| R1 | Execute movement commands: forward, backward, left, right, stop, CW/CCW rotation |
| R2 | Control two DC motors via TB6612FNG H-bridge with variable PWM speed |
| R3 | Manage RGB LEDs via PCA9685 16-channel PWM for turn signals and headlights |
| R4 | Control pan/tilt servo camera mount via PCA9685 |
| R5 | Emit audio feedback via piezo buzzer (beep, melody, alert patterns) |
| R6 | Display system status (IP address, diagnostics) on 128x64 OLED (SSD1306) |
| R7 | Receive text-based commands over UART from the vision system at 115200 baud |
| R8 | Support OTA firmware updates within 1.75 MB binary size limit |

### Command Protocol (UART, 115200 baud)

| Command | Action |
|---------|--------|
| `F` | Forward |
| `B` | Backward |
| `L` | Left |
| `R` | Right |
| `S` | Stop |
| `C` / `W` | Clockwise / counter-clockwise rotation |
| `SB` / `SM` / `SA` | Buzzer beep / melody / alert |
| `PAN:<deg>` | Pan camera to angle |
| `TILT:<deg>` | Tilt camera to angle |

### System Integration

- Both controllers connected via UART at GPIO14/15 (CAM) and GPIO16/17 (main).
- Power: 7.4 V LiPo (2S); motors run at battery voltage, ESP32s at regulated 3.3 V.
- Safety: movement commands time out automatically; emergency stop available.

## Success Criteria

- Robot navigates a 5 m straight-line course autonomously using AI vision commands.
- CI build pipeline produces firmware binaries within the 1.75 MB OTA size limit.
- MQTT telemetry stream available with JSON-formatted messages within 1 s of events.
