---
id: ADR-002
title: Dual ESP32 Architecture for Robot Car (Main Controller + Vision)
status: accepted
created: 2026-03-05
---

# ADR-002: Dual ESP32 Architecture for Robot Car (Main Controller + Vision)

## Context

An autonomous robot car must handle real-time motor control (deterministic, low-latency
PWM signals), I2C peripheral management (PCA9685 for LEDs and servos), and AI-driven
vision (HTTP requests to external APIs, JPEG encoding, MQTT publishing). Combining all
of these on one ESP32 creates resource contention: HTTP latency spikes disrupt motor
timing, and PSRAM-heavy camera DMA conflicts with I2C bus operations.

The team evaluated a single ESP32-CAM approach against a dedicated two-board design.

## Decision

The robot uses two physically separate ESP32 boards connected by a UART serial link:

| Role | Hardware | Responsibilities |
|------|----------|-----------------|
| Main Controller | Heltec WiFi LoRa 32 V1 | Motor control (TB6612FNG), LED management (PCA9685), servo pan/tilt, OLED display, piezo audio, UART command receiver |
| Vision System | ESP32-CAM (AI Thinker) | OV2640 image capture, AI backend HTTP client, MQTT telemetry, WiFi management, UART command sender |

The UART link uses 115200 baud with a text-based command protocol (single-letter movement
codes, structured servo angles). GPIO pins are fixed for PSRAM compatibility on the
ESP32-CAM side (TX: GPIO14, RX: GPIO15).

## Consequences

**Positive**
- Real-time motor control is isolated from network I/O; no priority inversion between
  FreeRTOS tasks running HTTP and PWM.
- Each board's firmware can be developed, tested, and flashed independently.
- Adding more sensor boards (e.g., a second camera, GPS) follows the same UART pattern
  without modifying motor control firmware.
- Heltec board's LoRa radio is available for future long-range telemetry without
  competing with WiFi on the vision board.

**Negative**
- Two boards to program, power, and debug instead of one.
- UART introduces a single point of failure; a disconnected wire halts all movement.
- Command round-trip adds one serial frame of latency (~1 ms at 115200 baud) between
  AI decision and motor actuation.
- Two separate CI build jobs required (`robocar-main` and `robocar-camera` matrix).

## Alternatives Considered

- **Single ESP32-CAM**: rejected; camera DMA and PSRAM usage leave insufficient IRAM
  for reliable motor PWM generation and I2C PCA9685 communication simultaneously.
- **ESP32-S3 with AI acceleration**: considered for future revision; current hardware
  is already assembled and functional with the dual-board design.
- **I2C inter-board communication**: rejected in favor of UART; UART requires fewer
  shared pins, tolerates longer cable runs, and avoids I2C address conflicts with the
  PCA9685 already on the main controller's I2C bus.
