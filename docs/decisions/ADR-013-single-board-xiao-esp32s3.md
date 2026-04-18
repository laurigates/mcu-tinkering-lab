---
id: ADR-013
title: Single-Board Robot Car with XIAO ESP32-S3 Sense
status: proposed
created: 2026-04-09
supersedes: ADR-002
---

# ADR-013: Single-Board Robot Car with XIAO ESP32-S3 Sense

## Context

ADR-002 established a dual-board architecture (Heltec WiFi LoRa 32 V1 + ESP32-CAM)
because the original ESP32's camera DMA and PSRAM usage left insufficient IRAM for
reliable motor PWM and I2C operations simultaneously. HTTP latency from AI service
calls also disrupted real-time motor timing.

New hardware changes the trade-off calculus:

- **XIAO ESP32-S3 Sense**: Built-in OV2640 camera, 8MB Octal PSRAM (separate SPI bus
  from I2C), 8MB flash, dual-core LX7 (faster than LX6), 11 usable GPIOs, native USB-C
- **TCA9548A**: I2C multiplexer providing 8 isolated channels on one bus
- **PCA9685**: 16-channel 12-bit PWM driver (already used in the current design)

The key insight: by driving ALL motor control through the PCA9685 (direction pins as
digital outputs, speed as 12-bit PWM), the ESP32-S3 only sends occasional I2C commands
rather than generating continuous PWM signals. This eliminates the resource contention
that motivated the original dual-board split.

## Decision

Consolidate the robot car onto a single XIAO ESP32-S3 Sense board (`robocar-unified`),
using the TCA9548A to multiplex I2C peripherals and the PCA9685 for all PWM outputs.

### Hardware Architecture

| Component | Connection | Purpose |
|-----------|-----------|---------|
| XIAO ESP32-S3 Sense | Main MCU | Camera, WiFi, AI, control |
| TCA9548A (0x70) | I2C (GPIO5/6) | Multiplexes I2C channels |
| PCA9685 (0x40) | TCA9548A ch0 | 16-ch PWM: motors, LEDs, servos |
| SSD1306 (0x3C) | TCA9548A ch1 | 128x64 OLED display |
| TB6612FNG | PCA9685 ch8-13 | Motor driver (direction + speed) |

### Core Affinity

- **Core 0**: Motor control task (prio 6), peripheral task (prio 4), serial commands
- **Core 1**: Camera capture (prio 5), AI analysis (prio 3), WiFi/MQTT/OTA

### Inter-task Communication

FreeRTOS queues replace the I2C inter-board protocol. AI analysis results are dispatched
to `motor_cmd_queue` and `peripheral_cmd_queue` via direct function calls.

## Consequences

**Positive**

- Eliminates I2C inter-board protocol, duplicate WiFi stacks, and coordinated OTA
  (significant code and complexity reduction).
- 7 free GPIOs for future sensors (ultrasonic, encoders, IMU).
- 12-bit motor speed resolution (4096 steps vs 256) for smoother control.
- 6 spare TCA9548A channels for future I2C sensors.
- Lower AI-to-motor latency (~5-10ms saved by eliminating I2C serialization).
- Single firmware binary for OTA, single serial console for debugging.
- Smaller physical footprint (XIAO 21x17.5mm vs two full-size boards).

**Negative**

- PCA9685 at 200Hz may produce slight audible motor whine (acceptable for hobby project).
- Single board is a single point of failure (mitigated: dual-board UART was already SPOF).
- I2C bus contention through TCA9548A requires mutex (worst case ~2ms motor delay).
- Larger unified firmware binary (mitigated: 8MB flash with 3.5MB OTA partitions).

## Alternatives Considered

- **Keep dual-board**: Rejected; the ESP32-S3 resolves the original resource contention,
  and the complexity of two boards with I2C inter-board protocol is high.
- **ESP32-S3 without TCA9548A**: Considered; PCA9685 and SSD1306 have different addresses
  and could share one bus, but the TCA9548A provides bus isolation and 6 expansion channels.
- **Separate PCA9685 frequencies for servos vs motors**: Rejected; 200Hz is a workable
  compromise frequency and avoids needing two PCA9685 boards.
