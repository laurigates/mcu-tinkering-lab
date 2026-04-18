# Blueprint Manifest

MCU Tinkering Lab — embedded systems monorepo for ESP32, STM32, and Arduino platforms.
Primary project: dual-ESP32 AI-powered robot car with pluggable AI vision backends.

## Documents

| ID | Type | Title | Status | Created |
|----|------|-------|--------|---------|
| PRD-001 | PRD | AI-Powered Robot Car System | draft | 2026-03-05 |
| PRD-002 | PRD | ESP32-CAM Vision and AI Backend | draft | 2026-03-05 |
| PRD-003 | PRD | Robot Car Physics Simulation | draft | 2026-03-05 |
| ADR-001 | ADR | Monorepo Structure for Multi-Platform MCU Projects | accepted | 2026-03-05 |
| ADR-002 | ADR | Dual ESP32 Architecture for Robot Car | accepted | 2026-03-05 |
| ADR-003 | ADR | Pluggable AI Backends (Claude API vs Ollama) | accepted | 2026-03-05 |
| ADR-004 | ADR | OTA Firmware Update Architecture | accepted | 2026-03-09 |
| ADR-005 | ADR | IT Troubleshooter Hardware Selection | accepted | 2026-03-12 |
| ADR-006 | ADR | USB Composite Device Architecture | accepted | 2026-03-15 |
| ADR-007 | ADR | Shared I2C Protocol Component | accepted | 2026-03-12 |
| ADR-008 | ADR | Gemini Robotics-ER 1.5 as Third AI Backend | draft | 2026-03-18 |
| ADR-009 | ADR | Switch Pro Controller On-Device Protocol | accepted | 2026-03-20 |
| ADR-010 | ADR | MQTT Logging Architecture | accepted | 2026-03-25 |
| ADR-011 | ADR | Gamepad Synth I2S Audio | accepted | 2026-04-07 |
| ADR-012 | ADR | Browser-Based Web Flasher Architecture | accepted | 2026-04-03 |
| PRD-004 | PRD | IT Troubleshooter | draft | 2026-03-12 |
| PRD-005 | PRD | Xbox-to-Switch Bridge | draft | 2026-03-15 |
| PRD-006 | PRD | NFC Scavenger Hunt | draft | 2026-03-18 |
| PRD-007 | PRD | Audiobook Player | draft | 2026-03-22 |
| PRD-008 | PRD | Gamepad Synth | draft | 2026-04-07 |

## Project Overview

| Attribute | Value |
|-----------|-------|
| Repository | https://github.com/laurigates/mcu-tinkering-lab |
| Primary Language (Firmware) | C/C++ (ESP-IDF v5.4+) |
| Primary Language (Simulation) | Python 3.11 |
| Build System | CMake via ESP-IDF, Make, justfile |
| CI/CD | GitHub Actions (11+ workflows) |
| Active Platforms | ESP32 (16 projects), Simulation |
| Planned Platforms | Arduino, STM32 |
