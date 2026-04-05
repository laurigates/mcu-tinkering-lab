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
| ADR-007 | ADR | Browser-Based Web Flasher Architecture | accepted | 2026-04-03 |

## Project Overview

| Attribute | Value |
|-----------|-------|
| Repository | https://github.com/laurigates/mcu-tinkering-lab |
| Primary Language (Firmware) | C/C++ (ESP-IDF v5.4+) |
| Primary Language (Simulation) | Python 3.11 |
| Build System | CMake via ESP-IDF, Make, justfile |
| CI/CD | GitHub Actions (6 workflows) |
| Active Platforms | ESP32 (7 projects), Simulation |
| Planned Platforms | Arduino, STM32 |
