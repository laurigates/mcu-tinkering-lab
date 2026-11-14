# Blueprint Manifest

MCU Tinkering Lab — embedded systems monorepo for ESP32, STM32, and Arduino platforms.
Primary project: AI-powered robot car (Teuvo) on a single XIAO ESP32-S3 Sense, planning with Gemini Robotics-ER (`packages/robotics/robocar`).

## Documents

| ID | Type | Title | Status | Created |
|----|------|-------|--------|---------|
| PRD-001 | PRD | AI-Powered Robot Car System | active | 2026-03-05 |
| PRD-002 | PRD | Robocar Vision and Gemini Planner | active | 2026-03-05 |
| PRD-003 | PRD | Robot Car Physics Simulation | active | 2026-03-05 |
| PRD-004 | PRD | Autonomous IT Troubleshooter Device | active | 2026-03-09 |
| PRD-005 | PRD | Xbox-to-Switch Bridge | active | 2026-04-05 |
| PRD-006 | PRD | NFC Scavenger Hunt | active | 2026-04-05 |
| PRD-007 | PRD | Audiobook Player | active | 2026-04-05 |
| PRD-008 | PRD | Gamepad Synth | active | 2026-04-07 |
| PRD-010 | PRD | ThinkPack Modular Toy Boxes | draft | 2026-04-11 |
| PRD-011 | PRD | Melody Detector | draft | 2026-04-26 |
| PRD-012 | PRD | Balancebot | draft | 2026-07-02 |
| PRD-013 | PRD | Rapid Hardware Prototyping Before a Fab Order | draft | 2026-10-22 |
| PRD-014 | PRD | Teuvo Sponsored Upgrades — Commissioned Feature Planning Through Conversation | draft | 2026-10-23 |
| PRD-015 | PRD | A Teuvo of Your Own — Named, Customised Robots for Other People | draft | 2026-10-23 |
| PRD-016 | PRD | Build Cost Model and Sourcing Planner | draft | 2026-10-31 |
| PRD-017 | PRD | Robocar Behaviour Twin — Scenario-Driven UX Evaluation | draft | 2026-11-05 |
| ADR-001 | ADR | Monorepo Structure for Multi-Platform MCU Projects | accepted | 2026-03-05 |
| ADR-002 | ADR | Dual ESP32 Architecture for Robot Car (Main Controller + Vision) | superseded | 2026-03-05 |
| ADR-003 | ADR | Pluggable AI Backends (Claude API vs Ollama Self-Hosted) | accepted | 2026-03-05 |
| ADR-004 | ADR | OTA Firmware Update Architecture | unknown | 2026-04-18 |
| ADR-005 | ADR | ESP32-S3 for IT Troubleshooter (USB HID Requirement) | accepted | 2026-03-09 |
| ADR-006 | ADR | USB Composite Architecture (HID + NCM) for IT Troubleshooter | accepted | 2026-03-09 |
| ADR-007 | ADR | Shared I2C Protocol Component | superseded by adr-013 | 2026-03-12 |
| ADR-008 | ADR | Gemini Robotics-ER 1.5 as Third AI Backend | accepted | 2026-03-22 |
| ADR-009 | ADR | Switch Pro Controller Protocol On-Device | accepted | 2026-03-15 |
| ADR-010 | ADR | MQTT Logging Architecture | accepted | 2025-08-13 |
| ADR-011 | ADR | Gamepad Synth I2S Audio Architecture | accepted | 2026-04-07 |
| ADR-012 | ADR | Browser-Based Web Flasher Architecture | unknown | 2026-04-18 |
| ADR-013 | ADR | Single-Board Robot Car with XIAO ESP32-S3 Sense | accepted | 2026-04-09 |
| ADR-014 | ADR | ThinkPack ESP-NOW Mesh Communication | proposed | 2026-04-11 |
| ADR-015 | ADR | `ota_github` Shared OTA Component | accepted | 2026-04-12 |
| ADR-016 | ADR | Hierarchical AI Controller (Planner + Reactive Executor) | accepted | 2026-04-17 |
| ADR-017 | ADR | ESP-DL for On-Device Vision in Melody Detector | proposed | 2026-04-26 |
| ADR-018 | ADR | Pico SDK Toolchain and Balancebot Control Architecture | proposed | 2026-07-02 |
| ADR-019 | ADR | Robocar Voice via Gemini TTS over I2S (MAX98357A) | accepted | 2026-07-20 |
| ADR-020 | ADR | Ambient Audio as a Second Evidence Gate on Speech (OR, not AND) | accepted | 2026-07-31 |
| ADR-021 | ADR | One Hardware Source of Truth — a Board × Header × Parts Join | accepted | 2026-08-19 |
| ADR-022 | ADR | Gate the Planner Request Itself — Dormancy Ladder plus a Hard Spend Ceiling | accepted | 2026-08-28 |
| ADR-023 | ADR | Schematic Rendering — Stay on Schemdraw, Draw Physical Footprints, Mark Every Crossing | accepted | 2026-08-30 |
| ADR-024 | ADR | Conversational Entity — Multi-Turn Multimodal Interaction, Autonomous VAD, and Expressive Personality | proposed | 2026-09-13 |
| ADR-025 | ADR | Reactive Head Aiming — The Executor Owns the Pan/Tilt Head | accepted | 2026-09-15 |
| ADR-026 | ADR | Balancebot LQR With a Live, Bumpless PID/LQR Switch | accepted | 2026-10-12 |
| ADR-027 | ADR | KiCad Carrier Boards Generated From the Hardware Join | accepted | 2026-10-13 |
| ADR-028 | ADR | Teuvo Connectivity — Several Networks, Services Armed on Link-Up, QR Join and an Offline Voice | proposed | 2026-10-23 |
| ADR-029 | ADR | Idle Gaze and Wander in the Executor | accepted | 2026-11-05 |
| ADR-030 | ADR | Robocar Behaviour Twin | proposed | 2026-11-05 |
| ADR-031 | ADR | Circuit Blocks: a Stage Designed Once, Used From Any hardware.toml | accepted | 2026-11-06 |
| ADR-032 | ADR | Real-Time Conversation over the Gemini Live API | proposed | 2026-11-07 |
| ADR-033 | ADR | Robocar Error Reporting through GlitchTip | accepted | 2026-11-09 |
| ADR-034 | ADR | Credential-Free Onboarding — Device-Flow Enrolment, BLE Improv and Broker Discovery | proposed | 2026-11-09 |
| ADR-035 | ADR | A Shared Dimension Standard for Printed Parts | proposed | 2026-11-09 |
| ADR-036 | ADR | Robocar Twin Executor and Plant Port | accepted | 2026-11-13 |
| PRP-hardware-in-the-loop-testing | PRP | Hardware-in-the-Loop (HIL) Testing | planned | 2026-04-18 |
| PRP-hierarchical-ai-controller | PRP | Hierarchical AI Controller (Planner + Reactive Executor) | draft | 2026-04-18 |
| PRP-host-based-unit-tests | PRP | Host-Based Unit Tests Expansion | in-progress | 2026-04-18 |
| PRP-melody-detector | PRP | Melody Detector — Implementation Plan | planned | 2026-04-27 |
| PRP-ota-update-implementation-plan | PRP | OTA Firmware Update System — Implementation Plan | unknown | 2026-04-18 |
| PRP-slam-autonomous-navigation | PRP | SLAM and Autonomous Navigation | planned | 2026-04-18 |

## Project Overview

| Attribute | Value |
|-----------|-------|
| Repository | https://github.com/laurigates/mcu-tinkering-lab |
| Primary Language (Firmware) | C/C++ (ESP-IDF v5.4+) |
| Primary Language (Simulation) | Python 3.12 |
| Build System | CMake via ESP-IDF, Make, justfile |
| CI/CD | GitHub Actions (11+ workflows) |
| Active Platforms | ESP32 (16 projects), Simulation |
| Planned Platforms | Arduino, STM32 |
