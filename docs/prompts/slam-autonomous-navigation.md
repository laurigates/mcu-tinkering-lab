# PRP: SLAM and Autonomous Navigation

**Status**: planned
**Source**: Feature FR-016, PRD-001 (AI Robot Car System) future enhancements
**Priority**: P3
**Confidence**: 5/10

---

## Goal

Enable the robocar to build a map of its environment and navigate autonomously without continuous AI inference for every movement decision. SLAM (Simultaneous Localization and Mapping) would allow the robot to remember explored areas and plan paths.

## Background

Currently the robot is fully reactive: the ESP32-CAM captures a frame, sends it to an AI backend (Claude/Ollama/Gemini), and the response determines the next movement command. This approach has high latency (300ms–2000ms per decision depending on backend) and high cost. For structured environments, a SLAM approach would allow autonomous exploration with AI inference used only for semantic decisions.

## Constraints

- ESP32-CAM has limited RAM (4MB PSRAM) and no hardware floating-point for matrix operations
- OV2640 camera provides monocular vision only (no depth)
- No IMU currently on the camera module (odometry limited to motor encoder counts)

## Options

### Option A: Visual SLAM on companion hardware
- Offload SLAM computation to a companion SBC (Raspberry Pi Zero 2W or Luckfox Pico Ultra)
- ESP32-CAM streams frames; SBC runs ORB-SLAM3 or similar
- Motor commands flow back to robocar-main via UART or MQTT

### Option B: Occupancy grid on ESP32 (simplified)
- Simplified 2D occupancy grid using ultrasonic distance sensor + odometry
- No visual SLAM; purely reactive obstacle mapping
- Feasible on ESP32 with PSRAM

### Option C: Cloud-assisted SLAM
- Send keyframes to cloud; receive map updates
- High latency; not suitable for real-time navigation

## Recommended Approach

Start with **Option B** (ultrasonic + odometry occupancy grid) as a stepping stone. This is implementable on current hardware and provides autonomous obstacle avoidance. Full visual SLAM (Option A) is a later milestone requiring hardware addition.

## Acceptance Criteria (Option B)

- [ ] Ultrasonic distance sensor integrated with robocar-main
- [ ] Odometry from motor encoder or timed movement estimates
- [ ] 2D occupancy grid maintained in ESP32 PSRAM
- [ ] Autonomous obstacle avoidance without AI inference
- [ ] Simulation (robocar-simulation) updated to model sensor readings

## Related

- PRD-001: AI-Powered Robot Car System
- Feature FR-016: SLAM / autonomous navigation
- ADR-003: Pluggable AI Backends
- docs/decisions/ADR-002-dual-esp32-architecture.md
