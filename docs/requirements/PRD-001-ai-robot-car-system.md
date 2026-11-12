---
id: PRD-001
title: AI-Powered Robot Car System
status: active
created: 2026-03-05
updated: 2026-11-10
---

# PRD-001: AI-Powered Robot Car System

## Problem Statement

Teuvo is a small wheeled robot that looks around, decides what to do from what it
sees, talks about it, and does not drive off a table or into a wall while doing so.
The AI that decides is a cloud model consulted every 15 s at best, so everything that
must happen within milliseconds (braking for an obstacle, holding a heading, aiming the
head) has to run on the robot without it.

The first design split this across two ESP32 boards (ADR-002). Since ADR-013 it runs on
one **XIAO ESP32-S3 Sense** (`packages/robotics/robocar`): camera, planner, reactive
control, voice, WiFi, MQTT and OTA on one module with 8 MB PSRAM. This PRD states what
that system must do; PRD-002 covers vision and the planner in detail.

## Goals

- Behave autonomously from camera input, with no remote control needed.
- Never let a slow or failed cloud call put the robot in danger: the on-board reflexes
  rank above every goal.
- Talk and listen as a character (Teuvo), within a spending budget the owner sets.
- Report its own state: OLED, LEDs, a spoken self-report, MQTT status, and error
  events in GlitchTip (ADR-033).
- Keep working with no broker and no home network, as far as the cloud services allow
  (ADR-028, ADR-034).
- Let movement and behaviour be developed without the hardware: physics simulation
  (PRD-003) and the behaviour twin (PRD-017).

## Non-Goals

- Multi-robot coordination.
- Offline speech recognition or an on-device language model; listening and speaking
  use Gemini (ADR-019, ADR-024).
- Odometry or SLAM in this phase (`docs/prompts/slam-autonomous-navigation.md`).

## Requirements

### R-A. Control architecture (ADR-016)

| Req | Description | State |
|-----|-------------|-------|
| R1 | A slow planner on Core 1 writes goals (`drive`, `track`, `rotate`, `stop`) to a shared `goal_state`; a ~30 Hz executor on Core 0 is the only code that drives the motors | done |
| R2 | The executor keeps running with no goal, a stale goal or no network; a goal past its expiry is not acted on | done |
| R3 | `drive` and `rotate` end by themselves after a time derived from `distance_cm`/`angle_deg`, capped at a few seconds (ADR-029 §4) | open, #1057 |
| R4 | Console and MQTT movement use a lease applied after the reflexes; it expires on its own | done |

### R-B. Safety reflexes

| Req | Description | State |
|-----|-------------|-------|
| R5 | Front RCWL-1601 rangefinder: brake forward motion on one reading below `STOP_THRESHOLD_CM` (15 cm); release on the running mean; worst-case obstacle-to-brake 297 ms (`REACTIVE_REFLEX_WORST_CASE_MS`) | done; bench measurement pending (#791) |
| R6 | Rear rangefinder holds a reverse; a turn in place runs unless both ends are blocked | done |
| R7 | Cliff reflex from front and rear floor sensors: fail closed, brake, back off, refuse motion toward an edge | done; sensors not fitted, off at boot until they are (#1033) |
| R8 | A failed rangefinder read is never reported to anything as a clear path | done |

### R-C. Body and peripherals

| Req | Description | State |
|-----|-------------|-------|
| R9 | Two DC motors through a TB6612FNG, PWM and direction from a PCA9685 behind a TCA9548A mux; coast and brake are separate | done |
| R10 | Pan/tilt head on two servos, aimed by the executor (ADR-025); idle gaze between dwell poses when nothing directs it (ADR-029) | done; servo write race (#1058) |
| R11 | Status LEDs (PCA9685), piezo buzzer, 128x32 SSD1306 OLED status screen | done |
| R12 | Every I2C peripheral is optional: a missing one is logged and named in the self-report, and the console still comes up | done |
| R13 | Power: 2x 18650 (7.4 V nominal) through an LM2596 buck set to 5.0 V | done; brownouts on motor start under investigation (#991) |

### R-D. Voice and listening

| Req | Description | State |
|-----|-------------|-------|
| R14 | Speech through Gemini TTS streamed over SSE to a MAX98357A, in the active persona's voice (ADR-019) | done |
| R15 | Voice turns from the PDM microphone: push-to-talk and hands-free VAD, answered by a flash model with the camera frame (ADR-024) | done |
| R16 | Speaking and voice turns are rationed by budgets that stay tripped until reset (ADR-020, ADR-022) | done |
| R17 | Real-time conversation over the Gemini Live API (ADR-032) | proposed; probe only (#619) |

### R-E. Connectivity, updates and observability

| Req | Description | State |
|-----|-------------|-------|
| R18 | Nothing waits on WiFi; network services start on link-up (ADR-028 §2) | partly; MQTT client creation and OTA subscription (#1054) |
| R19 | Several saved networks, including a phone hotspot; MQTT only on home networks (ADR-028 §1, §6) | open, #848 |
| R20 | WiFi provisioning without a cable or typed credential: Improv Serial today, Improv BLE proposed (ADR-034) | Serial done (protocol fix #1056); BLE proposed |
| R21 | OTA from the web flasher's manifest: strictly newer version only, build-SHA rule, rollback, announce and park before install | code done; no board has completed one (#1005); first check timing (#1055) |
| R22 | Resets, crash dumps and planner notes reach GlitchTip through MQTT (ADR-033) | done |
| R23 | Log lines over MQTT for an untethered dev loop | open, #956 |
| R24 | Gemini credential without a compile-time key: NVS today (`gemini auth`), device-flow enrolment with a proxy proposed (ADR-034, PRD-015 FR-034) | NVS done; enrolment proposed |

## Success Criteria

- On a table with an obstacle placed in its path, the robot stops before contact on
  every trial, with no network connected.
- With the cliff sensors fitted and the reflex on, it never drives off a table edge in
  a 10-minute free run.
- A brownout or panic produces a GlitchTip event naming the reset reason and the last
  log lines (ADR-033).
- An OTA update from a published release installs and confirms on the robot without a
  cable (#1005).
- Request latency for the planner and voice turns is measured and recorded per stage
  (DNS, TLS, upload, time to first byte) on home WiFi and a hotspot; no target is set
  until it is (ADR-034 §1).

## Related

ADR-013 (single board), ADR-016 (planner and executor), ADR-019, ADR-020, ADR-022,
ADR-024, ADR-025, ADR-028, ADR-029, ADR-032, ADR-033, ADR-034; PRD-002 (vision and
planner), PRD-003 (simulation), PRD-015 (robots for other owners), PRD-017 (behaviour
twin). Subsystem detail: `packages/robotics/robocar/docs/firmware/`.

## History

Rewritten 2026-11-10 for the single-board firmware. The first version described the
Heltec WiFi LoRa 32 main controller and an ESP32-CAM talking a one-letter UART command
protocol (ADR-002, superseded by ADR-013); that design is in git history.
