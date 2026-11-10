---
id: PRD-002
title: Robocar Vision and Gemini Planner
status: active
created: 2026-03-05
updated: 2026-11-10
---

# PRD-002: Robocar Vision and Gemini Planner

## Problem Statement

The robocar decides what to do by showing a camera frame to Gemini Robotics-ER and
acting on the function call it returns. Each request costs money and quota, takes
seconds, and is stateless: the model has never seen the previous frame and does not
remember what it said. So the robot must decide on the device **when** a request is
worth making, put into each request **everything** the model needs to answer it, and
turn the answer into goals the executor (PRD-001 R-A) can run safely.

This replaces the first version's ESP32-CAM with Claude and Ollama backends (ADR-003);
the planner has been Gemini-only since ADR-016, and the camera is the OV3660 on the
XIAO ESP32-S3 Sense (ADR-013).

## Goals

- Plan from what the camera sees, with the measured surroundings alongside the frame.
- Spend requests only when something has changed, and never more than a ceiling the
  owner sets.
- Never let the model assert or act on something the robot did not measure.
- Keep the camera and the planner on Core 1, away from the executor's timing.

## Non-Goals

- On-device neural inference.
- A second AI backend; one would be a newly designed abstraction, not a revival of the
  removed `claude_*`/`ollama_*` code.
- Video streaming (the separate `camera-vision/cam-webserver` project).

## Requirements

### R-A. Camera

| Req | Description | State |
|-----|-------------|-------|
| R1 | OV3660 capture at QVGA, JPEG quality 15, `fb_count` ≥ 2, DMA pinned to Core 1 | done |
| R2 | Exposure and gain left to the sensor's own loops; the JPEG frame buffer sized so a bright, detailed scene is not truncated (`CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE`) | done |
| R3 | A frame records the head's pan and tilt at capture, and gets a dwell pose only while the head has settled (ADR-029 §2) | done |

### R-B. When to ask (ADR-022)

| Req | Description | State |
|-----|-------------|-------|
| R4 | Request only on evidence: a changed view, a sound, a range change or a console command; back off 15 s → 30 → 60 → 120 → 300 s, then go dormant | done |
| R5 | A spend fuse (500 requests and 2,000,000 tokens per boot by default) that stays tripped until `plan resume` or a reboot; an unreadable `usageMetadata` is charged 4,000 tokens | done |
| R6 | No request while there is no link, so offline time spends nothing | done |

### R-C. What each request carries

| Req | Description | State |
|-----|-------------|-------|
| R7 | The frame, plus the front and rear rangefinder readings and whether the obstacle reflex is holding; a failed sensor is stated as unavailable, never as a reading (`planner_range.c`) | done (#1035) |
| R8 | Optional tools are declared only when the robot has decided they apply: `speak` and `sound` when the speech gates open, `note` when the note gate offers a trigger (stateless-model gating) | done |
| R9 | The reason a tool is offered is stated in the prompt, limited to what was measured (`speech_evidence.c`) | done |
| R10 | The closing "function calls only" instruction survives a long persona or recall list (reserved tail in the prompt buffer) | done |

### R-D. Turning the answer into goals

| Req | Description | State |
|-----|-------------|-------|
| R11 | Movement tools: `drive(heading_deg, distance_cm, speed_pct)`, `track(box_2d, max_speed_pct)`, `rotate(angle_deg)`, `stop()`; a heading past ±90° reverses | done |
| R12 | `drive` and `rotate` end by themselves after a bounded time (ADR-029 §4) | open, #1057 |
| R13 | `track` is the only aiming primitive; the executor chooses head, wheels or both (ADR-025) | done |
| R14 | Per-cycle decision logic lives in `planner_cycle_core.c`, so the behaviour twin runs the same code (ADR-030) | done |

### R-E. Credentials and service

| Req | Description | State |
|-----|-------------|-------|
| R15 | Model id verified against ListModels; currently `gemini-robotics-er-2-preview` | done |
| R16 | Gemini key from NVS (`gemini auth`), then `credentials.h`; released images carry none | done |
| R17 | Optionally route requests through the PRD-015 proxy with a per-device token from device-flow enrolment (ADR-034 §1), only after the extra hop's latency is measured | proposed |

## Success Criteria

- With the robot on a motionless bench for an hour, the planner goes dormant and makes
  no request after the ladder reaches it.
- No request runs after the spend fuse trips, until `plan resume`.
- The latency of a planner request is measured per stage (DNS, TLS, JPEG upload, time
  to first byte, total) and recorded with the date and network; there is no target
  until that exists.
- The model's distance judgements in planner notes agree with the rangefinder readings
  sent in the same request.

## Related

ADR-016 (planner and executor), ADR-020 (speech gates), ADR-022 (dormancy and spend
fuse), ADR-025 (head aiming), ADR-029 (idle gaze, bounded motion), ADR-030 (behaviour
twin), ADR-033 (planner notes), ADR-034 (enrolment); PRD-001; `.claude/rules/stateless-model-gating.md`,
`.claude/rules/gemini-api.md`; `packages/robocar/unified/docs/firmware/planner.md` and
`camera-and-log.md`.

## History

Rewritten 2026-11-10. The first version specified an ESP32-CAM (OV2640) with
compile-time Claude/Ollama backends sending one-letter commands over UART to a second
board (ADR-002, ADR-003). Its MQTT broker auto-discovery requirement (`_mqtt._tcp`
over mDNS) is carried forward in ADR-034 §3, limited to home networks.
