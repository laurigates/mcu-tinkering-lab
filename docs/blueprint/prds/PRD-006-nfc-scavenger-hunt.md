# PRD-006: NFC Scavenger Hunt

**Status**: active  
**Created**: 2026-04-05 (retroactively derived from git history)  
**Source commits**: feat: add NFC scavenger hunt firmware for ESP32-S3 (#146), feat: add audio cues for buttons and unique RFID tag tones (#148)  
**Confidence**: 7/10

---

## Overview

NFC Scavenger Hunt is an ESP32-S3 firmware project that drives an interactive RFID/NFC-based scavenger hunt game. Players scan NFC/RFID tags placed at locations, receiving unique audio feedback for each tag and button interactions.

## Problem

Physical scavenger hunts require a way to verify location visits and provide engaging feedback. NFC tags provide tamper-resistant, low-cost checkpoints. The device needs to read tags, track progress, and provide satisfying audio/visual feedback.

## Goals

- Read NFC/RFID tags at scavenger hunt checkpoints
- Play unique audio tones per tag to confirm successful scans
- Provide button-triggered audio cues for UI interactions
- Support mDNS for network discoverability

## Users

- Event organizers setting up interactive hunts (schools, camps, team events)
- Participants interacting with the device during the hunt

## Features

### FR-N01: RFID Tag Reading
Read NFC/RFID tags and identify unique tag IDs.

### FR-N02: Unique Per-Tag Audio Tones
Each registered NFC tag produces a unique audio tone when scanned (commit #148).

### FR-N03: Button Audio Cues
Physical buttons trigger audio feedback for UI navigation (commit #148).

### FR-N04: mDNS Discovery
Device discoverable on local network as `nfc-scavenger-hunt.local`.

### FR-N05: Deep Sleep / Power Management
Activity-based power management for battery-operated use.

## Architecture

- **Platform**: ESP32-S3 (Waveshare ESP32-S3-Zero or compatible)
- **NFC**: RC522 or compatible RFID module via SPI
- **Audio**: Buzzer/DAC output for tones
- **Network**: WiFi STA mode with mDNS

## Related ADRs

None yet — relatively new project (single commit origin).

## Status

Active. Core NFC reading and audio feedback implemented. Further game logic (progress tracking, hint system) planned.
