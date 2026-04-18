# PRD-007: Audiobook Player

**Status**: active
**Created**: 2026-04-05 (retroactively derived from git history)
**Source commits**: feat: Add ESPHome-based RFID audiobook player, feat: Migrate audiobook player from ESP8266 to ESP32 (2025-11-13), feat/audiobook (#48, 2025-11-23), feat: add activity-based deep sleep for audiobook-player (#143, 2026-03-23)
**Confidence**: 8/10

---

## Overview

The Audiobook Player is an RFID-triggered audio playback system built on ESP32. Physical RFID cards/tags are associated with audiobooks — tapping a card starts the corresponding book. Designed for accessibility, particularly for users who have difficulty navigating digital interfaces (e.g., children, elderly, visually impaired users).

## Problem

Standard audiobook apps require complex navigation through touch screens or voice interfaces. A physical card-based system allows users to simply pick up a card representing an audiobook and tap it on the reader to start playback — no menus, no authentication, no complexity.

## Goals

- Associate RFID/NFC cards with audiobook content
- Tap-to-play with immediate audio feedback
- Battery-powered operation with aggressive power management
- Visual feedback via LEDs for card recognition

## Users

- Children learning to listen to audiobooks independently
- Elderly or visually impaired users who benefit from a tangible interface
- Parents/caregivers setting up a simple media player

## Features

### FR-A01: RFID Card Recognition
Read RFID card UIDs and map to configured audiobook content.

### FR-A02: Audio Playback
Stream or play audiobook audio for the matched card.

### FR-A03: LED Feedback
Visual indication of card recognition state (commit #39: RFID tag visibility and LED feedback).

### FR-A04: Deep Sleep with Tilt Switch Wake
Activity-based deep sleep — device sleeps when not in use, wakes on tilt switch motion (commits: feat(audiobook-player): Add deep sleep with tilt switch wake, #143).

### FR-A05: Activity-Based Deep Sleep
Extended deep sleep triggered after inactivity timeout (commit #143: add activity-based deep sleep).

### FR-A06: Platform Migration
Migrated from ESP8266 to ESP32 for additional GPIO, I2S audio, and processing headroom (commit 2025-11-13).

### FR-A07: Board Flexibility
Support for ESP32 WiFi & Bluetooth Battery Board (renamed from earlier board, commit #49).

## Architecture

- **Platform**: ESP32 (ESP32 WiFi & Bluetooth Battery Board)
- **RFID**: RC522 module via SPI
- **Audio**: I2S DAC or PWM buzzer
- **Power**: Battery with deep sleep (tilt switch + activity timeout)
- **Build system**: ESP-IDF v5.4+

## Related Documents

- Feature tracking: FR-013 (audiobook player in feature-tracker.json)

## Status

Active. Core RFID-to-audio mapping and deep sleep implemented. Content management and streaming improvements planned.
