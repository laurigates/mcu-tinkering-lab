# PRD-005: Xbox-to-Switch Bridge

**Status**: active  
**Created**: 2026-04-05 (retroactively derived from git history)  
**Source commits**: feat: xbox-switch-bridge (2026-02), feat(xbox-switch-bridge): multiple PRs (#89, #101, #106, #121, #122), feat: move Switch Pro Controller protocol handling on-device (#134), feat: add Switch USB Proxy project (#131)  
**Confidence**: 8/10

---

## Overview

The Xbox-to-Switch Bridge is an ESP32-S3-based firmware project that acts as a wireless USB HID proxy, allowing an Xbox Bluetooth controller to control a Nintendo Switch by emulating a Switch Pro Controller over USB. The project evolved from a basic bridge to a fully on-device protocol implementation with swappable controller profiles.

## Problem

Nintendo Switch uses a proprietary USB HID protocol for Pro Controllers that is not natively compatible with Xbox controllers. Players who want to use Xbox controllers with the Switch require a bridge device that handles the protocol translation invisibly.

## Goals

- Receive Xbox controller input over Bluetooth
- Translate and forward input as a Switch Pro Controller over USB HID
- Support SoftAP configuration for wireless setup
- Provide a USB proxy mode for PC-side protocol development without reflashing

## Users

- Hobbyists / gamers wanting Xbox controller compatibility with Nintendo Switch
- Embedded developers studying USB HID emulation and Bluetooth HID

## Features

### FR-B01: Xbox Bluetooth HID Reception
Receive Xbox controller input over Bluetooth Classic or BLE.

### FR-B02: Switch Pro Controller USB Emulation
Present as a Switch Pro Controller to the host (Nintendo Switch or PC) using TinyUSB.

### FR-B03: On-Device Protocol Handling
Full Switch Pro Controller USB protocol implemented on ESP32-S3 (commit #134):
- SPI address map responses
- ACK byte handling
- Input report formatting (0x30 standard input, 0x21 subcommand replies)
- Correct subcmd reply timing (0x21 only on 0x21 subcmd replies, not 0x81 USB replies)

### FR-B04: Swappable Controller Profiles
Profile system for different controller mappings (commit #125).

### FR-B05: UDP Log Broadcasting
UDP broadcast for wireless debugging without serial connection (commit #101).

### FR-B06: WS2812 Status LED
RGB LED indicating connection state (commit #89).

### FR-B07: Debug Build Variant
Separate debug build with additional UART logging (commits #101, #121).

### FR-B08: Switch USB Proxy
Companion ESP32-S3 firmware (`switch-usb-proxy`) that presents as Switch Pro Controller over USB and forwards all protocol traffic to PC via UART for rapid Python-based protocol iteration without reflashing (commit #131).

### FR-B09: SoftAP Configuration
WiFi SoftAP for wireless controller pairing and configuration.

## Architecture

- **Platform**: ESP32-S3
- **USB**: TinyUSB (device mode, HID)
- **Bluetooth**: ESP-IDF Bluetooth Classic HID Host
- **Build variants**: Standard, debug-uart (conditional compilation)
- **Related project**: `switch-usb-proxy` (thin USB HID ↔ UART bridge for protocol development)

## Related ADRs

- ADR-009: Switch Pro Controller On-Device Protocol Handling

## Status

Active. Protocol fidelity improvements ongoing. Switch recognizes and accepts input from the bridge.
