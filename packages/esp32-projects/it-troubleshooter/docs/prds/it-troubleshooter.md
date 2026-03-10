# IT Troubleshooter — Product Requirements Document

**Status**: Phase 1 Complete / Phase 2 Planned
**Last Updated**: 2026-03-10
**Confidence**: 8/10 (derived from git history and source code analysis)

## Overview

IT Troubleshooter is a portable USB device built on the Waveshare ESP32-S3-Zero that plugs into any computer and emulates keyboard and network hardware. The primary use case is remote IT support and network troubleshooting — the device can type commands into BIOS or OS terminals, access systems over SSH, and run diagnostics without requiring pre-installed software on the target machine.

## Target Users

- IT administrators performing remote/physical troubleshooting
- Network engineers needing out-of-band device access
- Developers wanting a self-contained USB automation tool

## Architecture

The device uses a phased delivery approach, starting with a simple USB composite PoC and graduating toward a full AI-driven troubleshooting agent.

```
[Target Computer] ←── USB-C ───→ [ESP32-S3-Zero]
                                        │
                              ┌─────────┴──────────┐
                              │   HID Keyboard      │  (Phase 1+)
                              │   CDC-ACM Serial    │  (Phase 1)
                              │   CDC-NCM Ethernet  │  (Phase 4)
                              └─────────┬──────────┘
                                        │
                              ┌─────────┴──────────┐
                              │   WiFi Hotspot      │  (Phase 2+)
                              │   Claude API        │  (Phase 3+)
                              └────────────────────┘
```

## Phases

### Phase 1 — USB Composite PoC (Complete)

**Goal**: Prove ESP32-S3 can enumerate as a simultaneous HID keyboard and CDC-ACM serial device.

| FR Code | Requirement | Status |
|---------|-------------|--------|
| FR1 | USB HID boot-protocol keyboard recognized by BIOS and OS without drivers | Completed |
| FR2 | USB CDC-ACM serial port for bidirectional communication with host | Completed |
| FR3 | Type arbitrary ASCII strings via HID with correct US QWERTY keycodes | Completed |
| FR4 | CDC echo demo to verify bidirectional serial communication | Completed |
| FR5 | WS2812 RGB status LED indicating device state | Completed |

**Hardware**: Waveshare ESP32-S3-Zero (4MB flash, 2MB PSRAM, GPIO21 WS2812)

### Phase 2 — WiFi + Command Execution (Planned)

**Goal**: Connect to a mobile hotspot and execute diagnostic commands on target hosts via SSH.

| FR Code | Requirement | Status |
|---------|-------------|--------|
| FR2.1 | Connect to pre-configured WiFi hotspot on device boot | Planned |
| FR2.2 | Accept SSH or command input via CDC serial from operator | Planned |
| FR2.3 | Inject commands into target via HID keyboard | Planned |
| FR2.4 | NVS-persisted state machine for multi-step workflows across power cycles | Planned |
| FR2.5 | Visual LED feedback during WiFi connection and command execution phases | Planned |

### Phase 3 — Claude API Integration (Planned)

**Goal**: AI-driven troubleshooting; Claude analyzes output and decides next actions.

| FR Code | Requirement | Status |
|---------|-------------|--------|
| FR3.1 | Capture command output via CDC serial or USB HID | Planned |
| FR3.2 | Forward output to Claude API for analysis | Planned |
| FR3.3 | Execute Claude-suggested commands via HID injection | Planned |
| FR3.4 | Configurable API key and model via `credentials.h` | Planned |

### Phase 4 — CDC-NCM USB Ethernet (Planned)

**Goal**: Replace CDC-ACM with CDC-NCM for full TCP/IP over USB — enabling HTTP, SSH, and DHCP without HID injection.

| FR Code | Requirement | Status |
|---------|-------------|--------|
| FR4.1 | CDC-NCM USB Ethernet interface visible to host OS as network adapter | Planned |
| FR4.2 | DHCP server assigning IP to host | Planned |
| FR4.3 | WiFi-to-USB bridge routing host traffic through device's WiFi connection | Planned |
| FR4.4 | HTTP server hosting self-bootstrapping agent script | Planned |
| FR4.5 | Retain HID keyboard interface for BIOS-level access | Planned |

## Non-Functional Requirements

| Requirement | Target |
|-------------|--------|
| USB enumeration time | < 5 seconds |
| Time to first keystroke after mount | < 200 ms (Phase 2+) |
| HID inter-character delay | 10 ms (reliable for most OS keyboard handlers) |
| Boot-to-WiFi ready | < 15 seconds |
| Flash footprint | < 3.5 MB (within 4 MB flash with OTA headroom) |

## Technical Decisions

See ADRs for rationale behind key decisions:

- [ADR-0001: ESP32-S3 MCU Selection](../adrs/0001-esp32s3-mcu-selection.md)
- [ADR-0002: HID Boot Protocol Keyboard](../adrs/0002-hid-boot-protocol-keyboard.md)
- [ADR-0003: CDC-ACM to CDC-NCM Phased Approach](../adrs/0003-cdc-acm-to-ncm-phased.md)
- [ADR-0004: Caller-Driven Status LED](../adrs/0004-caller-driven-status-led.md)
- [ADR-0005: RMT DMA Disabled](../adrs/0005-rmt-dma-disabled.md)
- [ADR-0006: External UART for Debug Monitoring](../adrs/0006-external-uart-debug.md)
- [ADR-0007: Core Assignment Strategy](../adrs/0007-core-assignment-strategy.md)
- [ADR-0008: Phased Delivery Approach](../adrs/0008-phased-delivery.md)
