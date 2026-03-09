---
id: PRD-004
title: Autonomous IT Troubleshooter Device
status: draft
created: 2026-03-09
---

# PRD-004: Autonomous IT Troubleshooter Device

## Problem Statement

Troubleshooting computer hardware issues typically requires a skilled technician
with physical access, OS-level tools, and sometimes BIOS access. The process is
manual, error-prone, and requires the technician to know which diagnostics to run.
A hardware device that autonomously gathers diagnostics — including below-OS-level
operations like BIOS configuration — would reduce troubleshooting time and enable
remote-guided repair via AI.

## Goals

- Autonomously diagnose hardware issues on any Linux computer by plugging in a
  single USB device.
- Access BIOS/UEFI for configuration changes without human keyboard interaction.
- Bridge the device's WiFi connection to give the target host internet access.
- Integrate with hw-diag-agent for structured diagnostic workflows.
- Persist state across reboots for multi-step operations (e.g., change BIOS
  setting → reboot → verify in OS).

## Non-Goals

- Windows support (future enhancement).
- HDMI/display capture for screen reading (infeasible on ESP32 USB 1.1).
- Acting as a permanent monitoring device (single diagnostic session model).
- Replacing a full KVM-over-IP solution.

## Requirements

### Hardware

| Req | Description |
|-----|-------------|
| H1  | Waveshare ESP32-S3-Zero (dual-core 240MHz, 4MB flash, 2MB PSRAM, USB-C) |
| H2  | USB OTG in device mode via TinyUSB (HID + NCM composite) |
| H3  | WiFi STA connection to mobile hotspot for Claude API access |
| H4  | WS2812 RGB status LED on GPIO21 |
| H5  | Optional: USB-UART debug adapter on GPIO43/44 (USB PHY shared with TinyUSB) |

### USB Composite Device

| Req | Description |
|-----|-------------|
| U1  | HID boot-protocol keyboard — must work in BIOS (no OS drivers) |
| U2  | CDC-NCM Ethernet adapter — auto-configured on Linux (cdc_ncm module) |
| U3  | DHCP server on USB netif: device=192.168.4.1, host=192.168.4.2 |
| U4  | WiFi-to-USB Ethernet bridge: host gets internet through device |
| U5  | HTTP server on 192.168.4.1 serving bootstrap agent script |

### BIOS Access

| Req | Description |
|-----|-------------|
| B1  | NVS marker triggers BIOS entry key-hammering on next boot (< 200ms to first keystroke after USB mount) |
| B2  | Vendor-specific key profiles: Dell (F2), HP (F10), Lenovo (F1), generic (DEL/F2/F12/ESC) |
| B3  | BIOS vendor auto-detected via `dmidecode` during OS diagnostic, stored in NVS |
| B4  | Safety: abort after 3 failed reboot attempts (reboot_count in NVS) |
| B5  | Blind navigation: Claude sends key sequences based on known BIOS menu structures |

### Diagnostics

| Req | Description |
|-----|-------------|
| D1  | Bootstrap host agent autonomously (keyboard types curl command to download agent) |
| D2  | Execute shell commands on host via TCP channel over USB Ethernet |
| D3  | Invoke hw-diag-agent phases (discover, health, stress, crash, full) |
| D4  | Relay diagnostic results to Claude API for analysis and next-step decisions |
| D5  | Alternative: SSH to host if credentials available in NVS |

### State Persistence (NVS)

| Req | Description |
|-----|-------------|
| S1  | Device state enum survives power cycles |
| S2  | Session ID links pre/post-reboot diagnostic context |
| S3  | BIOS vendor and target action stored for blind navigation |
| S4  | WiFi credentials + Claude API key stored securely |
| S5  | Optional SSH credentials for managed hosts |

## Architecture Diagram

```
┌─────────────────────────────────────────────┐
│  Claude API (cloud)                          │
└──────────────┬──────────────────────────────┘
               │ HTTPS (WiFi)
┌──────────────▼──────────────────────────────┐
│  ESP32-S3 Device                             │
│  ┌──────────┐ ┌─────────┐ ┌─────────────┐   │
│  │NVS State │ │ Claude  │ │Orchestrator │   │
│  │Machine   │ │ Client  │ │             │   │
│  └──────────┘ └─────────┘ └──┬──────┬───┘   │
│                              │      │       │
│           ┌──────────────────┘      └────┐  │
│           ▼                              ▼  │
│     ┌──────────┐  ┌────────┐  ┌────────┐   │
│     │WiFi STA  │←→│USB HID │  │USB NCM │   │
│     │(hotspot) │  │Keyboard│  │Ethernet│   │
│     └──────────┘  └───┬────┘  └───┬────┘   │
└───────────────────────┼───────────┼─────────┘
              USB cable │           │
┌───────────────────────┼───────────┼─────────┐
│  Target Host          ▼           ▼         │
│              ┌──────────┐  ┌───────────┐    │
│              │ Keyboard │  │usb0       │    │
│              │ (BIOS/OS)│  │192.168.4.2│    │
│              └──────────┘  └─────┬─────┘    │
│                                  │          │
│                          ┌───────▼────────┐ │
│                          │ Host Agent     │ │
│                          │ (Python/TCP)   │ │
│                          │ → hw-diag-agent│ │
│                          └────────────────┘ │
└─────────────────────────────────────────────┘
```

## Success Criteria

1. Device plugged into unknown Linux machine → autonomously produces hw-diag report.
2. Device enters BIOS via NVS marker flow in < 5 seconds after host POST.
3. Host agent bootstrapped via keyboard typing with no pre-installed software.
4. WiFi bridge gives host internet access through device's hotspot connection.
