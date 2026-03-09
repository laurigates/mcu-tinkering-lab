---
id: ADR-005
title: ESP32-S3 for IT Troubleshooter (USB HID Requirement)
status: accepted
created: 2026-03-09
---

# ADR-005: ESP32-S3 for IT Troubleshooter (USB HID Requirement)

## Context

The IT troubleshooter device must emulate a USB keyboard to interact with
computers at the BIOS/UEFI level, before any OS or driver is loaded. The initial
hardware choice was the Seeed XIAO ESP32-C6 for its WiFi 6 support.

The ESP32-C6 has only a **USB-Serial-JTAG controller** — a fixed-function
hardware peripheral hardwired to expose CDC serial and JTAG debugging. The
ESP-IDF documentation states: "it cannot be reconfigured to perform any function
other than a serial port and JTAG debugging functionality." No TinyUSB device
mode, no HID class support, no USB OTG peripheral.

The ESP32-S3 has a **full USB OTG peripheral** supporting TinyUSB in device mode
with arbitrary USB classes including HID, CDC-ACM, CDC-NCM, and MSC. The
xbox-switch-bridge project in this repository already demonstrates production-
quality TinyUSB HID device emulation on a Waveshare ESP32-S3-Zero.

## Decision

Use the **Waveshare ESP32-S3-Zero** (already in inventory, proven in
xbox-switch-bridge) instead of the XIAO ESP32-C6.

The USB composite device presents two interfaces:
1. **HID boot-protocol keyboard** — works in BIOS without drivers.
2. **CDC-NCM Ethernet adapter** — provides TCP/IP channel and WiFi bridge.

A dual-chip approach (ESP32-C6 brain + CH9329 UART-to-USB-HID converter) was
considered but rejected: it adds a second MCU, inter-chip UART latency
(problematic for the BIOS fast-path timing), additional failure modes, and
hardware complexity — all for WiFi 6 that provides no meaningful benefit for
small API payloads.

## Consequences

**Positive**
- USB OTG peripheral supports HID (BIOS keyboard), CDC-NCM (Ethernet), and
  future USB classes (MSC for file transfer) in a single composite device.
- Dual-core 240 MHz allows separating USB/keyboard tasks (core 1) from WiFi/
  networking (core 0), matching the proven xbox-switch-bridge architecture.
- Existing TinyUSB v2.x patterns, sdkconfig, and status_led component are
  directly reusable from xbox-switch-bridge.
- NVS flash storage available for persistent state markers.

**Negative**
- WiFi 4 (802.11n) instead of WiFi 6 (802.11ax). Acceptable: device sends
  small JSON payloads to Claude API; throughput is not a bottleneck.
- Only 4 MB flash and 2 MB PSRAM (vs. XIAO ESP32-S3's 8/8 MB). Sufficient for
  firmware + NVS + mbedTLS cert bundle, but leaves less room for growth.
- USB-Serial-JTAG unavailable after TinyUSB init (shared PHY). Debugging
  requires USB-UART adapter on GPIO43/44 or WiFi-based log forwarding.

## Alternatives Considered

- **XIAO ESP32-C6**: rejected — no USB OTG, cannot emulate HID.
- **XIAO ESP32-S3**: not in inventory; same silicon as ESP32-S3-Zero, larger
  flash (8 MB) and PSRAM (8 MB OPI). Could upgrade later if flash becomes tight.
- **Dual-chip (C6 + CH9329)**: rejected — complexity, latency, no net benefit.
- **ESP32-S2**: has USB OTG but single-core, less RAM, no Bluetooth. The S3 is
  strictly better.
