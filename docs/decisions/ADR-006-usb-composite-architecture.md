---
id: ADR-006
title: USB Composite Architecture (HID + NCM) for IT Troubleshooter
status: accepted
created: 2026-03-09
---

# ADR-006: USB Composite Architecture (HID + NCM) for IT Troubleshooter

## Context

The IT troubleshooter device needs two distinct communication channels with the
target host:

1. **BIOS-level access**: keyboard input before any OS loads. Only USB HID
   boot-protocol keyboards are recognized by BIOS firmware.
2. **OS-level communication**: bidirectional command execution, file transfer,
   and diagnostic data exchange once the OS is running.

For OS-level communication, two USB class options were evaluated:
- **CDC-ACM** (virtual serial port): simple, proven in the codebase (similar to
  xbox-switch-bridge USB HID pattern), appears as `/dev/ttyACM0`.
- **CDC-NCM** (USB Ethernet): full TCP/IP stack, appears as `usb0` network
  interface, supports DHCP, HTTP server, WiFi bridge.

## Decision

Implement a **phased USB composite device**:

**Phase 1**: HID keyboard + CDC-ACM serial (simpler composite, faster PoC).
**Phase 4**: HID keyboard + CDC-NCM Ethernet (full architecture).

The Phase 4 NCM architecture provides:
- **DHCP server** (device=192.168.4.1, assigns 192.168.4.2 to host).
- **WiFi-to-USB bridge** so the host gets internet through the device.
- **HTTP server** at 192.168.4.1 serving the bootstrap agent script.
- **TCP channel** for structured command execution (vs. line-based serial).
- **Self-bootstrapping**: device types `Ctrl+Alt+T` then
  `curl -s http://192.168.4.1/agent.py | sudo python3 -` via HID keyboard.

ESP-IDF v5.4 includes an official `tusb_ncm` example that bridges WiFi to USB
Ethernet with DHCP — directly applicable. CDC-NCM is class-compliant on Linux
(kernel module `cdc_ncm`, built-in since 3.0) and macOS.

## Consequences

**Positive**
- TCP/IP over USB Ethernet is more robust than serial framing (automatic
  retransmission, flow control, multiplexing).
- WiFi bridge eliminates the need for the host to have its own internet
  connection for downloading packages or updates.
- HTTP server on the device enables self-bootstrapping without pre-installed
  software on the host.
- Standard networking tools (curl, SSH, netcat) work over the USB Ethernet link.

**Negative**
- NCM + HID composite has no official ESP-IDF example (NCM-only and HID-only
  exist separately). Custom descriptor construction required.
- `esp_tinyusb` NCM has a known throughput regression (~0.4 Mbps vs 7 Mbps).
  May need `esp-iot-solution` stack for production performance.
- USB Full Speed (12 Mbps theoretical) limits throughput, but sufficient for
  diagnostic data and SSH.
- Adds lwIP, DHCP server, and HTTP server to firmware — larger flash footprint.

## Alternatives Considered

- **CDC-ACM only**: simpler but requires custom serial framing protocol, no WiFi
  bridge capability, no self-bootstrapping via HTTP.
- **USB Mass Storage + HID**: device appears as USB drive containing agent
  script. Requires user to manually run the script. Less autonomous.
- **BLE HID + WiFi**: BLE keyboard doesn't work in BIOS. Rejected.
