# ADR-0003: CDC-ACM in Phase 1, CDC-NCM in Phase 4

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 8/10

## Context

For operator communication with the device, two USB serial class options exist:
- **CDC-ACM** (Abstract Control Model): Virtual serial port, widely supported, simple
- **CDC-NCM** (Network Control Model): USB Ethernet adapter, full TCP/IP stack, enables HTTP/SSH without HID injection

The device will eventually need full networking (Phase 4), but starting with networking adds significant complexity.

## Decision

Use **CDC-ACM** in Phase 1 (simple serial PoC), migrate to **CDC-NCM** in Phase 4 (USB Ethernet + WiFi bridge).

## Rationale

CDC-ACM is trivial to implement with TinyUSB's built-in CDC class support. CDC-NCM requires:
- Custom composite USB descriptor (no official ESP-IDF HID+NCM example exists)
- lwIP TCP/IP integration
- DHCP server for host IP assignment
- WiFi-to-USB NAT/bridge routing

Deferring NCM to Phase 4 allows Phase 1 to validate USB enumeration, HID injection, and the composite descriptor structure without network complexity.

## Evidence

- `credentials.h.example`: Comment explicitly documents Phase 1 (CDC-ACM) → Phase 4 (CDC-NCM) transition
- `components/usb_composite/usb_composite.c`: CDC-ACM implementation with single CDC instance
- `sdkconfig.defaults`: `CONFIG_TINYUSB_CDC_ENABLED=y`, `CONFIG_TINYUSB_CDC_COUNT=1`

## Consequences

- Phase 1–3: Operator communicates via CDC serial terminal (minicom, screen, etc.)
- Phase 4: Requires significant refactor of composite descriptor and addition of TCP/IP stack
- HID keyboard interface must be retained alongside NCM in Phase 4 (for BIOS access)
- Phase 4 NCM enables: SSH to managed hosts, HTTP self-bootstrap server, standard tools without HID injection
