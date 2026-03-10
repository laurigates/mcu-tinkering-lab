# ADR-0008: Phased Delivery Approach

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 9/10

## Context

The full IT Troubleshooter vision (USB keyboard + Ethernet + WiFi + AI) is complex to develop and validate in one shot. Intermediate validation reduces risk.

## Decision

Deliver the project in four distinct phases, each providing a standalone working device:

| Phase | Deliverable | USB Interface |
|-------|-------------|---------------|
| 1 | HID keyboard + CDC-ACM PoC | HID + CDC-ACM |
| 2 | WiFi + command execution via HID injection | HID + CDC-ACM |
| 3 | Claude API AI-driven troubleshooting | HID + CDC-ACM |
| 4 | CDC-NCM USB Ethernet + WiFi bridge | HID + CDC-NCM |

## Rationale

Each phase validates a critical subsystem before adding the next:

1. **Phase 1** validates: USB composite enumeration, HID boot keyboard compatibility, CDC-ACM bidirectional communication
2. **Phase 2** validates: WiFi stack coexistence with USB, NVS state persistence, HID injection for real workflows
3. **Phase 3** validates: Claude API integration, AI decision loop, output capture via CDC or HID
4. **Phase 4** validates: CDC-NCM Ethernet, DHCP server, WiFi bridge — highest complexity, builds on proven USB foundation

## Evidence

- Commit `e1c2a7f`: "feat(it-troubleshooter): add Phase 1 USB composite PoC" — explicitly Phase 1
- `credentials.h.example`: Contains `WIFI_SSID`, `WIFI_PASS`, `CLAUDE_API_KEY` fields with comments noting Phase 1→4 progression
- `main/main.c`: NVS flash initialized in Phase 1 even though not yet used — preparing for Phase 2 state machine

## Consequences

- Phase 1 code is intentionally minimal — avoid over-engineering for future phases
- Each phase merges as a standalone PR with working firmware
- Phase 4 requires the largest refactor (composite descriptor, lwIP, DHCP server) — timeline uncertain
- NVS is initialized from Phase 1 to avoid structural changes later
