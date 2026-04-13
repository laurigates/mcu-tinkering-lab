# ADR-009: Switch Pro Controller Protocol On-Device

**Status**: accepted
**Date**: 2026-03-15
**Source commits**: feat: move Switch Pro Controller protocol handling on-device (#134), fix(switch_pro_usb): correct SPI address map, ACK bytes (#124, dd6e79c), Add swappable controller profile system (#125)
**Confidence**: 8/10

---

## Context

The Xbox-to-Switch bridge initially relied on PC-side Python tooling for Switch Pro Controller protocol handling during development. Running protocol logic on the PC allowed rapid iteration but required a USB cable to a computer — not suitable for standalone operation. The Switch Pro Controller USB HID protocol is complex: it includes subcommand exchanges (0x01/0x10/0x80/0x81 USB commands), SPI flash reads for controller configuration, and timed input reports (0x30).

Critical bugs were found during development:
- Incorrect SPI address map responses caused Switch to reject the controller
- ACK bytes were wrong, causing handshake failures
- Timer counter was being stamped on 0x81 USB replies (should only be on 0x21 subcommand replies)

## Decision

Implement the full Switch Pro Controller protocol directly on the ESP32-S3 in the `switch_pro_usb` component. All SPI address map responses, subcommand ACKs, and input report timing are handled on-device. A companion `switch-usb-proxy` project remains available for PC-side protocol development (thin USB HID ↔ UART bridge) to allow iteration without reflashing.

## Consequences

**Positive:**
- Fully standalone operation — no PC required
- Sub-millisecond USB response timing possible in firmware (critical for Switch handshake)
- Swappable controller profile system implementable entirely on-device

**Negative:**
- Debugging protocol issues requires UART logging or the USB proxy detour
- Any Switch firmware update that changes the handshake requires firmware update

## Alternatives Considered

1. **Keep PC-side protocol handling** — Permanently tethered to PC; not useful as a standalone device
2. **USB passthrough on ESP32** — Requires USB host + device simultaneously; not supported by ESP32-S3 USB hardware
3. **Raspberry Pi Zero as bridge** — Much higher cost, power, and complexity

## Key Protocol Details Discovered

- SPI address `0x6000` / `0x6050`: controller serial number and calibration data
- 0x21 subcommand replies must include timer counter; 0x81 replies must not
- Input reports (0x30) sent at 60Hz after handshake completes
- Rumble data ignored (stub implemented, never sent to motors)

## Related

- PRD-005: Xbox-to-Switch Bridge
- `packages/esp32-projects/xbox-switch-bridge/components/switch_pro_usb/`
- `packages/esp32-projects/switch-usb-proxy/`
