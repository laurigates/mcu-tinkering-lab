# ADR-0003: Switch Pro Controller Emulation

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: High

## Context

The firmware must present itself as a Nintendo Switch controller over USB. The Switch supports several controller types: Joy-Con (L/R), Pro Controller, and various licensed third-party controllers.

## Decision

Emulate a **wired Nintendo Switch Pro Controller** (VID 0x057E, PID 0x2009).

Key reasons:
- **Well-documented protocol** — the [Nintendo Switch Reverse Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering) project provides comprehensive documentation of the wired USB protocol.
- **Full button set** — the Pro Controller has all the buttons needed to map from an Xbox controller (face buttons, shoulders, triggers, sticks, d-pad, home, capture).
- **USB wired mode** — simpler than Bluetooth pairing. The Switch dock expects wired controllers; plug-and-play after handshake.

Protocol implementation:
1. Switch sends 0x80 USB commands for handshake (STATUS → HANDSHAKE → HIGH_SPEED → FORCE_USB)
2. Controller responds with 0x81 replies (MAC address, controller type, ACKs)
3. Switch sends 0x01 sub-commands (device info, SPI flash reads for calibration, player LED config, IMU enable)
4. Controller responds with 0x21 sub-command replies
5. After FORCE_USB, controller sends 0x30 standard input reports at 125 Hz

## Consequences

- **HID report descriptor must match real Pro Controller** — captured from an actual controller; the Switch largely ignores it but it must be structurally valid for USB enumeration.
- **SPI flash read emulation** — the Switch reads calibration data and colors from "SPI flash" via sub-commands. The firmware returns sensible defaults (factory calibration centers, default colors).
- **No rumble output yet** — the HID descriptor includes the 0x10 rumble report but no handler processes it (see FR10).

## Alternatives Considered

- **Generic HID Gamepad**: The Switch doesn't accept generic gamepads on USB without specific handshake. Rejected.
- **Joy-Con emulation**: More complex (split controllers, IR, NFC). Rejected for scope.
- **Third-party controller emulation**: Less documented, may require different VID/PID. Rejected.
