# ADR-0002: HID Boot Protocol Keyboard

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 9/10

## Context

USB HID keyboards can use either a full HID report descriptor (flexible, arbitrary key mappings) or the simpler HID boot protocol (fixed 8-byte report, subclass=1, protocol=1). The IT Troubleshooter must work in BIOS/UEFI environments where no HID drivers are loaded.

## Decision

Implement the keyboard as **HID boot protocol** (bInterfaceSubClass=0x01, bInterfaceProtocol=0x01).

## Rationale

BIOS and UEFI firmware implements a simplified USB keyboard driver that only understands boot protocol. A standard HID keyboard with a complex report descriptor may not be recognized at the BIOS level, which would defeat the primary use case of BIOS access and pre-OS interaction.

Boot protocol uses a fixed 8-byte report:
```
Byte 0: Modifier keys (Ctrl, Shift, Alt, GUI)
Byte 1: Reserved (0x00)
Bytes 2–7: Up to 6 simultaneous keycodes
```

This is sufficient for all keyboard input needed (full ASCII + modifier combos).

## Evidence

- `components/usb_composite/usb_composite.c`: `bInterfaceSubClass = 0x01` (Boot Interface), `bInterfaceProtocol = 0x01` (Keyboard)
- HID report descriptor uses standard boot keyboard format (modifiers + 6 keycodes + 5 LED bits output)

## Consequences

- Works in BIOS/UEFI without drivers
- Compatible with all modern OS keyboard drivers (they accept boot protocol keyboards)
- Limited to 6 simultaneous keys — sufficient for normal typing
- LED output report (Num Lock, Caps Lock, Scroll Lock) is received and acknowledged but not acted upon
