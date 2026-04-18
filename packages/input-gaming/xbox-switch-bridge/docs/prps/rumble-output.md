# PRP: Rumble Output Support

**Status**: Implemented
**Feature**: FR10
**Date**: 2026-03-12

## Overview

Add vibration/rumble feedback from the Nintendo Switch to the Xbox controller. The HID descriptor already includes a 0x10 rumble-only output report, and the Switch sends rumble data during gameplay. Currently, `tud_hid_set_report_cb` receives these reports but the handler only processes sub-commands — rumble data is acknowledged but not forwarded to the Xbox controller.

## Current State

- `switch_pro_usb.c` receives 0x10 reports in `tud_hid_set_report_cb()`
- The handler dispatches to `handle_subcommand()` which ACKs but ignores rumble bytes (bytes 2-9 of the 0x01/0x10 reports)
- Bluepad32 v3.x provides `uni_hid_device_set_rumble()` for sending rumble to connected controllers
- The Xbox Series controller supports rumble via BLE

## Implementation Approach

1. Parse the Switch Pro Controller rumble encoding (4 bytes per motor: frequency + amplitude)
2. Convert to Bluepad32's rumble API format
3. Call `uni_hid_device_set_rumble()` from the bridge task (core 1), not from the TinyUSB callback (which runs in USB context)
4. Use a simple shared rumble state (same single-writer pattern as gamepad input, but reversed: core 1 writes, core 0 reads)

## Open Questions

- What is the Switch Pro Controller rumble encoding? (HD rumble uses frequency/amplitude pairs)
- Does Bluepad32's rumble API support the full frequency range, or only simple on/off?
- What is the acceptable latency for rumble feedback? (should be <16ms for game feel)

## References

- [Switch Pro Controller rumble format](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md)
- Bluepad32 rumble API: `uni_hid_device_set_rumble()`
