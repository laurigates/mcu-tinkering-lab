# ADR-0001: ESP32-S3 Target Hardware

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: Very High

## Context

The project needs a microcontroller that can simultaneously act as a BLE central (to connect to Xbox controllers) and a USB HID device (to emulate a Switch Pro Controller). The USB device must use native USB — not a USB-UART bridge — because the Nintendo Switch requires specific VID/PID and HID descriptors.

## Decision

Use the **Waveshare ESP32-S3-Zero** as the target hardware.

Key reasons:
- **ESP32-S3 has native USB-OTG** — required for custom VID/PID and HID descriptors. The original ESP32 and ESP32-C3 lack USB-OTG (ESP32 has no USB at all; C3 only has USB-Serial-JTAG).
- **BLE support** — ESP32-S3 supports BLE 5.0 for connecting to Xbox Series controllers.
- **Waveshare ESP32-S3-Zero form factor** — compact (24mm x 18mm), USB-C connector, 4MB flash, 2MB PSRAM, on-board WS2812 RGB LED on GPIO21.
- **ESP-IDF v5.4 support** — mature toolchain with TinyUSB integration via `esp_tinyusb` managed component.

## Consequences

- **USB PHY is shared** between USB-Serial-JTAG (debugging) and USB-OTG (TinyUSB). Only one can use the PHY at a time, requiring build variants for debug vs. production.
- **Limited GPIO** — the ESP32-S3-Zero has fewer broken-out pins than full dev boards. GPIO43/44 are available for UART debug output.
- **2MB PSRAM** available for BLE + WiFi stack memory pressure.

## Alternatives Considered

- **ESP32 (original)**: No USB-OTG. Rejected.
- **ESP32-C3**: USB-Serial-JTAG only, no USB-OTG. Rejected.
- **ESP32-S2**: Has USB-OTG but no Bluetooth. Rejected.
- **RP2040/STM32**: Would require separate BLE module. ESP32-S3 integrates both.
