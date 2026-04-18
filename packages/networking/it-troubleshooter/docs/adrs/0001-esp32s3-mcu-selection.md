# ADR-0001: ESP32-S3 MCU Selection

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 9/10

## Context

The IT Troubleshooter requires a microcontroller capable of emulating a USB HID keyboard and, in later phases, a USB CDC-NCM Ethernet adapter. Initial considerations included the ESP32-C6 (lower cost, newer) and the ESP32-S3.

## Decision

Use the **Waveshare ESP32-S3-Zero** (ESP32-S3, 4MB flash, 2MB PSRAM).

## Rationale

| Feature | ESP32-C6 | ESP32-S3 |
|---------|----------|----------|
| USB OTG (device mode) | No — USB-Serial-JTAG only | Yes — full USB 2.0 OTG |
| HID keyboard emulation | Not possible | Supported via TinyUSB |
| CDC-NCM (Phase 4) | Not possible | Supported via TinyUSB |
| Dual core | No | Yes (240 MHz ×2) |
| PSRAM | No | 2 MB (SPI) |

The ESP32-C6 was rejected because it only has USB-Serial-JTAG (JTAG-over-USB), which cannot act as a USB device class (HID, CDC, etc.) visible to the host OS as a keyboard or network adapter.

## Evidence

- Commit `e1c2a7f`: feat(it-troubleshooter): add Phase 1 USB composite PoC — uses `CONFIG_IDF_TARGET=esp32s3`
- `sdkconfig.defaults`: `CONFIG_TINYUSB_ENABLED=y`, `CONFIG_SPIRAM=y`

## Consequences

- Debugging requires a separate USB-UART adapter (GPIO43/44) because USB-Serial-JTAG shares the PHY with USB-OTG (see ADR-0006)
- PSRAM available for future buffering of SSH session output or Claude API payloads
- Dual core enables USB stack isolation on core 0 (see ADR-0007)
