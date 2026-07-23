# ADR-0001: ESP32-S3-Zero — Reuse xbox-switch-bridge Provenance

**Status**: Accepted
**Date**: 2026-07-22
**Confidence**: Very High

## Context

The espdancer backend needs a cheap, widely-available ESP32-S3 board with
native USB-OTG. Two local陪 provenance projects already run TinyUSB on exactly
the target hardware:

- `xbox-switch-bridge` — production BLE→USB bridge on Waveshare ESP32-S3-Zero.
- `switch-usb-proxy` — on-device Pro Controller emulator on the same board.

Both validated the S3's single-OTG-port constraint, the RMT↔TinyUSB DMA
conflict, the SoftAP-without-AMPDU workarounds, the build/flash/monitor flow,
and the `sdkconfig.defaults` that hand the PHY to TinyUSB. xbox-switch-bridge
additionally carries the four-build-variant system, the status-LED component,
and the `log_udp` SoftAP+UDP sink — all directly reusable.

## Decision

Standardize on the **Waveshare ESP32-S3-Zero** (ESP32-S3, 4 MB flash, 2 MB
PSRAM, WS2812 on GPIO21) and scaffold `facedancer-espdancer-fw` by lifting the
scaffolding from xbox-switch-bridge, dropping BLE/controller logic.

Any ESP32-S3 with native USB-OTG works; the S3-Zero is the reference board
because it's the one we've burned in.

## Consequences

- **Full Speed only.** The internal OTG-FS PHY is 12 Mbps. High Speed needs an
  external ULPI PHY, which no common devkit routes. `raw_usb_connect()` must
  reject/downgrade `DeviceSpeed.HIGH`/`LOW` (clone greatdancer's wording).
- **Single OTG port ⇒ no USBProxy/MITM.** The facedancer proxy needs the
  Facedancer to act as *host* to the proxied device *and* *device* to the
  target simultaneously. One port can't. Same limitation the MAX3421 boards
  carry; document it in the eventual facedancer README entry.
- The same `sdkconfig.defaults` PHY ownership rules apply: USB-Serial-JTAG is
  mutually exclusive with USB-OTG, so debug builds use WiFi UDP or an external
  CP2102 UART (GPIO43/44) for logging.

## Alternatives Considered

- **RP2040 / Pico W** — supported by Facedancer's `raspdancer` already; not an
  S3. Would split the hardware story.
- **A custom ULPI HS board** — unlocks High Speed, but is hardware design work,
  out of scope until the Full-Speed relay is proven.

## Reference

- `xbox-switch-bridge/docs/adrs/0001-esp32s3-target-hardware.md`