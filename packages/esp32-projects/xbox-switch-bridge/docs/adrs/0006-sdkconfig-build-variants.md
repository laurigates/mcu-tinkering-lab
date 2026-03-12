# ADR-0006: Four sdkconfig Build Variants

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: Very High

## Context

The ESP32-S3's USB PHY is shared between USB-Serial-JTAG (debug logging) and USB-OTG (TinyUSB/Switch emulation). Only one can own the PHY at a time. This creates a fundamental tension between debugging and production functionality.

Additionally, BLE + SoftAP coexistence is rated "C1: unstable" by Espressif, requiring an isolated WiFi-only build for diagnosing AP visibility issues.

## Decision

Use **four sdkconfig overlay files** layered on top of `sdkconfig.defaults`:

| Variant | File | TinyUSB | BLE | USB-Serial-JTAG | WiFi AP | Use Case |
|---------|------|---------|-----|-----------------|---------|----------|
| Production | `sdkconfig.defaults` | ON | ON | OFF | ON | Normal operation: Xbox → Switch bridging |
| Debug-UART | `sdkconfig.debug` | OFF | ON | ON | ON | BLE debugging with USB log output |
| Debug-USB | `sdkconfig.debug-usb` | ON | ON | OFF | ON (verbose) | Switch handshake debugging via WiFi UDP logs |
| WiFi-Test | `sdkconfig.wifi-test` | OFF | OFF | ON | ON | Isolate SoftAP visibility without BLE interference |

Build commands:
```bash
just build           # Production
just build-debug     # Debug-UART (SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug")
just build-debug-usb # Debug-USB  (SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug-usb")
just build-wifi-test # WiFi-Test  (SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wifi-test")
```

Later overlay files override earlier ones. Each overlay only contains the settings it needs to change.

## Consequences

- **Code uses `#if CONFIG_*` guards** — `main.c` conditionally compiles BLE, TinyUSB, and USB-Serial-JTAG code paths based on which variant is active.
- **Must delete `sdkconfig` when switching variants** — the generated `sdkconfig` caches values. The `check-sdkconfig` justfile recipe detects staleness and prompts for deletion.
- **Four distinct firmware binaries** — each variant produces a different binary. Only "production" is intended for end-user use.

## Alternatives Considered

- **Single build with runtime switching**: The USB PHY cannot be switched at runtime; it's a hardware-level exclusive. Rejected.
- **Two variants (production + debug)**: Insufficient — needed WiFi-Test to diagnose BLE/WiFi coexistence separately. Three wasn't enough either; Debug-USB was needed for handshake debugging with TinyUSB active.
- **Menuconfig per build**: Manual menuconfig is error-prone and not reproducible. Overlay files are version-controlled and deterministic. Rejected.
