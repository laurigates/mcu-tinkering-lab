# ADR-0004: Three sdkconfig Build Variants

**Status**: Accepted
**Date**: 2026-07-22
**Confidence**: Very High

## Context

The ESP32-S3's USB PHY is shared between USB-Serial-JTAG (debug logging) and
USB-OTG (TinyUSB). Only one can own the PHY at a time. This tension is
identical to xbox-switch-bridge (ADR-0006) and the S3 SoftAP needs WiFi tuned
differently from the JTAG-only case. We have no BLE coexistence axis here.

## Decision

Layer three sdkconfig overlays on `sdkconfig.defaults`:

| Variant | File | TinyUSB | USB-JTAG | Logging | WiFi AP |
|---------|------|---------|----------|---------|---------|
| Production | `sdkconfig.defaults` | ON | OFF | UDP | ON |
| Debug-UART | `sdkconfig.debug-uart` | ON | OFF | UART GPIO43/44 (CP2102) | ON |
| Debug-JTAG | `sdkconfig.debug-jtag` | OFF | ON | USB-Serial-JTAG | ON |
| Debug-USB | `sdkconfig.debug-usb` | ON | OFF | UDP (verbose) | ON |

Build via `SDKCONFIG_DEFAULTS="sdkconfig.defaults;<overlay>" just build-*`.
Later overlays override earlier ones. **Delete `./sdkconfig` before switching**
(the generated cache caches old values — xbox-switch-bridge's justfile
`check-sdkconfig` recipe detects staleness).

## Consequences

- `main.c` and `raw_usb` use `#if CONFIG_TINYUSB_ENABLED` / `CONFIG_USJ_*`
  source guards; components are listed unconditionally in CMake (ESP-IDF
  resolves REQUIRES before kconfig loads, so guards don't work there).
- Debug-JTAG can't emulate USB (PHY is with JTAG) — it's only for
  control-channel/WiFi bring-up without USB.
- Four distinct binaries; only Production is end-user.

## Alternatives Considered

- **Runtime PHY switching.** Hardware-level exclusive; rejected (same as
  xbox-switch-bridge ADR-0006).
- **Five variants (match xbox-switch-bridge).** We have no BLE coexistence to
  isolate, so the `wifi-test` variant is unnecessary.

## Reference

- `xbox-switch-bridge/docs/adrs/0006-sdkconfig-build-variants.md`