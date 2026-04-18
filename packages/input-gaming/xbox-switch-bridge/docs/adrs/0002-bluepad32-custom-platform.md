# ADR-0002: Bluepad32 v3.x Custom Platform

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: High

## Context

The firmware needs to scan for and connect to Xbox Series controllers over BLE, receiving gamepad input data (buttons, sticks, triggers, d-pad). Options range from raw BTstack/NimBLE to higher-level libraries.

## Decision

Use **Bluepad32 v3.x** with the **custom platform** integration mode.

Key reasons:
- **Bluepad32 handles controller quirks** — Xbox Series controllers have specific BLE HID nuances. Bluepad32 has tested support for many controller types including Xbox.
- **Custom platform gives full control** — Bluepad32 offers several integration modes (Arduino, CircuitPython, custom). The custom platform (`CONFIG_BLUEPAD32_PLATFORM_CUSTOM`) provides raw callbacks without framework overhead.
- **v3.x is the stable API** — v3.10.3 is the latest release. There is no v4.x in the repository.

Implementation:
- `uni_platform_custom_create()` returns a `struct uni_platform*` with callbacks for connect, disconnect, and controller data events.
- `uni_esp32_main()` is called from `app_main()` and does not return — it runs the BTstack event loop.
- Controller data is written to a shared state struct from the BTstack callback (core 0), read by the bridge task (core 1).

## Consequences

- **BTstack takes over core 0** — `bp32_host_start()` (which calls `uni_esp32_main()`) blocks indefinitely. All other tasks must be spawned before this call.
- **External dependency** — Bluepad32 + btstack must be cloned into `external/` via `just fetch-deps`. btstack sources need to be copied into the component directory (btstack's CMakeLists uses relative paths).
- **Locked to v3.x API** — function names like `uni_bt_enable_new_connections_safe()` are v3-specific.

## Alternatives Considered

- **Raw BTstack/NimBLE**: Full control but significant implementation effort for HID parsing, controller identification, and pairing. Rejected for complexity.
- **Bluepad32 Arduino platform**: Adds Arduino framework overhead. Rejected for unnecessary abstraction.
- **ESP-IDF BLE HID Host example**: Minimal and would require extensive modification for Xbox controller support. Rejected.
