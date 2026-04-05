# ADR-007: Shared I2C Protocol Component

**Status**: accepted  
**Date**: 2026-03-12  
**Source commit**: refactor(i2c): extract shared I2C protocol component (#110, c092847)  
**Confidence**: 9/10

---

## Context

The `i2c_protocol.h` and `i2c_protocol.c` files implementing the inter-ESP32 communication protocol were duplicated across `robocar-camera` and `robocar-main`. Both copies had diverged slightly — the camera version included `_Static_assert` checks and an `ESP_LOGE` tag-length guard in `prepare_begin_ota_command` that the main version lacked. Any change to the protocol required updating two copies, risking drift and bugs.

The ESP-IDF component system supports shared components via `EXTRA_COMPONENT_DIRS` in CMakeLists.txt, making deduplication straightforward.

## Decision

Extract the shared I2C protocol implementation into a dedicated ESP-IDF component at `packages/shared-libs/robocar-i2c-protocol/`. Use the camera version as the source of truth (it had the more complete implementation). Move board-specific configuration (pin numbers, port, frequency) out of the shared header into per-project `i2c_config.h` files.

## Consequences

**Positive:**
- Single source of truth for the I2C protocol
- Changes to the protocol propagate to both controllers automatically
- `_Static_assert` guards now protect both projects
- Clear separation between protocol definition and hardware configuration

**Negative:**
- Projects must include the shared-libs path in `CMakeLists.txt`
- Slightly more complex build setup for new projects consuming the component

## Alternatives Considered

1. **Keep duplication, add CI diff check** — Fragile; doesn't prevent drift
2. **Git subtree/submodule for shared code** — Overkill for a single component within the same monorepo
3. **Copy-on-change with comments** — Same as keeping duplication

## Files Changed

- Added: `packages/shared-libs/robocar-i2c-protocol/`
- Modified: `packages/esp32-projects/robocar-camera/main/CMakeLists.txt`
- Modified: `packages/esp32-projects/robocar-main/main/CMakeLists.txt`
- Added: per-project `i2c_config.h` files
