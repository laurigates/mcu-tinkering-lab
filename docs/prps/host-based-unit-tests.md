# PRP: Host-Based Unit Tests Expansion

**Status**: in-progress  
**Source**: Feature FR-015, commit feat: add host-based unit tests for i2c_protocol and status_led (#129)  
**Priority**: P2  
**Confidence**: 8/10

---

## Goal

Expand host-based (native Linux/macOS) unit tests for ESP32 firmware components. The foundation was laid in commit #129 with tests for `i2c_protocol` and `status_led`. The goal is to cover all shared components and critical business logic without requiring hardware.

## Background

ESP-IDF supports building and running unit tests on the host machine via `CONFIG_IDF_TARGET_LINUX`. This enables fast TDD cycles and CI coverage of firmware logic that doesn't require hardware peripherals. The `robocar-i2c-protocol` shared component and individual project components are candidates.

## Implementation Plan

### Phase 1: Shared component coverage (current)
- `packages/shared-libs/robocar-i2c-protocol/` — I2C protocol encode/decode
- `packages/esp32-projects/xbox-switch-bridge/components/switch_pro_usb/` — Switch protocol parsing

### Phase 2: Project-level component coverage
- `packages/esp32-projects/robocar-main/components/` — motor control logic, PID controller
- `packages/esp32-projects/robocar-camera/main/` — AI command parsing, UART message formatting

### Phase 3: CI integration
- Add host-test job to `esp32-build.yml` GitHub Actions workflow
- Run all host tests as part of PR checks

## Test Framework

- ESP-IDF `unity` test framework (already available)
- Build target: `idf.py build` with `CONFIG_IDF_TARGET_LINUX=y` in `sdkconfig.defaults`
- Discovery: `ctest` or custom `just test-host` recipe

## Acceptance Criteria

- [ ] Host tests pass on macOS (darwin/arm64) and Linux (CI)
- [ ] Coverage for all shared-libs components
- [ ] CI runs host tests on every PR
- [ ] Test runtime < 30 seconds

## Related

- Feature FR-015: ESP32 host-based unit tests
- ADR-007: Shared I2C Protocol Component
- Commit #129: feat: add host-based unit tests for i2c_protocol and status_led
