# ADR-015: `ota_github` Shared OTA Component

**Status**: accepted
**Date**: 2026-04-12
**Supersedes**: — (refines ADR-004)
**Confidence**: 9/10

---

## Context

[ADR-004](./ADR-004-ota-update-architecture.md) established the robocar's
OTA architecture (esp_ghota polling + MQTT push-notify + dual-board I2C
orchestration + SHA256 verification + rollback protection). That design
shipped and runs in production, but the implementation was split across
two project-local files with several robocar-specific details baked in:

- `packages/esp32-projects/robocar-camera/main/ota_manager.{h,c}` — 373 LOC
- `packages/esp32-projects/robocar-main/main/ota_handler.{h,c}` — 338 LOC

Robocar-specific coupling lived throughout:

- MQTT topic `"robocar/ota/notify"` hardcoded in both the device code and
  the reusable CI workflow (`.github/workflows/_build-esp32-firmware.yml`)
- Filename-match substring `"robocar-camera"` compiled in, not configurable
- Dual-controller I2C choreography interleaved with the generic
  download/verify/rollback logic in the same file
- Main-controller URL construction hardcoded to `.../robocar-main.bin`

Several other projects in the monorepo are candidates for OTA
(`esp32-cam-webserver`, `it-troubleshooter`, `esp32cam-llm-telegram`, the
eventual `robocar-unified`). Copy-pasting 700 LOC into each one and
stripping the robocar references is error-prone and fragments bug fixes
across N implementations.

## Decision

Extract the generic OTA logic into a reusable ESP-IDF component at
**`packages/shared-libs/ota_github/`** and refactor the robocar firmware
to consume it as a thin wrapper. Parameterize the hardcoded MQTT topic
in the reusable CI workflow so every project can publish to its own topic.

Key design choices:

1. **Two operating modes in one component**
   - `OTA_GITHUB_MODE_PULL` — periodic esp_ghota polling (camera's role)
   - `OTA_GITHUB_MODE_TRIGGERED` — external URL/tag trigger (main controller's role)

   A single component supporting both modes keeps the shared plumbing
   (rollback timer, state mutex, event bus, progress bookkeeping) in
   one place and lets callers pick the behavior that fits their topology.

2. **Hooks, not inheritance**
   Project-specific behavior (WiFi-on-demand bring-up, peripheral
   quiescence before reboot, peer orchestration) is expressed as an
   optional `ota_github_hooks_t` table. The component never calls
   `esp_wifi_*` or knows about I2C — those remain the app's responsibility.

3. **Injected MQTT client**
   The component accepts an already-initialized `esp_mqtt_client_handle_t`
   rather than creating one. Broker URI, credentials, and retry policy
   remain the caller's concern.

4. **esp_event bus for observers**
   Progress and lifecycle events are dispatched on `OTA_GITHUB_EVENTS`,
   so UIs, logs, and orchestration code can subscribe without polling.

5. **Backward-compatible ABI with I2C**
   `ota_github_status_t` shares numeric values with the I2C protocol's
   `ota_status_t`, so the main controller can report the component's
   status directly over I2C without remapping.

6. **Parameterized CI**
   `.github/workflows/_build-esp32-firmware.yml` gains an
   `mqtt_notify_topic` input. The previous hardcoded topic is restored
   in the robocar callers; other projects default to empty string
   (publish is skipped).

## Consequences

### Positive

- New ESP-IDF projects get OTA in ~10 lines of C + one CMake line.
- Single source of truth — bug fixes propagate to every consumer.
- Rollback, SHA256 verification, and stability timing are consistent.
- MQTT topic is no longer a global assumption; each project owns its own.
- Test surface shrinks: one component tested in isolation replaces N copies.
- Documentation (README + three `docs/*.md` + four Mermaid diagrams)
  provides a turnkey onboarding path for future maintainers.

### Negative

- Existing projects must migrate (trivial — it's a thin wrapper) or
  accept that their duplicate copy diverges from the shared one.
- esp_ghota is now a transitive dependency for any consumer of
  `ota_github`, even those that only use TRIGGERED mode and never call
  into esp_ghota. The linker would strip unused code, but the IDF
  Component Manager still downloads it.
- The "escape hatch" accessor
  `ota_github_pull_get_client_handle()` (returns `void *`, cast to
  `ghota_client_handle_t *` by the caller) leaks one esp_ghota concept
  through the public API. This is the minimum needed for the robocar
  camera's peer-orchestration logic; alternative designs (fully wrapping
  esp_ghota's semver helpers) were rejected as scope creep.

### Risks

- MQTT topic typos: a misconfigured `mqtt_notify_topic` (empty or
  mismatched) silently skips the notify step. The device-side poll
  fallback keeps updates flowing, but the "instant" UX degrades. The
  workflow logs the skip at INFO level to aid diagnosis.
- First-boot rollback behavior is unchanged: the stability timer still
  fires after `stability_timeout_ms` (default 60 s). On USB-flashed boots
  `esp_ota_mark_app_valid_cancel_rollback` returns a benign warning; the
  component logs it and continues.

## Alternatives Considered

1. **Leave each project with its own ota_manager.c / ota_handler.c.**
   Rejected: duplication creates drift and makes bug fixes expensive.
   Already observed with the `mqtt_logger_subscribe` call that exists in
   both `robocar-camera/ota_manager.c` and `robocar-unified/ota_manager.c`
   without a matching declaration.

2. **Switch to ESP-Rainmaker / ESP Insights for OTA.**
   Rejected: adds an Espressif cloud dependency, pulls in a large SDK,
   and duplicates our existing GitHub-based release pipeline. We want
   OTA tied to `release-please` tags, not a third-party dashboard.

3. **Custom OTA built directly on `esp_https_ota`, dropping esp_ghota.**
   Rejected for PULL mode: esp_ghota already handles GitHub API,
   semver parsing, asset matching, and release polling. Reimplementing
   those is meaningful code to own and test. Kept as the *direct* path
   for TRIGGERED mode because the peer already provides the URL/tag.

4. **Split into `ota_github_pull` and `ota_github_triggered` as separate
   components.** Rejected: shared state (rollback timer, mutex, event
   bus, MQTT subscriber) would have to be duplicated or extracted into
   a third "core" component. One component with two modes is simpler.

5. **Embed the dual-MCU I2C orchestration in the shared component.**
   Rejected: I2C topology, command IDs, and maintenance-mode semantics
   are robocar-specific. They remain in `robocar-camera/ota_manager.c`
   where they belong.

## Files Changed

### Added

- `packages/shared-libs/ota_github/CMakeLists.txt`
- `packages/shared-libs/ota_github/idf_component.yml`
- `packages/shared-libs/ota_github/README.md`
- `packages/shared-libs/ota_github/docs/architecture.md`
- `packages/shared-libs/ota_github/docs/adoption-guide.md`
- `packages/shared-libs/ota_github/docs/release-workflow.md`
- `packages/shared-libs/ota_github/docs/diagrams/state-machine.mmd`
- `packages/shared-libs/ota_github/docs/diagrams/sequence-pull.mmd`
- `packages/shared-libs/ota_github/docs/diagrams/sequence-push.mmd`
- `packages/shared-libs/ota_github/docs/diagrams/sequence-triggered.mmd`
- `packages/shared-libs/ota_github/docs/diagrams/partitions.mmd`
- `packages/shared-libs/ota_github/include/ota_github.h`
- `packages/shared-libs/ota_github/include/ota_github_events.h`
- `packages/shared-libs/ota_github/src/ota_github_internal.h`
- `packages/shared-libs/ota_github/src/ota_github.c`
- `packages/shared-libs/ota_github/src/ota_github_pull.c`
- `packages/shared-libs/ota_github/src/ota_github_direct.c`
- `packages/shared-libs/ota_github/src/ota_github_mqtt.c`
- `docs/blueprint/adrs/ADR-015-ota-github-shared-component.md` (this file)

### Modified

- `packages/esp32-projects/robocar-camera/main/ota_manager.c` — rewritten as a thin wrapper
- `packages/esp32-projects/robocar-camera/main/CMakeLists.txt` — add `ota_github` to REQUIRES
- `packages/esp32-projects/robocar-camera/main/idf_component.yml` — drop `fishwaldo/esp_ghota` (transitive now)
- `packages/esp32-projects/robocar-main/main/ota_handler.c` — rewritten as a thin wrapper
- `packages/esp32-projects/robocar-main/main/CMakeLists.txt` — add `ota_github` to REQUIRES
- `.github/workflows/_build-esp32-firmware.yml` — add `mqtt_notify_topic` input, drop hardcoded topic
- `.github/workflows/build-robocar-main.yml` — pass `mqtt_notify_topic: robocar/ota/notify`
- `.github/workflows/build-robocar-camera.yml` — pass `mqtt_notify_topic: robocar/ota/notify`

### Not changed (deliberately)

- `packages/esp32-projects/robocar-unified/main/ota_manager.c` — still uses the pre-refactor pattern. Migrating robocar-unified is a follow-up; the shared component is ready whenever that project is touched again.
- `docs/blueprint/adrs/ADR-004-ota-update-architecture.md` — remains the canonical architectural decision for robocar OTA. This ADR refines its implementation without superseding it.
