# ADR-015: `ota_github` Shared OTA Component

**Status**: accepted
**Date**: 2026-04-12
**Amended**: 2026-09-13 (see Update below — the CI half of this decision
never shipped; `build-firmware.yml` is the live release path instead)
**Supersedes**: — (refines ADR-004)
**Confidence**: 9/10

---

## Context

[ADR-004](./ADR-004-ota-update-architecture.md) established the robocar's
OTA architecture (esp_ghota polling + MQTT push-notify + dual-board I2C
orchestration + SHA256 verification + rollback protection). That design
shipped and runs in production, but the implementation was split across
two project-local files with several robocar-specific details baked in:

- `packages/robocar/camera/main/ota_manager.{h,c}` — 373 LOC
- `packages/robocar/main/main/ota_handler.{h,c}` — 338 LOC

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
**`packages/components/ota-github/`** and refactor the robocar firmware
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

- `packages/components/ota-github/CMakeLists.txt`
- `packages/components/ota-github/idf_component.yml`
- `packages/components/ota-github/README.md`
- `packages/components/ota-github/docs/architecture.md`
- `packages/components/ota-github/docs/adoption-guide.md`
- `packages/components/ota-github/docs/release-workflow.md`
- `packages/components/ota-github/docs/diagrams/state-machine.mmd`
- `packages/components/ota-github/docs/diagrams/sequence-pull.mmd`
- `packages/components/ota-github/docs/diagrams/sequence-push.mmd`
- `packages/components/ota-github/docs/diagrams/sequence-triggered.mmd`
- `packages/components/ota-github/docs/diagrams/partitions.mmd`
- `packages/components/ota-github/include/ota_github.h`
- `packages/components/ota-github/include/ota_github_events.h`
- `packages/components/ota-github/src/ota_github_internal.h`
- `packages/components/ota-github/src/ota_github.c`
- `packages/components/ota-github/src/ota_github_pull.c`
- `packages/components/ota-github/src/ota_github_direct.c`
- `packages/components/ota-github/src/ota_github_mqtt.c`
- `docs/decisions/ADR-015-ota-github-shared-component.md` (this file)

### Modified

- `packages/robocar/camera/main/ota_manager.c` — rewritten as a thin wrapper
- `packages/robocar/camera/main/CMakeLists.txt` — add `ota_github` to REQUIRES
- `packages/robocar/camera/main/idf_component.yml` — drop `fishwaldo/esp_ghota` (transitive now)
- `packages/robocar/main/main/ota_handler.c` — rewritten as a thin wrapper
- `packages/robocar/main/main/CMakeLists.txt` — add `ota_github` to REQUIRES
- `.github/workflows/_build-esp32-firmware.yml` — add `mqtt_notify_topic` input, drop hardcoded topic
- `.github/workflows/build-robocar-main.yml` — pass `mqtt_notify_topic: robocar/ota/notify`
- `.github/workflows/build-robocar-camera.yml` — pass `mqtt_notify_topic: robocar/ota/notify`

### Not changed (deliberately)

- `packages/robocar/unified/main/ota_manager.c` — still uses the pre-refactor pattern. Migrating robocar-unified is a follow-up; the shared component is ready whenever that project is touched again.
- `docs/decisions/ADR-004-ota-update-architecture.md` — remains the canonical architectural decision for robocar OTA. This ADR refines its implementation without superseding it.

## Update (2026-09-13)

Decision point 6 and the two `.github/workflows/build-robocar-*.yml` entries
under "Files Changed" describe a CI design that never actually ran in
production. `_build-esp32-firmware.yml` gained the `mqtt_notify_topic` input
as planned, but its only callers were per-project `release-build` jobs gated
on a `check-tag` step matching `<project>@v` — release-please emits
`<project>-v`, so that gate never fired and the reusable workflow was
orphaned from the day it was added. The single-workflow `build-firmware.yml`
(added later, superseding the abandoned per-project caller design) has
always been the live release path, and it hardcoded a single MQTT topic
(`robocar/ota/notify`) shared by every project rather than reading the
per-project topic this ADR intended.

`_build-esp32-firmware.yml` and `_build-esphome-firmware.yml` were deleted
as dead code (issue #409). The per-project MQTT topic this ADR called for
is now implemented in `build-firmware.yml` itself, sourced from each
project's `flasher.json` (`otaNotifyTopic`) rather than from a workflow
input — there is no per-project caller workflow for it to be an input to.
See [`release-workflow.md`](../../packages/components/ota-github/docs/release-workflow.md)
for the current contract. `architecture.md`, `adoption-guide.md`, and
`release-workflow.md` are corrected in the same change to point at
`build-firmware.yml` instead of the deleted reusables.

The 1.8 MB per-release OTA-partition size gate that `_build-esp32-firmware.yml`
was meant to enforce was never ported to `build-firmware.yml`; issue #556
tracked the decision.

## Update (2026-09-19): the size gate is ESP-IDF's, not ours

**Decision: do not port the hardcoded 1.8 MB gate. The build already is one.**

`idf.py build` fails when the app binary exceeds the smallest app partition of
the partition table it just generated — `components/esptool_py/CMakeLists.txt`
attaches `app_check_size` (which runs `partition_table/check_sizes.py` against
the built `partition-table.bin`) to the `app` target, in every ESP-IDF 5.x
build. So the limit each project is held to is its own real table, in the very
step `build-firmware.yml` runs, before anything is attached to a release.

The number the old gate hardcoded was wrong twice over:

| Project | Smallest app partition | Old gate (1887436) would have |
|---|---|---|
| robocar-main, robocar-camera | `0x1D0000` = 1900544 B | rejected a binary that fits, from 1887437 B up |
| robocar-unified, melody-detector | `0x380000` = 3.5 MB | rejected everything past 1.8 MB, with 1.7 MB still free |
| factory-only tables (switch-usb-proxy, xbox-switch-bridge, facedancer, bringup) | 2–3 MB | same, wrongly |
| the 9 flasher projects with no `partitions.csv` | 1 MB `CONFIG_PARTITION_TABLE_SINGLE_APP` default | passed a 1.5 MB binary that cannot flash |

What shipped instead (`tools/check-app-partition-fit.sh`, run after every CI
and release build): the same `check_sizes.py` invoked verbatim, with the app
file, table file and table offset read from the build's own
`flasher_args.json` — no re-parse of `partitions.csv`, no name assumption.
Its verdict is written to `build/app-partition-fit.txt`, shown in the job
summary per project, and its *absence* fails the job, so the check cannot
silently stop running if a future ESP-IDF detaches the target.

Verified with a negative control against the fetched v5.4 tool: exit 0 at
exactly 0x1D0000 bytes on robocar-main's table, exit 1 at one byte more, and
the same pair at 0x380000 on robocar-unified's.

Two pre-existing defects in `_ci-build-esp32.yml` fell out of reading it for
this: its "Check binary size limits" step compared `build/<project>.bin`
against a hardcoded 1.75 MB, a path that does not exist for robocar-main
(`idf-robocar.bin`) or robocar-camera (`esp32-cam-robocar.bin`), so `stat`
returned 0 and 0 bytes was "within limits"; and the build command ended in
`; true` at top level, so a failed `idf.py build` — and therefore any
overflow — exited 0. Both are corrected in the same change.
