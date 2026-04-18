# OTA Firmware Update System — Implementation Plan

## Phase Status

| Phase | Description | Status | Notes |
|-------|-------------|--------|-------|
| 1 | Camera OTA Manager (robocar-camera) | Complete | `ota_manager.c` with esp_ghota, MQTT trigger, I2C orchestration |
| 2 | Main Controller OTA Handler (robocar-main) | Complete | `ota_handler.c` with WiFi-on-demand, I2C command handlers |
| 3 | CI/CD Integration | Partial | Manifest generation and binary size checks done; MQTT notification template exists but requires `MQTT_BROKER_HOST` secret |
| 4 | ESPHome Template | Not started | Documented here as a pattern; no workflow file created yet |
| 5 | Documentation | Complete | ADR-004, OTA_UPDATES.md, OTA_DEPLOYMENT.md, WEB_FLASHER.md |
| 6 | Delta OTA Optimization | Deferred | Will revisit once full-image OTA is proven in production |

**Related documentation:**
- [ADR-004: OTA Update Architecture](adrs/ADR-004-ota-update-architecture.md) — Architecture decision
- [OTA_UPDATES.md](../../packages/robocar/docs/docs/OTA_UPDATES.md) — System guide
- [OTA_DEPLOYMENT.md](../../packages/robocar/docs/docs/OTA_DEPLOYMENT.md) — Operator runbook

## Context & Current State

The robocar project needs over-the-air firmware updates integrated with release-please + GitHub Actions. Two solutions are needed: **ESP-IDF** (robocar, now) and **ESPHome** (future projects, template only).

### What Already Exists (remarkably well-prepared)
- **Dual OTA partition tables** on both boards: `ota_0` (1.8MB) + `ota_1` (1.8MB) + `otadata` (8KB)
- **I2C OTA protocol commands** (0x50–0x54) with data structures: `ota_begin_data_t`, `ota_status_response_t`, `version_response_t`
- **`build-firmware.yml`** builds both firmwares, generates `manifest.json` with URLs + SHA256, uploads to GitHub Releases
- **release-please** with separate components (`robocar-main`, `robocar-camera`), grouped releases
- **MQTT infrastructure** on camera: broker at `mqtt://192.168.0.100:1883`, topics for logs/status/commands
- **WiFi manager** on main controller: full-featured with NVS credentials, reconnection, event callbacks
- **WiFi manager** on camera: with credential loading and auto-reconnection
- **Helper functions** for all I2C OTA commands: `prepare_begin_ota_command()`, `prepare_enter_maintenance_command()`, etc.

### User Decisions
- **Auto-update**: Automatic download + install when new version detected (with rollback safety)
- **Main controller WiFi**: Enable WiFi on Heltec for direct OTA downloads
- **Update trigger**: Both periodic polling AND MQTT push notification from the start
- **ESPHome**: Future planning only — create reusable workflow template and YAML pattern

---

## Architecture

### ESP-IDF: Hybrid MQTT-Signal + HTTPS-Pull + Periodic Polling

```
┌─────────────────────────────────────────────────────────────┐
│ GitHub Actions                                              │
│                                                             │
│  release-please ──► build-firmware.yml ──► MQTT publish     │
│                     (builds .bin files,    (notify topic     │
│                      manifest.json,        with version +   │
│                      uploads to Release)   manifest URL)    │
└───────────────────────────┬─────────────────────────────────┘
                            │
              ┌─────────────┴──────────────┐
              │ MQTT: robocar/ota/notify    │
              │ + Periodic poll every 6h    │
              ▼                             │
┌─────────────────────────────────────────────────────────────┐
│ ESP32-CAM (robocar-camera) — OTA Manager                    │
│                                                             │
│  1. MQTT notification OR periodic timer triggers check      │
│  2. HTTP GET manifest.json from GitHub Releases             │
│  3. Parse JSON → compare version vs esp_app_get_description │
│  4. Download robocar-camera.bin via esp_https_ota            │
│  5. Verify SHA256 → flash → mark valid → reboot             │
│  6. After stable boot: trigger main controller update       │
│     via I2C OTA commands (0x50→0x51→0x52→0x54)              │
└────────────────────────┬────────────────────────────────────┘
                         │ I2C: CMD_TYPE_BEGIN_OTA (0x51)
                         │ (sends release tag, not full URL)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ Heltec WiFi LoRa 32 (robocar-main) — OTA Handler           │
│                                                             │
│  1. Enter maintenance mode (stop motors/servos)             │
│  2. Init WiFi from NVS credentials                          │
│  3. Construct URL from base pattern + release tag           │
│  4. Download robocar-main.bin via esp_https_ota              │
│  5. Verify SHA256 → flash → mark valid → reboot             │
│  6. Report status back via I2C                              │
└─────────────────────────────────────────────────────────────┘
```

### Key Design Decisions

**I2C URL limitation workaround**: The existing `ota_begin_data_t` only has 20 bytes for URL — too short for GitHub URLs. Instead of sending the full URL over I2C, send only the **release tag** (e.g., `v0.2.0`, fits in 20 bytes). The main controller constructs the full URL from a compile-time base pattern:
```c
// Compile-time constant in config
#define OTA_BASE_URL "https://github.com/<owner>/<repo>/releases/download"
// At runtime: snprintf(url, sizeof(url), "%s/%s/robocar-main.bin", OTA_BASE_URL, tag);
```

**Version comparison**: Use `esp_app_get_description()->version` (embedded at compile time) rather than NVS. Simpler, no state to manage, version always matches the running firmware.

**MQTT notification from CI**: Use a lightweight `mosquitto_pub` command in the workflow. Requires `MQTT_BROKER_URL` in GitHub Secrets. If the broker isn't internet-reachable, the polling fallback ensures updates still work.

**Rollback**: Enable `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`. New firmware must call `esp_ota_mark_app_valid_cancel_rollback()` after stable operation (60s uptime). If it crashes before that, bootloader automatically reverts.

---

## Build vs Buy: Component Options

### Option A: `esp_ghota` — GitHub OTA Component (Recommended for camera)

[`esp_ghota`](https://github.com/Fishwaldo/esp_ghota) is a purpose-built ESP-IDF component for OTA updates from GitHub Releases:

- **Built-in GitHub API integration** with streaming JSON parser (low memory)
- **Semver comparison** out of the box
- **Periodic timer** for automated checks (`ghota_start_update_timer()`)
- **Filename glob matching** — maps release assets to device variants
- **ESP event loop integration** for progress/status callbacks
- **Supports private repos** via GitHub PAT tokens
- **Supports partition updates** (SPIFFS, LittleFS) alongside firmware
- **Compatible with rollback and anti-rollback**

This eliminates the need to write custom manifest downloading, JSON parsing, version comparison, and periodic polling logic. The camera's `ota_manager.c` would wrap `esp_ghota` instead of reimplementing that plumbing.

**Trade-off**: Uses GitHub API (not raw release downloads), which has rate limits (60/hour unauthenticated, 5000/hour with PAT). Polling every 6h from 2 devices = 8 calls/day — well within limits.

### Option B: `esp_delta_ota` — Delta/Differential OTA (Future Enhancement)

[`esp_delta_ota`](https://components.espressif.com/components/espressif/esp_delta_ota) sends only the binary diff between firmware versions:

- **~95% size reduction** — patches are typically ~5% of full binary size
- **Uses `detools`** (Python) for patch generation + `heatshrink` compression on device
- **No extra partitions needed** — patches applied in-place during streaming
- **No bootloader changes** — works with standard ESP-IDF OTA partitions

**CI/CD workflow addition** for delta OTA:
```yaml
- name: Generate delta patch
  run: |
    pip install detools
    # Download previous release binary as base
    gh release download "$PREV_TAG" -p "robocar-camera.bin" -D /tmp/base/
    # Generate delta patch
    python esp_delta_ota_patch_gen.py \
      --chip esp32 \
      --base_binary /tmp/base/robocar-camera.bin \
      --new_binary build/robocar-camera.bin \
      --patch_file_name robocar-camera-delta.bin
```

**Caveat**: Each delta patch only works from one specific base version. A device on v1.0 can't apply a v1.1→v1.2 patch. This adds complexity:
- CI must generate patches from ALL supported base versions, OR
- Devices must do full OTA if they're more than 1 version behind

**Recommendation**: Implement full-image OTA first (Phase 1–3). Add delta OTA as a Phase 6 optimization once the base system is proven. The manifest.json can include both `url` (full binary) and `delta_url` (patch from previous version) — devices use delta when available, fall back to full.

### Chosen Approach

| Component | Role | Phase |
|-----------|------|-------|
| `esp_ghota` | Camera: GitHub release checking, semver, download orchestration | Phase 1 |
| `esp_https_ota` | Both boards: actual OTA flash (used by esp_ghota internally, and directly on main) | Phase 1–2 |
| Custom `ota_manager` | Camera: MQTT notification handler, I2C orchestration for main controller | Phase 1 |
| Custom `ota_handler` | Main: I2C command handler, WiFi-on-demand, OTA execution | Phase 2 |
| `esp_delta_ota` | Both: bandwidth optimization via differential patches | Phase 6 (future) |

---

## Implementation Steps

### Phase 1: Camera OTA Manager (robocar-camera)

#### 1.1 New file: `packages/robocar/camera/main/ota_manager.h`

```c
#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include "esp_err.h"

// Initialize OTA manager (MQTT subscription + periodic check timer)
esp_err_t ota_manager_init(void);

// Manually trigger an update check
esp_err_t ota_manager_check_update(void);

// Get current firmware version string
const char* ota_manager_get_version(void);

// Mark current firmware as valid (call after stable boot)
esp_err_t ota_manager_confirm_valid(void);

#endif // OTA_MANAGER_H
```

#### 1.2 New file: `packages/robocar/camera/main/ota_manager.c`

Key implementation details:
- **`esp_ghota` integration**: Use `ghota_init()` with config pointing to the GitHub repo, `ghota_start_update_timer()` for periodic checks (6h interval)
- **Filename matching**: Configure `filenamematch` as `"robocar-camera*.bin"` to pick the right asset from GitHub Releases
- **Event handler**: Register for `esp_ghota` events (GHOTA_EVENT_UPDATE_AVAILABLE, GHOTA_EVENT_UPDATE_FAILED, GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS, etc.) via ESP event loop
- **MQTT handler**: Additionally subscribe to `robocar/ota/notify` — on notification, call `ghota_check()` for immediate check (supplements the periodic timer)
- **MQTT status reporting**: Publish OTA progress/status to `robocar/ota/status` topic
- **Post-update**: Reboot, then in `app_main()` call `ota_manager_confirm_valid()` after 60s stability timer
- **Main controller orchestration**: After self-update confirmed, check if main controller also needs update:
  1. `prepare_enter_maintenance_command()` → I2C send
  2. `prepare_begin_ota_command()` with release tag → I2C send
  3. Poll `prepare_get_ota_status_command()` every 5s until complete/failed
  4. `prepare_reboot_command()` → I2C send
- **FreeRTOS task**: OTA runs in dedicated task (`OTA_TASK_STACK_SIZE = 8192`, priority 5) to avoid blocking main loop

#### 1.3 Config additions: `packages/robocar/camera/main/config.h`

```c
// OTA Configuration
#define OTA_ENABLED                    1
#define OTA_CHECK_INTERVAL_MS          (6 * 60 * 60 * 1000)  // 6 hours
#define OTA_MANIFEST_URL               "https://github.com/OWNER/REPO/releases/latest/download/manifest.json"
#define OTA_MQTT_NOTIFY_TOPIC          "robocar/ota/notify"
#define OTA_MQTT_STATUS_TOPIC          "robocar/ota/status"
#define OTA_STABILITY_TIMEOUT_MS       60000   // 60s before marking firmware valid
#define OTA_TASK_STACK_SIZE            8192
#define OTA_TASK_PRIORITY              5
#define OTA_HTTP_TIMEOUT_MS            30000   // 30s HTTP timeout
#define OTA_MAX_RETRY_COUNT            3
#define OTA_RETRY_DELAY_MS             60000   // 1 minute between retries
```

Note: `OWNER/REPO` should be set via `Kconfig.projbuild` menu so it's configurable at build time without touching source.

#### 1.4 Modify: `packages/robocar/camera/sdkconfig.defaults`

Add:
```
# OTA Support
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_FULL=y
CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=n
```

#### 1.5 Modify: `packages/robocar/camera/main/CMakeLists.txt`

Add `ota_manager.c` to SRCS list. Add `esp_https_ota` and `cJSON` to REQUIRES. Add `esp_ghota` as an ESP component dependency in `idf_component.yml`.

#### 1.6 Modify: `packages/robocar/camera/main/main.c`

Add `ota_manager_init()` after MQTT and WiFi initialization. Add stability timer for `ota_manager_confirm_valid()`.

---

### Phase 2: Main Controller OTA Handler (robocar-main)

#### 2.1 New file: `packages/robocar/main/main/ota_handler.h`

```c
#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include "esp_err.h"

// Initialize OTA handler (register I2C command handlers)
esp_err_t ota_handler_init(void);

// Get current OTA status
uint8_t ota_handler_get_status(void);

// Get current OTA progress (0-100)
uint8_t ota_handler_get_progress(void);

// Mark current firmware as valid
esp_err_t ota_handler_confirm_valid(void);

#endif // OTA_HANDLER_H
```

#### 2.2 New file: `packages/robocar/main/main/ota_handler.c`

Key implementation details:
- **I2C command dispatcher integration**: Register handlers for commands 0x50–0x54 in existing I2C slave handler
- **Maintenance mode** (0x50): Stop motors (`motor_stop()`), disable servo updates, display "OTA UPDATE" on OLED
- **Begin OTA** (0x51): Extract release tag from `ota_begin_data_t.url` field, construct full URL:
  ```c
  snprintf(url, sizeof(url), "%s/%s/robocar-main.bin", OTA_REPO_BASE_URL, tag);
  ```
- **WiFi initialization**: Call `wifi_manager_init()` + `wifi_manager_connect(NULL, NULL)` (loads NVS credentials). WiFi credentials must be pre-provisioned via `just robocar::credentials` or NVS.
- **OTA download**: Same `esp_https_ota()` + certificate bundle approach as camera
- **Status reporting** (0x52): Return `ota_status_response_t` with progress percentage
- **Version reporting** (0x53): Return `version_response_t` from `esp_app_get_description()`
- **Reboot** (0x54): `esp_restart()` after brief delay (allow I2C response to complete)
- **Rollback**: Same `esp_ota_mark_app_valid_cancel_rollback()` mechanism

#### 2.3 Config additions: `packages/robocar/main/main/system_config.h`

```c
// OTA Configuration
#define OTA_ENABLED                    1
#define OTA_REPO_BASE_URL              "https://github.com/OWNER/REPO/releases/download"
#define OTA_STABILITY_TIMEOUT_MS       60000
#define OTA_HTTP_TIMEOUT_MS            30000
```

#### 2.4 Modify: `packages/robocar/main/sdkconfig.defaults`

Add:
```
# OTA Support
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_FULL=y
CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=n
```

Note: WiFi is already available on the Heltec board and the wifi_manager module exists. The main controller just doesn't currently call `wifi_manager_init()` during normal operation — it will only initialize WiFi when an OTA command is received.

#### 2.5 Modify: `packages/robocar/main/main/CMakeLists.txt`

Add `ota_handler.c` to SRCS. Add `esp_https_ota`, `cJSON`, `esp_wifi` to REQUIRES.

#### 2.6 Modify: `packages/robocar/main/main/main.c`

Add `ota_handler_init()` call. Add rollback confirmation after stability timer.

---

### Phase 3: CI/CD Integration

#### 3.1 Modify: `.github/workflows/build-firmware.yml`

Add MQTT notification step after firmware upload:

```yaml
- name: Notify devices via MQTT
  if: success()
  continue-on-error: true  # Don't fail release if MQTT is unreachable
  run: |
    # Install mosquitto client
    sudo apt-get install -y mosquitto-clients
    # Publish update notification
    mosquitto_pub \
      -h "${{ secrets.MQTT_BROKER_HOST }}" \
      -p "${{ secrets.MQTT_BROKER_PORT }}" \
      -t "robocar/ota/notify" \
      -m "{\"version\":\"${{ steps.version.outputs.version }}\",\"manifest_url\":\"https://github.com/${{ github.repository }}/releases/latest/download/manifest.json\"}" \
      --cafile /etc/ssl/certs/ca-certificates.crt || true
```

**Required GitHub Secrets:**
- `MQTT_BROKER_HOST`: Public hostname/IP of MQTT broker
- `MQTT_BROKER_PORT`: MQTT port (default 1883, or 8883 for TLS)

**Alternative if broker not internet-reachable:** The `continue-on-error: true` and `|| true` ensure the release still succeeds. Devices will pick up the update via periodic polling within 6 hours.

#### 3.2 Enhance manifest.json in `build-firmware.yml`

Add `min_version` field and use proper JSON generation:

```yaml
- name: Create firmware manifest
  run: |
    jq -n \
      --arg version "${{ steps.version.outputs.version }}" \
      --arg date "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
      --arg main_url "https://github.com/${{ github.repository }}/releases/download/$TAG/robocar-main.bin" \
      --arg main_sha "${{ steps.hashes.outputs.main_sha256 }}" \
      --argjson main_size ${{ steps.hashes.outputs.main_size }} \
      --arg cam_url "https://github.com/${{ github.repository }}/releases/download/$TAG/robocar-camera.bin" \
      --arg cam_sha "${{ steps.hashes.outputs.camera_sha256 }}" \
      --argjson cam_size ${{ steps.hashes.outputs.camera_size }} \
      '{
        version: $version,
        release_date: $date,
        main_controller: {
          url: $main_url,
          sha256: $main_sha,
          size: $main_size,
          min_version: "0.0.0"
        },
        esp32_cam: {
          url: $cam_url,
          sha256: $cam_sha,
          size: $cam_size,
          min_version: "0.0.0"
        }
      }' > manifest.json
```

#### 3.3 Version embedding in builds

Both projects use `project_description.version` from ESP-IDF. In each project's `CMakeLists.txt`, add before `project()`:

```cmake
# Read version from release-please managed file
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
endif()
```

This makes `esp_app_get_description()->version` return the release-please version automatically.

---

### Phase 4: ESPHome Template (Future Use)

#### 4.1 New file: `.github/workflows/esphome-build.yml`

```yaml
name: Build ESPHome Firmware

on:
  release:
    types: [published]
  workflow_dispatch:
    inputs:
      device:
        description: 'Device YAML to build'
        required: true
        type: string

jobs:
  build:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        device: ${{ fromJson(inputs.device || '[]') }}
    steps:
      - uses: actions/checkout@v6

      - name: Build ESPHome firmware
        uses: esphome/build-action@v4
        id: build
        with:
          yaml-file: ${{ matrix.device }}

      - name: Generate MD5
        run: |
          md5sum ${{ steps.build.outputs.name }}/firmware.ota.bin | cut -d' ' -f1 > firmware.md5

      - name: Upload to release
        if: github.event_name == 'release'
        uses: softprops/action-gh-release@v2
        with:
          files: |
            ${{ steps.build.outputs.name }}/firmware.ota.bin
            firmware.md5
```

#### 4.2 ESPHome YAML pattern (documented in ADR, not created as a file)

```yaml
# Template for ESPHome OTA from GitHub Releases
esphome:
  name: my-device
  on_boot:
    priority: -100
    then:
      - delay: 60s
      - script.execute: check_ota

ota:
  - platform: http_request

http_request:

globals:
  - id: current_version
    type: std::string
    initial_value: '"1.0.0"'

script:
  - id: check_ota
    then:
      - http_request.get:
          url: https://github.com/OWNER/REPO/releases/latest/download/manifest.json
          on_response:
            then:
              - lambda: |-
                  // Parse response, compare versions
                  // If newer, trigger OTA flash
              - ota.http_request.flash:
                  url: https://github.com/OWNER/REPO/releases/latest/download/firmware.ota.bin
                  md5_url: https://github.com/OWNER/REPO/releases/latest/download/firmware.md5

interval:
  - interval: 6h
    then:
      - script.execute: check_ota
```

---

### Phase 5: Documentation

#### 5.1 New file: `docs/decisions/ADR-004-ota-update-architecture.md`

Document:
- Decision to use hybrid MQTT-signal + HTTPS-pull + periodic polling
- Why WiFi on main controller (vs I2C proxy)
- Rollback strategy
- I2C tag-based URL construction pattern
- Security model (HTTPS + SHA256 + certificate bundle)
- ESPHome template pattern for future devices

---

### Phase 6: Delta OTA Optimization (Future)

Once the full-image OTA system is proven, add differential updates to reduce bandwidth ~95%:

#### 6.1 Add `esp_delta_ota` component dependency

Both projects add `espressif/esp_delta_ota` to `idf_component.yml`.

#### 6.2 CI/CD delta patch generation in `build-firmware.yml`

```yaml
- name: Generate delta patches
  run: |
    pip install detools
    # Get previous release tag
    PREV_TAG=$(gh release list --limit 2 --json tagName -q '.[1].tagName')
    if [ -n "$PREV_TAG" ]; then
      # Download previous binaries
      gh release download "$PREV_TAG" -p "robocar-camera.bin" -D /tmp/base/ || true
      gh release download "$PREV_TAG" -p "robocar-main.bin" -D /tmp/base/ || true
      # Generate delta patches (camera)
      if [ -f /tmp/base/robocar-camera.bin ]; then
        python -m detools create_patch -c heatshrink \
          /tmp/base/robocar-camera.bin build/robocar-camera.bin \
          robocar-camera-delta.bin
      fi
      # Generate delta patches (main)
      if [ -f /tmp/base/robocar-main.bin ]; then
        python -m detools create_patch -c heatshrink \
          /tmp/base/robocar-main.bin build/robocar-main.bin \
          robocar-main-delta.bin
      fi
    fi
```

#### 6.3 Enhanced manifest.json

```json
{
  "version": "0.3.0",
  "esp32_cam": {
    "url": "...robocar-camera.bin",
    "sha256": "...",
    "size": 1234567,
    "delta": {
      "from_version": "0.2.0",
      "url": "...robocar-camera-delta.bin",
      "sha256": "...",
      "size": 61728
    }
  }
}
```

#### 6.4 Device-side logic

OTA manager checks if `delta.from_version` matches current version. If yes, use `esp_delta_ota` APIs to apply the patch. If no match (device is >1 version behind), fall back to full-image OTA.

---

## File Change Summary

### New Files (6)
| File | Purpose |
|------|---------|
| `packages/robocar/camera/main/ota_manager.h` | Camera OTA orchestration API |
| `packages/robocar/camera/main/ota_manager.c` | MQTT sub, periodic check, download, flash, orchestrate main |
| `packages/robocar/main/main/ota_handler.h` | Main controller OTA handler API |
| `packages/robocar/main/main/ota_handler.c` | I2C OTA command handler, WiFi init, download, flash |
| `.github/workflows/esphome-build.yml` | Reusable ESPHome CI/CD workflow template |
| `docs/decisions/ADR-004-ota-update-architecture.md` | Architecture decision record |

### Modified Files (8)
| File | Change |
|------|--------|
| `packages/robocar/camera/main/config.h` | Add OTA config constants |
| `packages/robocar/camera/main/main.c` | Add `ota_manager_init()`, rollback confirmation |
| `packages/robocar/camera/main/CMakeLists.txt` | Add ota_manager.c, esp_https_ota dep |
| `packages/robocar/camera/sdkconfig.defaults` | Enable rollback + cert bundle |
| `packages/robocar/main/main/system_config.h` | Add OTA config constants |
| `packages/robocar/main/main/main.c` | Add `ota_handler_init()`, rollback confirmation |
| `packages/robocar/main/main/CMakeLists.txt` | Add ota_handler.c, esp_https_ota + wifi deps |
| `.github/workflows/build-firmware.yml` | MQTT notification step, improved manifest.json |

### Potentially Modified (if needed)
| File | Change |
|------|--------|
| `packages/robocar/main/sdkconfig.defaults` | Create if doesn't exist, add OTA + cert configs |
| `packages/robocar/camera/main/Kconfig.projbuild` | Add OTA menu for GitHub repo URL config |
| Both `CMakeLists.txt` (project root) | Add `PROJECT_VER` from version.txt |

---

## Verification Plan

1. **Build verification**: Both projects compile with OTA components (CI catches this)
2. **Unit test**: Version comparison logic (semver parsing)
3. **Integration test (manual)**:
   - Flash v0.1.0 via USB
   - Create GitHub Release with v0.2.0 firmware
   - Verify camera detects update via logs (serial or MQTT)
   - Verify camera downloads and flashes successfully
   - Verify rollback works (flash broken firmware, confirm auto-revert)
   - Verify camera triggers main controller update via I2C
   - Verify main controller downloads and flashes via WiFi
4. **MQTT test**: Publish to `robocar/ota/notify` manually, verify camera responds
5. **Polling test**: Wait for periodic check, verify manifest download

---

## Risk Considerations

1. **Binary size**: OTA partition is 1.8MB. Current builds must stay under this. The `build-firmware.yml` already warns at 1.5MB.
2. **WiFi credentials on main controller**: Must be pre-provisioned via NVS before first OTA. Document this in setup guide.
3. **GitHub rate limits**: Unauthenticated API calls are limited to 60/hour. Polling every 6h from 2 devices = 8 calls/day — well within limits.
4. **MQTT broker exposure**: If broker isn't internet-reachable, MQTT push from CI won't work. Polling fallback handles this gracefully.
5. **Power loss during OTA**: ESP-IDF's dual-partition scheme handles this — incomplete writes don't affect running firmware.
6. **PSRAM availability during OTA**: On ESP32-CAM, OTA must be careful with memory. The `esp_https_ota` component handles streaming without buffering the entire image.
