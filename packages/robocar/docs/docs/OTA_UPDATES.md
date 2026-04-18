# OTA Firmware Updates

This document describes the over-the-air (OTA) firmware update system for the ESP32 AI-Powered Robot Car. The system allows firmware updates without physical USB access by downloading new releases from GitHub.

## Overview

The OTA system uses a **hybrid notification** architecture:

1. **MQTT push** — GitHub Actions publishes to `robocar/ota/notify` after a release, triggering an immediate update check
2. **Periodic polling** — Camera checks GitHub Releases every 6 hours via `esp_ghota`
3. **Manual trigger** — Publish to the MQTT topic to trigger an on-demand check

The camera (ESP32-CAM) acts as the **OTA orchestrator**:
- Updates itself first from GitHub Releases
- After stable boot confirmation (60s), orchestrates main controller update via I2C

The main controller (Heltec WiFi LoRa 32) updates itself:
- Receives release tag from camera via I2C OTA commands
- Initializes WiFi on-demand using NVS-stored credentials
- Downloads firmware directly from GitHub Releases via HTTPS

### Unified Board Variant (robocar-unified)

The [XIAO ESP32-S3 Sense](../../../docs/decisions/ADR-013-single-board-robocar.md) single-board variant uses a simplified OTA flow:
- Self-updates via `esp_ghota` — same mechanism as the camera board
- No I2C orchestration needed (all functionality is on one board)
- 8MB flash with 3.5MB OTA partitions (vs. 1.8MB on 4MB boards)
- Filename match: `robocar-unified` (configured in `config.h`)
- OTA task runs on Core 1 alongside WiFi/MQTT, while motor control stays on Core 0

## How It Works

```
GitHub Release published
        │
        ├──► GitHub Actions publishes MQTT notification
        │         │
        │         ▼
        │    Camera receives MQTT message
        │    on robocar/ota/notify
        │         │
        ▼         ▼
    Camera polls GitHub ──► esp_ghota checks
    every 6 hours            latest release tag
                                   │
                          ┌────────┴────────┐
                          │ New version?    │
                          │   (semver)      │
                          └────────┬────────┘
                                   │ yes
                                   ▼
                          Download camera firmware
                          from GitHub Release asset
                          (robocar-camera.bin)
                                   │
                                   ▼
                          Flash to OTA partition
                          & reboot
                                   │
                                   ▼
                          60s stability timer
                          (rollback if crash)
                                   │
                                   ▼
                          Mark firmware valid
                          (cancel rollback)
                                   │
                                   ▼
                          Query main controller
                          version via I2C
                                   │
                                   ▼
                          Send I2C OTA commands:
                          1. Enter maintenance mode
                          2. Begin OTA (release tag)
                                   │
                                   ▼
                          Main controller:
                          - Init WiFi from NVS
                          - Construct URL from tag
                          - Download & flash via
                            esp_https_ota
                          - Report status via I2C
```

## Rollback Protection

Both boards use ESP-IDF's built-in app rollback mechanism:

- `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` in sdkconfig
- After OTA flash, the new firmware boots from the alternate OTA partition
- A 60-second stability timer runs after boot
- If the device crashes before the timer fires, the bootloader automatically rolls back to the previous firmware on next boot
- Once the timer fires, `esp_ota_mark_app_valid_cancel_rollback()` is called, making the new firmware permanent

## Security

The OTA system enforces several layers of protection:

- **HTTPS only** — `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=n` prevents plaintext firmware downloads. All connections to GitHub use TLS.
- **Certificate bundle** — `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_FULL=y` embeds Mozilla's root CA certificates so the device can verify GitHub's TLS certificate without manual cert management. Adds ~50KB to the binary.
- **Hash verification** — The main controller verifies a SHA256 prefix during I2C-triggered updates to confirm the firmware matches the expected release.
- **Rollback protection** — Dual OTA partitions with a 60-second stability timer provide defense-in-depth against broken firmware (see above).
- **No inbound connections** — Devices pull updates from GitHub; no ports are exposed on the device network.

## Dependencies

| Component | Version | Purpose | Fallback |
|-----------|---------|---------|----------|
| [`esp_ghota`](https://github.com/Fishwaldo/esp_ghota) | `>=0.0.3` | GitHub API integration, semver comparison, periodic polling | Replace with direct `esp_https_ota` + manual GitHub API calls |
| `esp_https_ota` | ESP-IDF built-in | HTTPS firmware download and flash | Core ESP-IDF component, no external dependency |
| `semver` | Bundled with esp_ghota | Semantic version parsing and comparison | — |

`esp_ghota` is pinned in `idf_component.yml` for both `robocar-camera` and `robocar-unified`. If the upstream project becomes unmaintained, the essential functionality (GitHub release checking + download) can be reimplemented using `esp_http_client` + `cJSON` + `esp_https_ota` directly.

## Prerequisites

### WiFi Credentials

Both boards need WiFi credentials stored in NVS before OTA will work:

- **Camera**: Already uses WiFi for AI inference — credentials are loaded from `credentials.h` at compile time and saved to NVS
- **Main controller**: Normally operates without WiFi. For OTA, it uses credentials stored in NVS. These are provisioned during the initial USB flash via the WiFi manager

### Partition Table

Both projects already have OTA-compatible partition tables (`partitions.csv`) with dual `ota_0`/`ota_1` partitions. No changes needed.

### GitHub Repository

The `esp_ghota` component checks GitHub Releases for your repository. The org and repo are configured via Kconfig menuconfig or sdkconfig.defaults:

```
CONFIG_OTA_GITHUB_ORG="laurigates"
CONFIG_OTA_GITHUB_REPO="mcu-tinkering-lab"
```

## Configuration

### Camera (ESP32-CAM)

OTA settings in `robocar-camera/main/config.h`:

| Setting | Default | Description |
|---------|---------|-------------|
| `OTA_ENABLED` | `1` | Enable/disable OTA system |
| `OTA_CHECK_INTERVAL_MIN` | `360` | Polling interval (minutes) |
| `OTA_STABILITY_TIMEOUT_MS` | `60000` | Time before confirming firmware |
| `OTA_FIRMWARE_FILENAME_MATCH` | `"robocar-camera"` | Release asset filename pattern |
| `OTA_MQTT_NOTIFY_TOPIC` | `"robocar/ota/notify"` | MQTT topic for push notifications |
| `OTA_MQTT_STATUS_TOPIC` | `"robocar/ota/status"` | MQTT topic for status reports |

GitHub org/repo are configurable via `idf.py menuconfig` under "OTA Update Configuration".

### Main Controller (Heltec)

OTA settings in `robocar-main/main/system_config.h`:

| Setting | Default | Description |
|---------|---------|-------------|
| `OTA_ENABLED` | `1` | Enable/disable OTA handler |
| `OTA_STABILITY_TIMEOUT_MS` | `60000` | Time before confirming firmware |
| `OTA_GITHUB_ORG` | `"laurigates"` | GitHub org for download URL |
| `OTA_GITHUB_REPO` | `"mcu-tinkering-lab"` | GitHub repo for download URL |
| `OTA_HTTP_TIMEOUT_MS` | `30000` | HTTPS download timeout |

## Version Management

Firmware version is embedded at compile time via `PROJECT_VER` in the project-level `CMakeLists.txt`:

```cmake
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()
```

The `version.txt` files are managed by release-please. When a release is created, release-please bumps the version, and the CI pipeline builds firmware with the new version embedded. `esp_ghota` uses semver comparison to determine if an update is available.

## CI/CD Integration

The `build-firmware.yml` workflow handles the release pipeline:

1. Builds both firmware binaries
2. Generates SHA256 hashes
3. Creates a `manifest.json` with download URLs and checksums
4. Uploads binaries + manifest to the GitHub Release
5. Publishes MQTT notification to `robocar/ota/notify` (if `MQTT_BROKER_HOST` secret is configured)
6. Checks binary sizes against OTA partition limits (1.8MB)

### GitHub Actions Secrets (Optional)

For MQTT push notifications, configure these repository secrets:

| Secret | Description |
|--------|-------------|
| `MQTT_BROKER_HOST` | MQTT broker hostname or IP |
| `MQTT_BROKER_PORT` | MQTT broker port (default: 1883) |

If not configured, devices will still receive updates via periodic polling.

## Testing OTA

### Trigger a Manual Update Check

```bash
# Via MQTT (requires mosquitto-clients)
mosquitto_pub -h <broker> -t "robocar/ota/notify" \
  -m '{"version":"0.2.0","tag":"v0.2.0"}'
```

### Monitor OTA Progress

```bash
# Subscribe to OTA status topic
mosquitto_sub -h <broker> -t "robocar/ota/status" -v
```

### Check Current Firmware Version

The firmware version is logged at boot:
```
I (1234) OTA_Manager: Current firmware version: 0.1.0
```

## Troubleshooting

### OTA update not starting
- Verify WiFi is connected (check serial monitor logs)
- Ensure GitHub is reachable from the device's network
- Check that the release has the correctly named binary (e.g., `robocar-camera.bin`)

### Firmware rolls back after update
- The new firmware crashed within 60 seconds of boot
- Check serial monitor for crash logs before and after the rollback
- The previous firmware version will be restored automatically

### Binary too large for OTA partition
- OTA partition is 1.8MB — the CI pipeline checks this automatically
- If approaching the limit, consider reducing log verbosity, removing unused components, or using `CONFIG_COMPILER_OPTIMIZATION_SIZE`

### Main controller won't update
- Ensure WiFi credentials are stored in NVS (provisioned during initial USB flash)
- Check I2C communication between camera and main controller
- Monitor main controller serial output for OTA handler logs

## Related Documents

- [OTA_DEPLOYMENT.md](OTA_DEPLOYMENT.md) — Operator runbook: provisioning, releasing, monitoring, rollback, and recovery procedures
- [WEB_FLASHER.md](WEB_FLASHER.md) — Browser-based initial flashing (OTA handles subsequent updates)
- [ADR-004: OTA Update Architecture](../../../docs/decisions/ADR-004-ota-update-architecture.md) — Architecture decision with rationale, alternatives, and trade-offs
- [OTA Implementation Plan](../../../docs/prompts/ota-update-implementation-plan.md) — Phase-by-phase implementation plan with status tracking
- [PARTITION_UPDATE_NOTES.md](PARTITION_UPDATE_NOTES.md) — Partition table migration from 6MB to 4MB layout
