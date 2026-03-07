# ADR-004: OTA Firmware Update Architecture

## Status

Accepted

## Context

The robocar uses two ESP32 boards (camera + main controller) that require firmware updates. Currently, updates require physical USB access to each board. As the robot is deployed in environments where USB access is impractical, we need over-the-air (OTA) update capability.

Requirements:
- Update both ESP32 boards without physical access
- Integrate with existing release-please + GitHub Actions CI/CD pipeline
- Auto-update when new releases are published
- Safe rollback if new firmware is broken
- Support future ESPHome projects with the same pattern

The project already had OTA-ready partition tables (dual `ota_0`/`ota_1` partitions), I2C OTA command protocol (0x50-0x54), and a CI pipeline that builds firmware binaries and uploads them to GitHub Releases with a `manifest.json`.

## Decision

### ESP-IDF (Robocar): Hybrid MQTT-Signal + HTTPS-Pull + Periodic Polling

We use a three-layer update notification system:

1. **MQTT push** (immediate): GitHub Actions publishes to `robocar/ota/notify` after a release. Camera receives notification and triggers immediate update check.
2. **Periodic polling** (6-hour fallback): Camera uses `esp_ghota` component to periodically check GitHub Releases for new versions. Works even if MQTT notification fails.
3. **Manual trigger**: Update check can be triggered via MQTT command.

**Update flow:**
- Camera (ESP32-CAM) acts as OTA orchestrator — it has WiFi and network connectivity
- Camera updates itself first using `esp_ghota` (GitHub OTA component) + `esp_https_ota`
- After successful self-update and stability confirmation, camera orchestrates main controller update via I2C OTA commands
- Main controller initializes WiFi on-demand (from NVS credentials) to download its own firmware directly from GitHub Releases

**Key components:**
- `esp_ghota` (fishwaldo/esp_ghota): Handles GitHub API integration, semver comparison, periodic timers, and filename matching for release assets
- `esp_https_ota`: ESP-IDF built-in for HTTPS firmware download and flash
- ESP-IDF certificate bundle: Handles GitHub TLS certificates without manual cert management
- App rollback: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` with 60-second stability timer

**I2C URL limitation workaround:** The existing `ota_begin_data_t` has only 20 bytes for URL — too short for GitHub URLs. Instead of sending full URLs over I2C, the camera sends the release tag (e.g., "v0.2.0"). The main controller constructs the full download URL from a compile-time base pattern.

### ESPHome (Future Projects): HTTP Request OTA with Periodic Polling

Future ESPHome devices will use the `http_request` OTA platform:
- Device periodically fetches firmware from `releases/latest/download/` URL
- MD5 verification for integrity
- GitHub Actions workflow template (`esphome-build.yml`) builds firmware and uploads to releases

## Alternatives Considered

### 1. MQTT Binary Transfer
Sending firmware binary over MQTT. Rejected because:
- MQTT is designed for small messages, not 1.8MB binary transfers
- Would overload the broker and require chunking logic
- No resume capability on failure

### 2. Camera Proxying Firmware to Main Controller over I2C/UART
Having the camera download main controller firmware and relay it byte-by-byte. Rejected because:
- I2C at 100kHz would take ~30 minutes for 1.8MB
- UART would be faster but still slow and complex
- Main controller already has WiFi hardware — using it directly is simpler

### 3. Custom Manifest Server
Running a dedicated server for version checking. Rejected because:
- GitHub Releases already provides stable URLs (`releases/latest/download/`)
- `esp_ghota` handles GitHub API integration out of the box
- No additional infrastructure to maintain

### 4. Delta OTA (esp_delta_ota)
Sending only binary diffs between versions (~95% size reduction). Deferred to Phase 6 because:
- Adds complexity (each patch only works from one specific base version)
- Full-image OTA should be proven first
- Can be added later as the manifest already supports both `url` and `delta` fields

## Consequences

### Positive
- Firmware updates without physical access
- Automatic updates tied to release-please versioning
- Safe rollback protection via dual OTA partitions
- Works behind firewalls (device pulls, no inbound connections needed)
- MQTT push provides fast notification; polling provides reliable fallback
- Reusable pattern for future ESPHome projects

### Negative
- Main controller now uses WiFi (was previously WiFi-free during normal operation) — only during OTA
- Depends on GitHub availability for downloads (mitigated by retry logic)
- `esp_ghota` adds a third-party dependency
- Certificate bundle increases binary size (~50KB)

### Risks
- Binary size: OTA partition is 1.8MB. Certificate bundle and OTA code add ~50-100KB. CI now checks binary sizes with warning/critical thresholds.
- WiFi credentials on main controller must be pre-provisioned via NVS before first OTA
- GitHub API rate limits: 60 requests/hour unauthenticated. Polling every 6h from 2 devices = 8 calls/day — well within limits.

## Implementation

| Component | File | Purpose |
|-----------|------|---------|
| `ota_manager` | `robocar-camera/main/ota_manager.c` | Camera OTA orchestration: esp_ghota, MQTT, I2C orchestration |
| `ota_handler` | `robocar-main/main/ota_handler.c` | Main controller: WiFi-on-demand, esp_https_ota |
| `i2c_slave` | `robocar-main/main/i2c_slave.c` | Updated OTA command handlers (0x50-0x54) |
| `build-firmware.yml` | `.github/workflows/` | MQTT notification, improved manifest.json |
| `esphome-build.yml` | `.github/workflows/` | ESPHome CI/CD template |
