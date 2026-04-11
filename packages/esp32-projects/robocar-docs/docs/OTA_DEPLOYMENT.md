# OTA Deployment Runbook

This document provides step-by-step procedures for deploying and maintaining over-the-air (OTA) firmware updates in production. It is an **operator runbook** — practical guidance for managing OTA operations. For a description of how the OTA system works, see [OTA_UPDATES.md](OTA_UPDATES.md).

## 1. Initial Board Provisioning

### 1.1 Flash Firmware via USB (First Time)

Both boards must be flashed with firmware via USB before OTA updates can work.

#### Camera Board (ESP32-CAM)

1. Connect ESP32-CAM to your machine via USB (using CH340 or similar adapter)
2. From the repo root, build the camera firmware:
   ```bash
   just robocar::build-cam
   ```
3. Flash the firmware:
   ```bash
   PORT=/dev/cu.usbserial-0001 just robocar::flash-cam
   ```
   (adjust PORT if your USB device is different)
4. Monitor startup logs:
   ```bash
   PORT=/dev/cu.usbserial-0001 just robocar::monitor-cam
   ```
   Look for `OTA_Manager: Current firmware version: X.Y.Z`

#### Main Controller (Heltec WiFi LoRa 32)

1. Connect the Heltec board to your machine via USB
2. From the repo root, build the main controller firmware:
   ```bash
   just robocar::build-main
   ```
3. Flash the firmware:
   ```bash
   PORT=/dev/cu.usbserial-0002 just robocar::flash-main
   ```
4. Monitor startup logs:
   ```bash
   PORT=/dev/cu.usbserial-0002 just robocar::monitor-main
   ```

### 1.2 WiFi Credential Provisioning

#### Camera Board

The camera board loads WiFi credentials from `credentials.h` at compile time. These credentials are compiled into the firmware binary and automatically saved to NVS (non-volatile storage) during first boot.

**Prerequisite**: Before building the camera firmware, create the credentials file:
```bash
cat > packages/esp32-projects/robocar-camera/main/credentials.h << 'EOF'
#ifndef CREDENTIALS_H
#define CREDENTIALS_H

#define WIFI_SSID "your-ssid"
#define WIFI_PASSWORD "your-password"
#define MQTT_BROKER_URI "mqtt://192.168.0.100:1883"

#endif // CREDENTIALS_H
EOF
```

Note: `credentials.h` is gitignored and must never be committed. The file should contain only sensitive values used at compile time.

#### Main Controller

The main controller stores WiFi credentials in NVS (non-volatile storage) and uses a WiFi manager for provisioning during initial flash. The WiFi manager is available during the first 30 seconds after boot if no valid SSID is stored.

**To provision WiFi on first boot:**

1. Flash the firmware (see section 1.1 above)
2. During the first 30 seconds after boot, the WiFi manager enters **configuration mode**:
   - Look for an SSID starting with `RoboCar-` in your WiFi network list
   - Connect to it with password `12345678` (default, changeable in `system_config.h`)
3. Once connected to the config portal, you'll see a captive portal for entering your WiFi credentials
4. Enter your home WiFi SSID and password
5. Submit and wait for the board to connect to your home network
6. The credentials are now stored in NVS and will persist across power cycles

**To re-provision WiFi later:**
- Press the WiFi reset button on the board (gpio pin varies by Heltec revision)
- Or reflash via USB and repeat the WiFi manager process

### 1.3 Verify OTA Readiness

Before OTA updates are deployed, verify that both boards are OTA-ready:

#### Check Partition Table

```bash
# On camera board
idf.py -p /dev/cu.usbserial-0001 partition_table
```

Expected output should show two OTA partitions (`ota_0` and `ota_1`), each sized for the OTA binary (typically 1.8MB).

#### Verify esp_ghota Configuration

The esp_ghota component checks the Kconfig settings for GitHub org and repo:

```bash
# Build with verbose output to see config
idf.py -p /dev/cu.usbserial-0001 build | grep -i "ota_github"
```

Expected output:
```
CONFIG_OTA_GITHUB_ORG="laurigates"
CONFIG_OTA_GITHUB_REPO="mcu-tinkering-lab"
```

If these are incorrect, edit `sdkconfig.defaults` in the respective project and rebuild.

#### Test MQTT Connectivity (Optional)

If MQTT is configured for push notifications, verify the camera can reach the MQTT broker:

```bash
# Monitor camera logs
PORT=/dev/cu.usbserial-0001 idf.py monitor | grep -i "mqtt"
```

Expected: `MQTT_Client: Connected to broker`

---

## 2. Creating a Release (Triggering OTA)

### 2.1 Semantic Versioning via Conventional Commits

The release workflow uses **release-please** to automatically generate release PRs from conventional commits:

- `feat:` — Minor version bump (v0.1.0 → v0.2.0)
- `fix:` — Patch version bump (v0.2.0 → v0.2.1)
- `BREAKING CHANGE:` footer — Major version bump (v0.1.0 → v1.0.0)

Examples:
```bash
git commit -m "feat(camera): add night vision mode"          # → v0.2.0
git commit -m "fix(ota): retry on network timeout"          # → v0.2.1
git commit -m "BREAKING CHANGE: reformat I2C protocol"      # → v1.0.0
```

### 2.2 Creating a Release

The release workflow is **fully automated** via GitHub Actions:

1. **Conventional commits** accumulate on the `main` branch
2. **release-please** automatically creates a release PR when triggered by GitHub's scheduled actions (or manually via Actions tab)
3. Review and merge the release PR (it only updates version files and creates a GitHub Release)
4. **GitHub Release triggers CI** — the `build-firmware.yml` workflow automatically:
   - Builds both firmware binaries
   - Generates SHA256 checksums
   - Uploads binaries to the release assets
   - Sends MQTT notification to the devices (if `MQTT_BROKER_HOST` secret is set)

**To manually trigger a release PR:**

1. Go to **Actions** → **release-please** workflow
2. Click **Run workflow** and select `main` branch
3. release-please will create a PR with version bumps and changelog
4. Review, approve, and merge the PR
5. GitHub automatically creates the Release (which triggers CI)

### 2.3 What Happens Automatically

Once a GitHub Release is published:

1. **CI workflow (`build-firmware.yml`) runs:**
   - Builds `robocar-camera.bin` and `robocar-main.bin`
   - Calculates SHA256 hashes
   - Creates `manifest.json` with download URLs and checksums
   - Uploads all three files to the release assets
   - **Binary size check**: verifies each binary ≤ 1.8MB (OTA partition limit)
   - If `MQTT_BROKER_HOST` secret is configured, publishes to `robocar/ota/notify`

2. **Devices receive notification:**
   - Camera receives MQTT push message (immediate)
   - Camera begins download and flash
   - Main controller is orchestrated by camera after 60s stability check

### 2.4 Manual Trigger (Without GitHub Release)

If you need to test OTA without creating a full release, you can manually publish an MQTT notification:

```bash
# Replace <broker-ip> with your MQTT broker IP
mosquitto_pub -h <broker-ip> -t "robocar/ota/notify" \
  -m '{"version":"0.2.0","tag":"v0.2.0"}'
```

The devices will attempt to download firmware from `https://github.com/laurigates/mcu-tinkering-lab/releases/download/v0.2.0/robocar-camera.bin` (and similarly for main controller).

---

## 3. GitHub Secrets Setup

OTA notifications require the GitHub Actions workflow to publish to your MQTT broker. Configure these **repository secrets** to enable push notifications:

### Optional Secrets

| Secret | Example | Purpose |
|--------|---------|---------|
| `MQTT_BROKER_HOST` | `192.168.0.100` or `mqtt.example.com` | MQTT broker hostname or IP for push notifications |
| `MQTT_BROKER_PORT` | `1883` | MQTT broker port (default: 1883 if not set) |

To configure:
1. Go to **Settings** → **Secrets and variables** → **Actions**
2. Click **New repository secret**
3. Name: `MQTT_BROKER_HOST`, Value: your broker IP/hostname
4. (Optional) Add `MQTT_BROKER_PORT` if using non-standard port

If secrets are not configured, devices will still receive updates via **periodic polling** (camera checks every 6 hours).

### Compiled Configuration

GitHub org and repo are **compiled into the firmware via Kconfig** and do NOT use secrets:

```
CONFIG_OTA_GITHUB_ORG="laurigates"
CONFIG_OTA_GITHUB_REPO="mcu-tinkering-lab"
```

These are set in `sdkconfig.defaults` in each project and are read-only at runtime.

---

## 4. Monitoring OTA Progress

### 4.1 Serial Monitor

Watch the boards during OTA operations:

```bash
# Camera board
PORT=/dev/cu.usbserial-0001 idf.py monitor

# Main controller board
PORT=/dev/cu.usbserial-0002 idf.py monitor
```

Key log messages to watch for:

**Camera (esp_ghota and OTA_Manager):**
```
I (12345) esp_ghota: Checking for updates...
I (23456) esp_ghota: New version available: 0.2.0
I (34567) esp_ghota: Downloading robocar-camera.bin...
I (45678) OTA_Manager: OTA firmware flashed successfully
I (56789) OTA_Manager: Stability timer started (60s)
I (67890) OTA_Manager: Firmware validated, rollback cancelled
```

**Main Controller (OTA_Handler and esp_https_ota):**
```
I (12345) OTA_Handler: Received OTA command: BEGIN_OTA
I (23456) OTA_Handler: WiFi init complete
I (34567) esp_https_ota: Starting HTTPS OTA...
I (45678) esp_https_ota: Downloaded 1234567 bytes
I (56789) OTA_Handler: OTA complete, rebooting...
I (67890) OTA_Manager: Stability timer started (60s)
```

### 4.2 MQTT Status Topic

Subscribe to the status topic to monitor OTA updates remotely:

```bash
mosquitto_sub -h <broker-ip> -t "robocar/ota/status" -v
```

Sample status messages:
```
robocar/ota/status {"device":"camera","status":"checking","timestamp":"2024-01-15T12:34:56Z"}
robocar/ota/status {"device":"camera","status":"downloading","version":"0.2.0","timestamp":"2024-01-15T12:34:57Z"}
robocar/ota/status {"device":"camera","status":"complete","version":"0.2.0","timestamp":"2024-01-15T12:35:30Z"}
robocar/ota/status {"device":"main","status":"begin_ota","version":"0.2.0","timestamp":"2024-01-15T12:35:45Z"}
robocar/ota/status {"device":"main","status":"complete","version":"0.2.0","timestamp":"2024-01-15T12:37:15Z"}
```

### 4.3 Expected Timeline

Typical OTA deployment timeline (assuming both devices are running and connected):

| Event | Time | Duration |
|-------|------|----------|
| MQTT notification published | T+0s | — |
| Camera checks GitHub | T+1s | ~5s |
| Camera downloads firmware | T+6s | ~90s |
| Camera flashes and reboots | T+96s | ~10s |
| Camera stability timer | T+106s | 60s |
| Main controller receives OTA command | T+166s | — |
| Main controller initializes WiFi | T+167s | ~15s |
| Main controller downloads firmware | T+182s | ~90s |
| Main controller flashes and reboots | T+272s | ~10s |
| Main controller stability timer | T+282s | 60s |
| **OTA Complete** | T+342s | **~5.7 minutes total** |

---

## 5. Rollback Procedure

The OTA system automatically rolls back to the previous firmware if the new firmware crashes before confirming stability. Rollback can also be triggered manually.

### 5.1 Automatic Rollback

**How it works:**

1. New firmware boots from the alternate OTA partition
2. 60-second stability timer starts (`OTA_STABILITY_TIMEOUT_MS`)
3. If the firmware crashes before the timer expires, the bootloader detects the crash on next power-on
4. Bootloader automatically boots the previous firmware from the other OTA partition
5. Once the stability timer completes, `esp_ota_mark_app_valid_cancel_rollback()` is called, making the new firmware permanent

**To observe rollback:**

```bash
# Monitor logs during and after OTA
idf.py monitor

# After rollback, you'll see:
# I (1234) boot: OTA app rollback detected
# I (5678) OTA_Manager: Current firmware version: 0.1.0 (reverted)
```

### 5.2 Manual Rollback

To roll back to a previous firmware version manually, reflash via USB:

```bash
# Flash the previous firmware binary
idf.py -p /dev/cu.usbserial-0001 --app-bin robocar-camera-v0.1.0.bin flash
```

Alternatively, you can revert in firmware using the partition API:

```c
#include "esp_ota_ops.h"

esp_err_t err = esp_ota_mark_rollback();
if (err == ESP_OK) {
    esp_restart();  // Will boot previous firmware on next startup
}
```

### 5.3 Checking Rollback State

To check if the device has rolled back or if rollback is pending:

```bash
# Monitor logs at startup
idf.py monitor | head -50
```

Look for these messages:

```
I (1234) boot: OTA app rollback detected       # Rollback just occurred
I (5678) OTA_Manager: Rollback cancelled       # New firmware confirmed stable
I (9012) OTA_Manager: Rollback pending         # New firmware still in confirmation period
```

---

## 6. Recovery Procedures

### 6.1 Board Won't Boot After Failed OTA

**Symptom:** Device reboots repeatedly or doesn't start after OTA attempt.

**Cause:** OTA partition corruption or bootloader crash.

**Recovery:**

1. Hold the **BOOT** button on the device (GPIO0)
2. Power cycle the device (unplug and replug USB)
3. Connect to your machine via USB
4. Reflash the previous known-good firmware:
   ```bash
   idf.py -p /dev/cu.usbserial-0001 --app-bin previous-version.bin flash
   ```
5. Release the BOOT button
6. Verify startup: `idf.py monitor`

### 6.2 Main Controller Stuck in Maintenance Mode

**Symptom:** Main controller doesn't respond to I2C OTA commands or remains in maintenance mode after OTA attempt.

**Cause:** The maintenance mode flag (`s_maintenance_mode`) in the OTA orchestration loop is not being reset.

**Recovery:**

1. Power cycle the main controller board (unplug and replug power)
2. The maintenance mode flag is reset on boot
3. Verify normal operation: check logs for I2C communication

If the issue persists, reflash the main controller firmware via USB.

### 6.3 WiFi Not Connecting for OTA

**Symptom:** OTA fails on the main controller because WiFi won't connect.

**Causes and fixes:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| `WiFi credentials invalid` error | NVS credentials are corrupted or wrong | Re-provision WiFi (see section 1.2) |
| `AP not found` error | Router SSID is not broadcasting | Ensure SSID is visible to devices |
| `Auth failed` error | Wrong password stored in NVS | Re-provision WiFi with correct password |
| `IP assignment timeout` error | Router DHCP is not responding | Restart router, check DHCP range |

**To verify NVS credentials:**

```bash
# Connect to main controller serial monitor
idf.py monitor

# Look for WiFi init logs:
# I (1234) WiFi_Manager: SSID from NVS: my-ssid
# I (2345) WiFi_Manager: Connecting to my-ssid...
# I (3456) WiFi_Manager: Connected! IP: 192.168.1.100
```

**To re-provision:**

1. Press the WiFi reset button on the main controller (or reflash and trigger WiFi manager within first 30s)
2. Connect to the RoboCar AP
3. Enter correct WiFi credentials
4. Power cycle and verify connection

### 6.4 I2C Communication Failure During Orchestration

**Symptom:** Camera logs show "I2C write failed" or main controller doesn't receive OTA command.

**Cause:** I2C wiring loose or I2C addresses misconfigured.

**Recovery:**

1. **Check wiring:**
   - Camera SDA (GPIO 15) → Main controller SDA (GPIO 26)
   - Camera SCL (GPIO 14) → Main controller SCL (GPIO 27)
   - Ground → Ground (shared)

2. **Verify I2C addresses:**
   - Camera: master (no slave address)
   - Main controller: slave at address `0x42` (configurable in `i2c_protocol.h`)

3. **Check clock speed:**
   - Default is 100kHz; can be increased to 400kHz if wiring is short
   - Configured in `i2c_protocol.h`: `I2C_MASTER_FREQ_HZ`

4. **Test I2C manually:**
   ```bash
   # From main controller
   idf.py monitor | grep -i "i2c"
   ```
   Look for: `I2C_Handler: I2C initialized, slave address 0x42`

5. **Power cycle both devices** and retry OTA

---

## 7. Binary Size Limits

Each firmware binary must fit within the OTA partition, which is limited to **1.8MB** (1,887,436 bytes).

### 7.1 Check Binary Size

After building, the CI workflow automatically checks binary sizes:

```bash
# After build, check locally
ls -lh build/robocar-camera.bin build/robocar-main.bin

# Expected output should show binaries < 1.8MB
# Example:
# -rw-r--r--  1 user  group  1.2M  Jan 15 12:34 robocar-camera.bin
# -rw-r--r--  1 user  group  950K  Jan 15 12:34 robocar-main.bin
```

### 7.2 If Binary Exceeds Limit

If a binary approaches or exceeds 1.8MB:

1. **Reduce log verbosity:**
   ```c
   // In main/config.h or system_config.h
   #define LOG_LOCAL_LEVEL ESP_LOG_WARN  // Instead of ESP_LOG_DEBUG
   ```

2. **Enable size optimization:**
   ```bash
   # Edit sdkconfig.defaults and add:
   CONFIG_COMPILER_OPTIMIZATION_SIZE=y
   ```

3. **Remove unused components:**
   - Edit `CMakeLists.txt` and comment out unused REQUIRES
   - Example: remove `esp_http_server` if not used

4. **Check component bloat:**
   ```bash
   idf.py build --verbose
   # Look for large `.o` files in the build log
   ```

5. **Rebuild and verify:**
   ```bash
   rm -rf build
   idf.py build
   ls -lh build/robocar-camera.bin
   ```

---

## 8. MQTT Broker Setup

The OTA system can push notifications to your MQTT broker when a release is published. A development Mosquitto broker is included in the robocar-docs Makefile.

### 8.1 Development Stack (Local Testing)

Start the included Mosquitto broker:

```bash
cd packages/esp32-projects/robocar-docs
make mosquitto-start
```

This starts Mosquitto on `localhost:1883` with mDNS advertisement at `mosquitto.local`.

**To test MQTT locally:**

```bash
# In one terminal, subscribe to status
mosquitto_sub -h localhost -t "robocar/ota/status"

# In another, trigger a manual update check
mosquitto_pub -h localhost -t "robocar/ota/notify" \
  -m '{"version":"0.2.0","tag":"v0.2.0"}'
```

### 8.2 Production MQTT Broker

For production, you need a persistently running MQTT broker accessible from:
- The robot's WiFi network (for devices to receive notifications)
- GitHub Actions runners (for CI/CD to publish notifications)

**Popular options:**

| Broker | Setup | Cost | Notes |
|--------|-------|------|-------|
| **Mosquitto** (self-hosted) | `docker run -d eclipse-mosquitto` | Free | Full control, requires server |
| **HiveMQ Cloud** | Managed service | Free tier available | Cloud-hosted, easiest setup |
| **Azure IoT Hub** | Azure subscription | Pay-per-message | Enterprise, more features |

**Example: Docker Mosquitto**

```bash
docker run -d --name mosquitto \
  -p 1883:1883 \
  -v /path/to/mosquitto.conf:/mosquitto/config/mosquitto.conf \
  eclipse-mosquitto
```

### 8.3 Configure GitHub Actions Secrets

To enable MQTT notifications in GitHub Actions:

1. **Determine your broker hostname/IP:**
   - Self-hosted: your server's IP or domain name
   - Cloud service: connection string from provider

2. **Go to GitHub:**
   - Repo → Settings → Secrets and variables → Actions

3. **Add secrets:**
   - `MQTT_BROKER_HOST`: your broker hostname/IP
   - `MQTT_BROKER_PORT`: port number (optional, defaults to 1883)

4. **Verify in workflow:**
   ```bash
   # Check that build-firmware.yml has:
   if: secrets.MQTT_BROKER_HOST != ''
   ```

### 8.4 Firewall / Network Considerations

- **Incoming from devices:** Port 1883 (MQTT) must be reachable from robot's WiFi network
- **Outgoing from GitHub:** Port 1883 must be reachable from GitHub Actions IP ranges
- **TLS/SSL:** Standard MQTT is unencrypted; use MQTT over TLS (port 8883) for production

---

## End-to-End Verification Checklist

Use this checklist to verify the full OTA pipeline after initial setup or major changes.

### Prerequisites
- [ ] Both boards flashed via USB with OTA-enabled firmware
- [ ] WiFi credentials provisioned on both boards (camera via `credentials.h`, main via WiFi manager)
- [ ] MQTT broker running and reachable from both boards
- [ ] A GitHub Release exists with correctly named binaries (`robocar-camera.bin`, `robocar-main.bin`)

### Camera Self-Update
- [ ] Trigger update check: `mosquitto_pub -h <broker> -t "robocar/ota/notify" -m '{"version":"<new>","tag":"v<new>"}'`
- [ ] Camera logs show `esp_ghota: New version available`
- [ ] Camera downloads and flashes firmware (monitor progress via serial or MQTT status topic)
- [ ] Camera reboots and logs `Current firmware version: <new>`
- [ ] Stability timer completes (60s) and logs `Firmware marked as valid`

### Main Controller Update (Orchestrated by Camera)
- [ ] Camera queries main controller version via I2C
- [ ] Camera sends `CMD_TYPE_ENTER_MAINTENANCE_MODE` (motors stop)
- [ ] Camera sends `CMD_TYPE_BEGIN_OTA` with release tag
- [ ] Main controller initializes WiFi and downloads firmware
- [ ] Main controller logs OTA progress and reboots
- [ ] Main controller stability timer completes (60s)

### Rollback Verification
- [ ] Flash a known-broken firmware (e.g., one that crashes within 10s)
- [ ] Confirm bootloader detects crash and reverts to previous partition
- [ ] Logs show `OTA app rollback detected` after revert
- [ ] Device resumes normal operation on previous firmware version

### Polling Fallback
- [ ] Disable MQTT broker (or disconnect device from MQTT)
- [ ] Wait for periodic poll interval (or reduce `OTA_CHECK_INTERVAL_MIN` for testing)
- [ ] Confirm camera detects update via polling alone

---

## Troubleshooting Checklist

| Problem | Check | Solution |
|---------|-------|----------|
| OTA won't start | Device WiFi connected? | Check `idf.py monitor` for WiFi logs |
| | Correct GitHub org/repo? | Verify `CONFIG_OTA_GITHUB_*` in sdkconfig |
| | Release has binaries? | Check GitHub Release assets |
| Firmware rolls back | Logs show crash? | Review `idf.py monitor` output before rollback |
| | 60s stability timer expired? | Check `OTA_STABILITY_TIMEOUT_MS` config |
| MQTT push doesn't work | GitHub secrets set? | Verify `MQTT_BROKER_HOST` and `PORT` secrets |
| | Broker reachable? | Test: `nc -zv <broker-ip> 1883` |
| | Firewall blocking? | Ensure port 1883 (or custom port) is open |
| Board won't boot | Bootloop after OTA? | Reflash via USB with BOOT button held |
| | Partition table corrupted? | Run `idf.py erase-flash` then reflash |
| | Previous version lost? | Download and reflash from GitHub Releases |

---

## References

- [OTA_UPDATES.md](OTA_UPDATES.md) — System design and architecture
- [ADR-004: OTA Update Architecture](../../../docs/blueprint/adrs/ADR-004-ota-update-architecture.md) — Design rationale
- [esp_ghota component](https://github.com/fishwaldo/esp_ghota) — GitHub releases client
- [esp_ota_ops.h API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html) — OTA partition management
