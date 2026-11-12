---
reviewed: 2026-11-03
paths:
  - 'packages/**/*wifi*'
  - 'packages/**/main/main.{c,cpp}'
  - 'packages/**/main/CMakeLists.txt'
  - '**/sdkconfig.defaults'
---
# mDNS Hostname Configuration

## Every WiFi Project Must Have mDNS

All ESP32 projects that connect to WiFi in STA mode MUST configure mDNS so the device is discoverable on the local network via `<hostname>.local`.

## Required Steps for New WiFi Projects

1. **Include header**: `#include "mdns.h"` in the file that initializes WiFi
2. **Add CMake dependency**: Add `mdns` to `REQUIRES` in the main `CMakeLists.txt`
3. **Declare the managed component**: add `espressif/mdns: '^1.8.0'` under
   `dependencies:` in `main/idf_component.yml`. In ESP-IDF 5.x mDNS is a managed
   component, not part of the framework; without the entry `mdns.h` is not found
   (brainbox, #671). It has no enable switch: there is no `CONFIG_MDNS_ENABLED`,
   and ESP-IDF ignores such a line with only an "unknown kconfig symbol" build
   warning (#707). The `kconfig-symbols` pre-commit hook now rejects it
4. **Initialize after WiFi connects**:
   ```c
   ESP_ERROR_CHECK(mdns_init());
   ESP_ERROR_CHECK(mdns_hostname_set("<project-name>"));
   ESP_ERROR_CHECK(mdns_instance_name_set("<Human-Readable Name>"));
   ESP_LOGI(TAG, "mDNS initialized: <project-name>.local");
   ```
5. **Advertise services** if the device runs a server:
   ```c
   mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
   ```

## Hostname Convention

Use the project directory name as the hostname (e.g., `esp32-cam-webserver`, `robocar`, `it-troubleshooter`). This keeps hostnames predictable and consistent.

## Placement

Initialize mDNS **after** WiFi has connected (after `IP_EVENT_STA_GOT_IP` or equivalent), but **before** starting application services.

## Current Project Hostnames

| Project | Hostname | URL |
|---------|----------|-----|
| esp32-cam-webserver | `esp32-cam` | `http://esp32-cam.local` |
| esp32cam-llm-telegram | `esp32cam-llm-telegram` | `esp32cam-llm-telegram.local` |
| robocar | `robocar` | `robocar.local` |
| nfc-scavenger-hunt | `nfc-scavenger-hunt` | `nfc-scavenger-hunt.local` |
| it-troubleshooter | `it-troubleshooter` | `it-troubleshooter.local` |
