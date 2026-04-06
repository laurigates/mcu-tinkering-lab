# mDNS Hostname Configuration

## Every WiFi Project Must Have mDNS

All ESP32 projects that connect to WiFi in STA mode MUST configure mDNS so the device is discoverable on the local network via `<hostname>.local`.

## Required Steps for New WiFi Projects

1. **Include header**: `#include "mdns.h"` in the file that initializes WiFi
2. **Add CMake dependency**: Add `mdns` to `REQUIRES` in the main `CMakeLists.txt`
3. **Enable in sdkconfig**: Add `CONFIG_MDNS_ENABLED=y` to `sdkconfig.defaults`
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

Use the project directory name as the hostname (e.g., `esp32-cam-webserver`, `robocar-main`, `it-troubleshooter`). This keeps hostnames predictable and consistent.

## Placement

Initialize mDNS **after** WiFi has connected (after `IP_EVENT_STA_GOT_IP` or equivalent), but **before** starting application services.

## Current Project Hostnames

| Project | Hostname | URL |
|---------|----------|-----|
| esp32-cam-webserver | `esp32-cam` | `http://esp32-cam.local` |
| esp32cam-llm-telegram | `esp32cam-llm-telegram` | `esp32cam-llm-telegram.local` |
| robocar-main | `robocar-main` | `robocar-main.local` |
| robocar-camera | `robocar-camera` | `robocar-camera.local` |
| nfc-scavenger-hunt | `nfc-scavenger-hunt` | `nfc-scavenger-hunt.local` |
| it-troubleshooter | `it-troubleshooter` | `it-troubleshooter.local` |
