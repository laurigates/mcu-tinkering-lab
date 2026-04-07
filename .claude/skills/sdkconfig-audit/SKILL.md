---
name: sdkconfig-audit
description: Audit sdkconfig.defaults by inferring needed settings from source code analysis
argument-hint: "<project-path>"
user-invocable: true
allowed-tools: Read, Grep, Glob
---

## Task

Audit `sdkconfig.defaults` for the project at `$1` (relative path under `packages/esp32-projects/`).

## Process

1. **Read source files** (`main/*.c`, `main/*.h`) to detect feature usage
2. **Map features to required sdkconfig entries** using the table below
3. **Read existing `sdkconfig.defaults`** (if it exists)
4. **Compare and report** missing, unnecessary, or default-reliant settings

## Feature Detection → Config Mapping

| Source Pattern | Required Config | Notes |
|---------------|----------------|-------|
| `#include "esp_wifi.h"` or `esp_wifi_*` calls | `CONFIG_ESP_WIFI_*` basics | WiFi subsystem |
| `#include "esp_bt.h"` or BLE/BT usage | `CONFIG_BT_ENABLED=y` | Bluetooth |
| `CONFIG_BT_NIMBLE_ENABLED` vs `CONFIG_BT_BLUEDROID_ENABLED` | Depends on stack used | Check for NimBLE vs Bluedroid imports |
| `#include "mdns.h"` or `mdns_init()` | `CONFIG_MDNS_ENABLED=y` | mDNS discovery |
| `#include "esp_camera.h"` | `CONFIG_CAMERA_*`, PSRAM settings | Camera module |
| `#include "esp_tls.h"` or HTTPS client | `CONFIG_ESP_TLS_*` | TLS/HTTPS |
| `#include "esp_https_ota.h"` | OTA partition table required | OTA updates |
| `xTaskCreate` with large stacks or many tasks | `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` | Stack sizing |
| WiFi + BLE + USB init in `app_main` | `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192`+ | Multiple subsystem init |
| `#include "tinyusb.h"` or USB device | `CONFIG_TINYUSB_*` | USB device stack |
| `esp_deep_sleep_start()` | Deep sleep related configs | Power management |
| `led_strip_*` or WS2812 | RMT channel config | LED strip driver |

## Also Check

- **Target chip**: `CONFIG_IDF_TARGET` should match justfile `target` variable
- **Flash size**: `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y` (or appropriate size)
- **Log level**: `CONFIG_LOG_DEFAULT_LEVEL_INFO=y` (recommended for production)
- **Watchdog**: `CONFIG_ESP_TASK_WDT_TIMEOUT_S` if project has long-running operations
- **Partition table**: Custom `CONFIG_PARTITION_TABLE_CUSTOM=y` if `partitions.csv` exists

## Output Format

```
## sdkconfig.defaults Audit: <project-name>

### Missing (should add)
| Setting | Reason | Suggested Value |
|---------|--------|-----------------|
| CONFIG_MDNS_ENABLED | Source uses mdns_init() | y |

### Present (looks good)
| Setting | Value | Notes |
|---------|-------|-------|
| CONFIG_IDF_TARGET | "esp32s3" | Matches justfile target |

### Possibly unnecessary
| Setting | Reason |
|---------|--------|
| CONFIG_BT_ENABLED | No Bluetooth usage found in source |

### Recommendations
- <actionable suggestions>
```

## Style Rules

- **Only report actionable findings** — don't list every possible sdkconfig option
- **Link to source evidence** — show which file/line triggered the detection
- **Don't modify files** — this skill is read-only audit; suggest changes for the user to apply
