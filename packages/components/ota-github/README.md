# `ota_github` — GitHub-Releases OTA component for ESP-IDF

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-%E2%89%A55.1-red)](https://docs.espressif.com/projects/esp-idf/)
[![License](https://img.shields.io/badge/license-MIT-blue)](#license)

**Drop-in, project-agnostic OTA firmware updates from GitHub Releases.** Add
the component to `EXTRA_COMPONENT_DIRS`, call `ota_github_init(&cfg)` once,
and your device will pull new firmware on a timer (or on an MQTT push, or
when an external controller triggers it over I2C). SHA256 verification,
rollback protection, and esp_event-based progress reporting are all handled
for you.

---

## Feature matrix

| Feature | PULL mode | TRIGGERED mode |
|---|:---:|:---:|
| Periodic GitHub poll (configurable interval) | ✔ | — |
| Semver comparison of running vs latest release | ✔ | — |
| External URL / tag trigger (I2C, UART, HTTP, ...) | ✔ | ✔ |
| MQTT push-notify (rate-limited) | ✔ | ✔ |
| `esp_https_ota` download + flash | ✔ | ✔ |
| 4-byte SHA256 prefix verification | (via esp_ghota) | ✔ |
| Rollback cancel after stability window | ✔ | ✔ |
| Thread-safe status / progress / error getters | ✔ | ✔ |
| `esp_event` bus for async observers | ✔ | ✔ |
| Project-specific hooks (`pre_download`, `ready_to_reboot`, ...) | ✔ | ✔ |

Either mode can be combined with MQTT push-notify. PULL mode is the common
case for a single connected device. TRIGGERED mode is for devices whose
update is orchestrated by a peer (e.g. a dual-MCU setup where one board
has the internet connection).

---

## 30-second adoption

```cmake
# your-project/CMakeLists.txt
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
```

```cmake
# your-project/main/CMakeLists.txt
idf_component_register(
    SRCS "main.c"
    REQUIRES "ota_github"     # pulls in esp_ghota transitively
)
```

```c
// main.c
#include "ota_github.h"

void app_main(void) {
    /* ...bring up WiFi, NVS, network... */

    ota_github_config_t cfg       = OTA_GITHUB_CONFIG_DEFAULT();
    cfg.github_org                = "laurigates";
    cfg.github_repo               = "mcu-tinkering-lab";
    cfg.firmware_filename_match   = "my-project";   /* substring match on asset name */
    cfg.poll_interval_min         = 360;            /* 6 h — the default */
    ESP_ERROR_CHECK(ota_github_init(&cfg));
}
```

That's it. On the next GitHub release tagged e.g. `my-project@v1.2.3` with
an asset whose filename contains `my-project`, the device will download,
verify, flash, and reboot. Rollback protection activates automatically; if
the new firmware panics within 60 seconds of boot, the bootloader reverts
to the previous partition.

---

## When to read what

| Document | Audience |
|---|---|
| [`docs/architecture.md`](docs/architecture.md) | Understanding the state machine, tasks, memory, and failure modes |
| [`docs/adoption-guide.md`](docs/adoption-guide.md) | Bringing up OTA on a new ESP-IDF project, step by step |
| [`docs/release-workflow.md`](docs/release-workflow.md) | What the release/CI side needs to publish for the device to pick it up |
| [`docs/diagrams/`](docs/diagrams/) | Mermaid sources for every diagram referenced above |
| [`include/ota_github.h`](include/ota_github.h) | Full public API with per-field documentation |

---

## What this component does NOT do

Deliberate non-goals that callers must still provide:

- **Network bring-up.** You must have WiFi (or Ethernet, or tunneled LTE)
  connected before the first OTA check. The `pre_download_hook` exists so
  low-power projects can bring WiFi up on demand, but the component itself
  never calls `esp_wifi_*`.
- **NVS credential storage.** Credentials for WiFi or the GitHub URL belong
  in your project, not in this component.
- **MQTT client lifecycle.** Pass in an already-initialized client handle;
  the component only registers event handlers on it.
- **Multi-device orchestration.** If your product has multiple MCUs that
  update together, coordinate them at the application layer. A typical
  pattern is to run PULL mode on the internet-facing MCU and TRIGGERED
  mode (invoked over I2C/UART) on the peer — see the robocar reference
  implementation.
- **Partition table.** You must already ship a dual-OTA partition layout
  (two `app` partitions + an `otadata` partition). Every ESP-IDF OTA
  example has one you can copy.

---

## Configuration reference

Every field is documented inline in [`include/ota_github.h`](include/ota_github.h).
Key fields:

| Field | Default | Notes |
|---|---|---|
| `mode` | `MODE_PULL` | `PULL` = periodic poll; `TRIGGERED` = externally driven |
| `github_org` / `github_repo` | required | `https://github.com/{org}/{repo}` |
| `firmware_filename_match` | required (PULL) | Substring matched against release asset names |
| `poll_interval_min` | 360 | Minutes. Set to 0 to disable polling (MQTT/manual only) |
| `stability_timeout_ms` | 60000 | Window for rollback cancellation |
| `http_timeout_ms` | 30000 | HTTP timeout during download |
| `task_stack_size` | 8192 | Worker task stack (raise if you hook heavy work) |
| `mqtt_enabled` + `mqtt_client` + `mqtt_notify_topic` | off | Optional push-notify |
| `hooks.pre_download_hook` | NULL | Called before any HTTP — e.g. turn on WiFi |
| `hooks.on_update_ready_to_reboot` | NULL | Return non-OK to defer reboot |

---

## License

MIT, matching the rest of the monorepo. See the repository root `LICENSE`.
