# Adopting `ota_github` in a new ESP-IDF project

Estimated effort: 20 minutes to first successful OTA, assuming the project
already has WiFi working.

---

## 1. Prerequisites

- ESP-IDF ≥ 5.1 (≥ 6.0 if using the newest esp_ghota releases)
- Project boots and connects to WiFi before `app_main` hands control to your
  business logic
- `nvs_flash` initialized
- Default esp_event loop created (`esp_event_loop_create_default()`)
- A dual-OTA partition table (see
  [architecture.md § Partition contract](architecture.md#partition-contract))

## 2. Wire up the component

### 2.1. Make it visible to the build

In your project root `CMakeLists.txt` (the one next to `sdkconfig.defaults`):

```cmake
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
```

Adjust the relative path to however deeply nested your project is.

### 2.2. Declare the dependency

In `main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main.c"
    REQUIRES "ota_github"
)
```

`ota_github`'s `idf_component.yml` declares `fishwaldo/esp_ghota` as a
transitive dependency — you do NOT need to list it yourself.

### 2.3. Enable rollback in sdkconfig

Add to `sdkconfig.defaults`:

```ini
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_ESP_HTTPS_OTA_DECRYPT_CB=n           # unless you're using encrypted OTA
CONFIG_ESP_TLS_INSECURE=n
```

Delete the generated `sdkconfig` (not `sdkconfig.defaults`) so the new
defaults take effect, then run a clean build.

## 3. Call `ota_github_init`

The minimum useful snippet:

```c
#include "ota_github.h"

static void start_ota(void) {
    ota_github_config_t cfg     = OTA_GITHUB_CONFIG_DEFAULT();
    cfg.github_org              = "laurigates";
    cfg.github_repo             = "mcu-tinkering-lab";
    cfg.firmware_filename_match = "my-project";
    ESP_ERROR_CHECK(ota_github_init(&cfg));
}

void app_main(void) {
    /* WiFi, NVS, etc. */
    wait_for_wifi_connected();
    start_ota();
}
```

## 4. Release-side requirements

Two things must be true on GitHub for the device to find firmware:

1. A release exists with a semver tag. This monorepo's release-please tags
   are `<component>-v1.2.3`; esp_ghota still picks the correct version
   because the comparison is done on the appended semver, not the prefix.
2. A release **asset** whose filename `fnmatch(cfg.firmware_filename_match,
   name, 0)` matches — a **full-string** match, not a substring match. Set
   `firmware_filename_match` to your project's `flasher.json` `otaAssetName`
   exactly, or to a pattern with an explicit wildcard (e.g. `"my-project*"`)
   if you want to match more than one exact name.

See [release-workflow.md](release-workflow.md) for the full CI pipeline,
which is [`build-firmware.yml`](../../../../.github/workflows/build-firmware.yml)
— it discovers every project from its `flasher.json`, not from a per-project
workflow caller.

## 5. Optional: add MQTT push-notify

Lets you trigger an immediate update without waiting for the poll
interval. Requires a working `esp_mqtt_client_handle_t` from your
application.

```c
cfg.mqtt_enabled      = true;
cfg.mqtt_client       = my_mqtt_client;             /* already connected */
cfg.mqtt_notify_topic = "my-project/ota/notify";    /* must equal your
                                                      * flasher.json
                                                      * otaNotifyTopic */
```

Add the matching `otaNotifyTopic` to your project's `flasher.json` —
`build-firmware.yml` publishes the release tag there automatically once it
resolves your release to your project's component. See
[release-workflow.md § MQTT notify topic](release-workflow.md#mqtt-notify-topic)
for the scheme and the payload contract.

## 6. Optional: project-specific hooks

```c
static esp_err_t bring_up_wifi(void *ctx) {
    return wifi_manager_connect(NULL, NULL);
}

static esp_err_t quiesce_peripherals(void *ctx) {
    motor_stop_all();
    buzzer_off();
    return ESP_OK;  /* proceed with reboot */
}

cfg.hooks.pre_download_hook         = bring_up_wifi;
cfg.hooks.on_update_ready_to_reboot = quiesce_peripherals;
```

Return anything other than `ESP_OK` from `pre_download_hook` to abort the
update. Return anything other than `ESP_OK` from
`on_update_ready_to_reboot` to leave the device with the new firmware
staged but not yet booted — useful when a peer needs to orchestrate the
reboot.

## 7. Optional: observe events

```c
static void on_ota_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    ota_github_event_payload_t *p = data;
    switch (id) {
        case OTA_GITHUB_EVENT_UPDATE_AVAILABLE:
            ESP_LOGI("app", "new version: %s", p->version);
            break;
        case OTA_GITHUB_EVENT_PROGRESS:
            ui_set_progress_bar(p->progress);
            break;
        /* … */
    }
}

esp_event_handler_register(OTA_GITHUB_EVENTS, ESP_EVENT_ANY_ID,
                           &on_ota_event, NULL);
```

## 8. TRIGGERED mode (peer orchestration)

Use this when your device doesn't poll GitHub itself — a peer tells it
what to download. The robocar main controller is the reference example.

```c
ota_github_config_t cfg        = OTA_GITHUB_CONFIG_DEFAULT();
cfg.mode                       = OTA_GITHUB_MODE_TRIGGERED;
cfg.github_org                 = "laurigates";
cfg.github_repo                = "mcu-tinkering-lab";
cfg.triggered_asset_filename   = "robocar-main.bin";
cfg.hooks.pre_download_hook    = bring_up_wifi;   /* WiFi-on-demand */
ESP_ERROR_CHECK(ota_github_init(&cfg));

/* Later, when the peer sends BEGIN_OTA over I2C: */
ota_github_trigger_tag(NULL, received_tag, received_sha_prefix);

/* The peer polls: */
ota_github_get_status();
ota_github_get_progress();
```

## 9. First-boot considerations

- The rollback timer starts inside `ota_github_init`. If your
  `app_main` takes >10 s to reach that point, tune
  `stability_timeout_ms` accordingly.
- On a freshly-USB-flashed device there is no inactive OTA partition to
  roll back to; `esp_ota_mark_app_valid_cancel_rollback` returns a benign
  warning which the component logs and ignores.
- Web Flasher users get the full standard partition layout, so the second
  OTA will use `ota_1` and have full rollback safety.

## 10. Verification checklist

- [ ] `idf.py build` succeeds — it fails by itself if the binary exceeds the
      smallest `ota_0`/`ota_1` partition of your table; read the
      `binary size … free` line it prints for headroom
- [ ] Device boots; logs show `ota_github initialized`
- [ ] `Rollback stability timer started (60000 ms)` appears once
- [ ] `Firmware marked as valid — rollback cancelled` appears 60 s later
- [ ] Poll fires on schedule (log `Triggering manual update check` or
      the equivalent ghota "No update available")
- [ ] On a staged release: update downloads, flashes, reboots into new
      version, and the rollback message appears again

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `ghota_init failed` | GitHub unreachable during init, or missing `firmware_filename_match` |
| `esp_https_ota_begin: ESP_FAIL` | DNS / captive portal / redirect loop |
| `SHA256 prefix mismatch` | Device received wrong asset — check `firmware_filename_match` or the CI artifact name |
| Device loops between two versions | Your new firmware panics within the stability window — check the bootloader console for rollback reason |
| `No update available` forever | The tag on GitHub parses to a semver ≤ the running `esp_app_get_description()->version`; bump the version in CMakeLists |
