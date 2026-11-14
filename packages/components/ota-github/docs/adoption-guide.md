# Adopting `ota_github` in a new ESP-IDF project

Estimated effort: 20 minutes to first successful OTA, assuming the project
already has WiFi working.

`ota_github` downloads a release asset when the application asks for one
(TRIGGERED mode); it does not poll GitHub. A project that should discover
new releases by itself copies robocar's manifest poll,
[`ota_manager.c`](../../../robotics/robocar/main/ota_manager.c), instead.

---

## 1. Prerequisites

- ESP-IDF ≥ 5.1
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

`ota_github` has no managed-component dependencies; it uses ESP-IDF's own
`esp_https_ota` and `mqtt`.

### 2.3. Enable rollback in sdkconfig

Add to `sdkconfig.defaults`:

```ini
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_ESP_HTTPS_OTA_DECRYPT_CB=n           # unless you're using encrypted OTA
CONFIG_ESP_TLS_INSECURE=n
```

Delete the generated `sdkconfig` (not `sdkconfig.defaults`) so the new
defaults take effect, then run a clean build.

## 3. Call `ota_github_init`, then trigger

The minimum useful snippet:

```c
#include "ota_github.h"

static void start_ota(void) {
    ota_github_config_t cfg      = OTA_GITHUB_CONFIG_DEFAULT();  /* TRIGGERED */
    cfg.github_org               = "laurigates";
    cfg.github_repo              = "mcu-tinkering-lab";
    cfg.triggered_asset_filename = "my-project.bin";  /* = flasher.json otaAssetName */
    cfg.hooks.pre_download_hook  = bring_up_wifi;      /* optional, see § 6 */
    ESP_ERROR_CHECK(ota_github_init(&cfg));
}

void app_main(void) {
    /* WiFi, NVS, etc. */
    wait_for_wifi_connected();
    start_ota();
}

/* Later, when a peer, console command or MQTT message names a release: */
ota_github_trigger_tag(NULL, received_tag, received_sha_prefix);

/* The caller (or a peer over I2C) polls: */
ota_github_get_status();
ota_github_get_progress();
```

`ota_github_trigger_url` takes a full HTTPS URL instead, for an asset that
does not follow the release-download URL shape.

## 4. Release-side requirements

Two things must be true on GitHub for the download to succeed:

1. A release exists under the tag you pass. This monorepo's release-please
   tags are `<component>-v1.2.3`; the component puts the tag in the URL as
   given and never parses it.
2. The release has an **asset** named exactly `cfg.triggered_asset_filename`
   (or the `asset_override` argument). Set it to your project's
   `flasher.json` `otaAssetName`; there is no pattern matching.

See [release-workflow.md](release-workflow.md) for the full CI pipeline,
which is [`build-firmware.yml`](../../../../.github/workflows/build-firmware.yml)
— it discovers every project from its `flasher.json`, not from a per-project
workflow caller.

## 5. Optional: add MQTT push-notify

Lets the release pipeline trigger the download: the message payload is the
release tag, passed to `ota_github_trigger_tag`. Requires a working
`esp_mqtt_client_handle_t` from your application.

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
        case OTA_GITHUB_EVENT_PROGRESS:
            ui_set_progress_bar(p->progress);
            break;
        case OTA_GITHUB_EVENT_FAILED:
            ESP_LOGW("app", "OTA failed, error %u", p->error_code);
            break;
        /* … */
    }
}

esp_event_handler_register(OTA_GITHUB_EVENTS, ESP_EVENT_ANY_ID,
                           &on_ota_event, NULL);
```

## 8. First-boot considerations

- The rollback timer starts inside `ota_github_init`. If your
  `app_main` takes >10 s to reach that point, tune
  `stability_timeout_ms` accordingly.
- On a freshly-USB-flashed device there is no inactive OTA partition to
  roll back to; `esp_ota_mark_app_valid_cancel_rollback` returns a benign
  warning which the component logs and ignores.
- Web Flasher users get the full standard partition layout, so the second
  OTA will use `ota_1` and have full rollback safety.

## 9. Verification checklist

- [ ] `idf.py build` succeeds — it fails by itself if the binary exceeds the
      smallest `ota_0`/`ota_1` partition of your table; read the
      `binary size … free` line it prints for headroom
- [ ] Device boots; logs show `ota_github initialized`
- [ ] `Rollback stability timer started (60000 ms)` appears once
- [ ] `Firmware marked as valid — rollback cancelled` appears 60 s later
- [ ] On a staged release: a trigger downloads, flashes, reboots into the new
      version, and the rollback message appears again

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `ota_github_init` returns `ESP_ERR_INVALID_ARG` | `github_org` or `github_repo` unset, or `mode` is not `OTA_GITHUB_MODE_TRIGGERED` (a zeroed config instead of `OTA_GITHUB_CONFIG_DEFAULT()`) |
| `esp_https_ota_begin: ESP_FAIL` | DNS / captive portal / redirect loop, or no asset of that name under that tag (404) |
| `SHA256 prefix mismatch` | Device received wrong asset — check `triggered_asset_filename` or the CI artifact name |
| Device loops between two versions | Your new firmware panics within the stability window — check the bootloader console for rollback reason |
