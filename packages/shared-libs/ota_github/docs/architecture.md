# `ota_github` architecture

This document explains the internal structure of the component. Read it if
you're debugging an update failure, tuning memory usage, or extending the
component with a new mode.

## Module layout

```
ota_github/
├── include/
│   ├── ota_github.h          ← public API
│   └── ota_github_events.h   ← esp_event bus declarations
└── src/
    ├── ota_github.c          ← init, state mutex, stability timer, API entry points
    ├── ota_github_pull.c     ← PULL mode: wraps esp_ghota's event stream
    ├── ota_github_direct.c   ← TRIGGERED mode: esp_https_ota worker task + SHA256
    ├── ota_github_mqtt.c     ← optional MQTT subscribe → check/trigger
    └── ota_github_internal.h ← private shared state
```

Each `.c` file owns a single concern. They communicate through the global
state in `g_ota_github` (mutex-guarded) and the public esp_event bus.

## State machine

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> IN_PROGRESS : update detected / trigger
    IN_PROGRESS --> SUCCESS : download + verify OK
    IN_PROGRESS --> FAILED : any error
    SUCCESS --> [*] : esp_restart
    FAILED --> IDLE : error surfaced to caller
    IDLE --> MAINTENANCE_MODE : caller sets it (e.g. peer is updating)
    MAINTENANCE_MODE --> IDLE : caller clears it
```

Source: [diagrams/state-machine.mmd](diagrams/state-machine.mmd).

The `MAINTENANCE_MODE` status is not produced by the component itself — it's
reserved for callers (like the robocar main controller) that want to report
a "we're out of service while a peer updates us" state over the same
reporting API.

## PULL mode sequence

```mermaid
sequenceDiagram
    autonumber
    participant App
    participant ota_github
    participant esp_ghota
    participant GitHub

    App->>ota_github: ota_github_init(PULL)
    ota_github->>esp_ghota: ghota_init + start_update_timer
    loop every poll_interval_min
        esp_ghota->>GitHub: GET /releases/latest
        GitHub-->>esp_ghota: JSON (tag, assets)
        alt newer tag
            esp_ghota-->>ota_github: GHOTA_EVENT_UPDATE_AVAILABLE
            ota_github->>App: hooks.on_update_available
            ota_github->>App: hooks.pre_download_hook
            App-->>ota_github: ESP_OK
            ota_github->>esp_ghota: ghota_start_update
            esp_ghota->>GitHub: GET /releases/download/…
            GitHub-->>esp_ghota: firmware stream
            esp_ghota-->>ota_github: PROGRESS events
            ota_github->>App: hooks.on_progress
            esp_ghota-->>ota_github: FINISH_FIRMWARE_UPDATE
            ota_github->>App: hooks.on_update_ready_to_reboot
            App-->>ota_github: ESP_OK
            ota_github->>ota_github: esp_restart
        else no update
            esp_ghota-->>ota_github: NOUPDATE_AVAILABLE
        end
    end
```

Source: [diagrams/sequence-pull.mmd](diagrams/sequence-pull.mmd).

## MQTT-notify sequence

```mermaid
sequenceDiagram
    autonumber
    participant CI as GitHub Actions
    participant Broker as MQTT broker
    participant Device as ota_github (device)
    participant GitHub

    CI->>Broker: publish mqtt_notify_topic = release_tag
    Broker-->>Device: DATA event (topic, payload)
    Device->>Device: rate-limit (max 1/60s)
    alt PULL mode
        Device->>Device: ota_github_check_now
    else TRIGGERED mode
        Device->>Device: ota_github_trigger_tag(payload)
    end
    Device->>GitHub: GET /releases/…
```

Source: [diagrams/sequence-push.mmd](diagrams/sequence-push.mmd).

## TRIGGERED mode sequence (peer orchestration)

```mermaid
sequenceDiagram
    autonumber
    participant Orchestrator as Peer MCU (PULL + orchestration)
    participant Target as Target MCU (TRIGGERED)
    participant GitHub

    Note over Orchestrator: Finished self-update, now updates peer
    Orchestrator->>Target: I2C/UART: BEGIN_OTA(tag, sha256[0..3])
    Target->>Target: ota_github_trigger_tag(tag, sha)
    Target->>Target: hooks.pre_download_hook (lazy WiFi)
    Target->>GitHub: GET /releases/download/{tag}/{asset}.bin
    GitHub-->>Target: firmware stream
    Target->>Target: verify SHA256 prefix (4 bytes)
    Target-->>Orchestrator: I2C status polling (ota_github_get_status)
    Orchestrator->>Target: I2C: REBOOT
    Target->>Target: esp_restart
```

Source: [diagrams/sequence-triggered.mmd](diagrams/sequence-triggered.mmd).

## Partition contract

The component assumes a standard dual-OTA partition table. If your project
doesn't have one, copy [`partitions.csv`][robocar-partitions] from the
robocar reference:

```
# Name,   Type, SubType, Offset,   Size
nvs,      data, nvs,     0x9000,   0x6000
otadata,  data, ota,     0xF000,   0x2000
phy_init, data, phy,     0x11000,  0x1000
ota_0,    app,  ota_0,   0x12000,  0x1CD000
ota_1,    app,  ota_1,   0x1DF000, 0x1CD000
spiffs,   data, spiffs,  0x3AC000, 0x54000
```

```mermaid
block-beta
  columns 7
  nvs["nvs\n24 KB"]:1
  ota["otadata\n8 KB"]:1
  phy["phy\n4 KB"]:1
  app0["ota_0 (app)\n1.8 MB"]:2
  app1["ota_1 (app)\n1.8 MB"]:2
```

Source: [diagrams/partitions.mmd](diagrams/partitions.mmd).

At any moment one partition is *active*, the other is *inactive*. OTA
writes to the inactive partition. On reboot the bootloader switches.
`ota_github_confirm_valid` (called automatically after the stability
timeout) tells the bootloader not to roll back.

**Binary size limit:** 1.8 MB — enforced in CI
(`_build-esp32-firmware.yml` fails the release build if exceeded).

## Tasks and memory

| Task | Stack | Priority | When alive |
|---|---|---|---|
| `ota_github_dl` | `cfg.task_stack_size` (default 8192) | `cfg.task_priority` (default 5) | During a TRIGGERED download; ends with `vTaskDelete` |
| esp_ghota internal | configured by esp_ghota | configured by esp_ghota | Always (PULL mode) |
| `ota_stability` (software timer) | FreeRTOS timer task | timer task priority | Always, for `stability_timeout_ms` after boot |

**RAM overhead** (approximate, ESP32, release build):

- `ota_github` itself: ~1 KB for `g_ota_github` + mutex + timer
- `esp_ghota` (PULL only): ~6 KB task stack + heap for release JSON parsing
- `esp_https_ota` during download: ~10 KB TLS context + 4 KB HTTP buffer
- Certificate bundle (flash, not RAM): ~50 KB

## Failure modes and error codes

`ota_github_get_error_code()` returns a small tag matching the codes in
[include/ota_github.h](../include/ota_github.h):

| Code | Meaning | Typical cause |
|---:|---|---|
| 0 | no error | — |
| 1 | internal | task creation / OOM |
| 2 | `pre_download_hook` failed | WiFi could not be brought up |
| 3 | `esp_https_ota_begin` failed | DNS / TLS / redirect / 404 |
| 4 | download or flash error | TCP reset, short read, flash-write error |
| 5 | `esp_https_ota_finish` failed | corrupt image |
| 6 | SHA256 prefix mismatch | tampered or wrong asset |

On code 6 the component additionally calls
`esp_ota_mark_app_invalid_rollback_and_reboot()` so the bootloader refuses
to run the bad image on the next reset.

## Thread-safety

- All status/progress getters take a 50 ms mutex — safe to call from any
  task including interrupt-level code via `xSemaphoreTakeFromISR` variants
  (not currently exposed; open an issue if you need it).
- Event handlers (ghota, mqtt) may run on separate tasks; they only touch
  state through the public helpers, which acquire the mutex.
- `ota_github_init` is NOT thread-safe — call it once during startup.

## Extending the component

The most common extensions live outside the component (hooks, events). If
you need a new mode (e.g. update from HTTP basic-auth server) the cleanest
approach is a new `ota_github_<mode>.c` that exposes `ota_github_<mode>_run`
and an enum value. Keep the public API stable.

[robocar-partitions]: ../../../esp32-projects/robocar-main/partitions.csv
