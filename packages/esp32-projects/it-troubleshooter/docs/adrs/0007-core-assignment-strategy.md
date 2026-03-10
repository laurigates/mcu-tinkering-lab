# ADR-0007: Core Assignment Strategy

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 8/10

## Context

ESP32-S3 has two cores (core 0 = PRO_CPU, core 1 = APP_CPU). TinyUSB creates an internal task. Application tasks can run on either core. The choice affects USB timing reliability and application responsiveness.

## Decision

- **Core 0**: TinyUSB internal task (USB stack, interrupt handling)
- **Core 1**: Application tasks (CDC echo, future WiFi, Claude API, command execution)

TinyUSB task config: `{ .size = 4096, .priority = 5, .xCoreID = 0 }`

## Rationale

USB interrupt handling is time-sensitive — jitter can cause enumeration failures or missed HID reports. Pinning TinyUSB to core 0 gives it a dedicated core with predictable scheduling.

Application work (WiFi, API calls, HID injection, CDC processing) runs on core 1 without competing with the USB stack. This mirrors the proven pattern from xbox-switch-bridge where BTstack was pinned to core 0 and the bridge task ran on core 1.

## Evidence

- `components/usb_composite/usb_composite.c`: `tinyusb_config_t.task = { .xCoreID = 0 }`
- `main/main.c`: `xTaskCreatePinnedToCore(..., 1)` for CDC echo task (core 1)
- TinyUSB requires non-zero `.task.size` and `.task.priority` — zero values return `ESP_ERR_INVALID_ARG`

## Consequences

- USB timing is isolated from application blocking operations (WiFi, HTTP, UART)
- Future tasks should default to core 1 unless they have USB-level timing requirements
- If WiFi is added, `esp_wifi_set_ps(WIFI_PS_NONE)` should be set for minimum latency
- Core 0 must not be overloaded with high-frequency application tasks
