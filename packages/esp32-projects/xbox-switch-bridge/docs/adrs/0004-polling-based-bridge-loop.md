# ADR-0004: Polling-Based 125 Hz Bridge Loop

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: High

## Context

The bridge needs to read Xbox controller state, map it to Switch format, and send USB HID reports. The timing must match the Switch's expected input rate.

## Decision

Use a **polling-based bridge loop running at 125 Hz** (8ms interval) on core 1 via `vTaskDelayUntil()`.

```c
#define BRIDGE_LOOP_INTERVAL_MS 8
TickType_t last_wake = xTaskGetTickCount();
while (1) {
    // read state, map buttons, send report, update LED
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BRIDGE_LOOP_INTERVAL_MS));
}
```

Key reasons:
- **Matches USB HID poll rate** — the Switch polls for input reports every 8ms (125 Hz). Sending faster wastes CPU; sending slower introduces input lag.
- **Simpler than event-driven** — an event-driven approach would require synchronizing BLE callbacks with USB report timing. Polling decouples the two.
- **Deterministic timing** — `vTaskDelayUntil()` provides consistent intervals regardless of processing time, avoiding drift.
- **LED updates integrated** — `status_led_update()` is called in the same loop, providing >50 Hz refresh for blink patterns.

## Consequences

- **Core 1 dedicated** — the bridge task is pinned to core 1 via `xTaskCreatePinnedToCore()`. Core 0 is reserved for BTstack/BLE.
- **Slight input latency** — worst case one full loop cycle (8ms) between BLE state update and USB report. Acceptable for gaming.
- **Stack monitoring** — the bridge task logs stack high-water mark at exponential intervals (100, 1000, 10000 reports) to catch sizing issues early.

## Alternatives Considered

- **Event-driven (BLE callback triggers USB send)**: More complex synchronization, no guarantee of 125 Hz timing. Rejected.
- **Higher frequency (250/500 Hz)**: The Switch ignores reports faster than its poll rate. Wasted CPU. Rejected.
- **FreeRTOS software timer**: Less control over core affinity. Rejected.
