# ADR-0008: Caller-Driven Status LED (No Background Task)

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: Very High

## Context

The WS2812 status LED needs to display blink patterns (SCANNING, CONNECTED_NO_USB, CONNECTED_USB) that require periodic updates at >50 Hz. The typical approach would be a dedicated FreeRTOS task.

However, BTstack's `bp32_host_start()` on core 0 starves other core 0 tasks. This was discovered during development of the it-troubleshooter project (same hardware) and confirmed in the xbox-switch-bridge project.

## Decision

Use a **caller-driven LED update** pattern. The `status_led` component exposes:
- `status_led_set_mode()` — sets the desired mode (thread-safe, just writes a volatile enum)
- `status_led_update()` — performs the actual RMT write (must be called from an existing loop)

The bridge task on core 1 calls `status_led_update()` at the end of each 125 Hz iteration:

```c
while (1) {
    // ... bridge logic ...
    status_led_update();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(8));
}
```

This provides 125 Hz LED refresh, which is more than sufficient for visible blink patterns.

## Consequences

- **No additional task or stack allocation** — the LED update runs in the bridge task's existing stack.
- **LED update is coupled to bridge loop** — if the bridge task stalls, LED updates stop. This is acceptable because a stalled bridge loop is itself an error condition.
- **Same pattern as it-troubleshooter** — consistent component API across projects. The `status_led` component is architecturally identical.
- **RMT DMA must be disabled** — `with_dma = true` conflicts with TinyUSB DMA on ESP32-S3. The component uses `with_dma = false` + `mem_block_symbols = 48` (ESP32-S3 has 48 symbols per RMT block, not 64).

## Alternatives Considered

- **Dedicated LED task on core 1**: Adds stack usage and task management for a trivial operation. The bridge loop already runs at 125 Hz. Rejected.
- **Dedicated LED task on core 0**: BTstack starves core 0 tasks, causing LED update starvation. Rejected.
- **FreeRTOS software timer**: Timer callbacks run in the timer task context, which is on core 0 by default. Same starvation problem. Rejected.
- **Hardware timer interrupt**: Overkill for LED blinking. RMT writes from ISR context are not supported. Rejected.
