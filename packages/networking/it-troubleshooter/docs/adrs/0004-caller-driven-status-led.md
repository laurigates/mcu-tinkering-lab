# ADR-0004: Caller-Driven Status LED (No Background Task)

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 8/10

## Context

The WS2812 RGB LED (GPIO21) needs periodic refresh for blink/pulse animations. This can be implemented as:
1. A dedicated FreeRTOS task polling a shared state variable
2. A caller-driven update function invoked from existing application loops

## Decision

Implement the LED as **caller-driven**: `status_led_update()` must be called periodically (≥50 Hz) from the application's existing loop. No background task is created.

## Rationale

This pattern was refined from the xbox-switch-bridge project, which initially had a dedicated LED task but dropped it after discovering that `bp32_host_start()` (BTstack) starved other core 0 tasks.

For the IT Troubleshooter, the USB stack runs on core 0 via TinyUSB's internal task. Adding a separate LED task on core 0 risks scheduling contention. The application loop on core 1 is a natural call site.

Benefits:
- Zero additional task overhead
- No inter-task synchronization for LED state (volatile atomic store + single writer)
- Application controls LED update rate implicitly through its own loop frequency
- Easier to integrate into future WiFi/Claude API loops (just add `status_led_update()` call)

## Evidence

- `components/status_led/status_led.c`: No task creation; `status_led_update()` reads volatile mode and calls `esp_timer_get_time()` for blink timing
- `main/main.c`: CDC echo task on core 1 calls `status_led_update()` in its loop
- `sdkconfig.defaults`: `CONFIG_FREERTOS_HZ=1000` (1ms tick resolution supports accurate blink timing)

## Consequences

- Callers must invoke `status_led_update()` frequently enough (≥50 Hz) for smooth blink rendering
- If the application task blocks (e.g., waiting for WiFi), LED may appear frozen — caller responsible for non-blocking design
- LED state changes (`status_led_set_mode()`) are non-blocking volatile stores; safe to call from any context
