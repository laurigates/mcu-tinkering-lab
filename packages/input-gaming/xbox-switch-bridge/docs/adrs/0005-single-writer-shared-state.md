# ADR-0005: Single-Writer Shared State (No Mutex)

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: High

## Context

The bridge architecture has two cores accessing the same gamepad state:
- Core 0 (BTstack/Bluepad32): writes `s_gamepad_state` on every BLE controller data callback
- Core 1 (bridge task): reads `s_gamepad_state` every 8ms to build USB HID reports

This is a classic shared-state concurrency problem. The question is whether to use a mutex, atomic operations, or rely on the single-writer pattern.

## Decision

Use a **single-writer pattern without a mutex**.

```c
/* Shared state protected by a single-writer assumption (BP32 task) */
static xbox_gamepad_state_t s_gamepad_state;
```

Core 0 is the sole writer; core 1 only reads via `memcpy()`:

```c
bool bp32_host_get_state(xbox_gamepad_state_t *state) {
    if (!s_gamepad_state.connected) return false;
    memcpy(state, &s_gamepad_state, sizeof(xbox_gamepad_state_t));
    return true;
}
```

Key reasons:
- **No contention on the critical path** — the bridge loop runs at 125 Hz. Adding a mutex introduces priority inversion risk and potential jitter on the USB report timing.
- **Worst case is one stale frame** — if the reader copies mid-write, it gets a mix of old and new values for a single 8ms frame. For gamepad input, this is imperceptible.
- **Struct is small** — `xbox_gamepad_state_t` is ~20 bytes. `memcpy()` on ESP32-S3 completes in well under 1us.
- **BTstack callback is the only writer** — Bluepad32's `on_controller_data` callback runs in the BTstack task. No other code writes to `s_gamepad_state`.

## Consequences

- **Not suitable for safety-critical data** — this pattern works because a torn read produces at most one frame of slightly wrong input. Do not use this pattern for state where partial reads cause errors.
- **Must remain single-writer** — if multi-controller support (FR12) is added, the pattern must be revisited to ensure only one writer per state struct.
- **`volatile` on bridge state enum** — `s_state` uses `volatile` since it's written by core 0 (BLE callbacks) and read by core 1 (bridge loop).

## Alternatives Considered

- **FreeRTOS mutex**: Safe but adds jitter to the 125 Hz loop. The mutex take/give overhead is small, but priority inversion risk is real when BTstack is involved. Rejected.
- **Double-buffering**: Write to buffer A, atomically swap pointer to buffer B. More complex, unnecessary for 20-byte structs. Rejected.
- **FreeRTOS queue**: Good for event-driven designs but adds copy overhead and queue management. Rejected for polling-based architecture.
