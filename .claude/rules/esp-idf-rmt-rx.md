# ESP-IDF v5.4 RMT RX: Three Hardware-Register Constraints (and a Fallback That Hides Them)

The RMT RX peripheral (used here for HC-SR04 ultrasonic echo timing) has three
config constraints enforced against **hardware register widths**. Violating any
one fails a different call, and — critically — this driver falls back to a
GPIO-polling mode on RMT failure, so a broken RMT config is **silent**: the
device still "works" (degraded) and you never see the error unless you read the
boot log. All three below were latent in `ultrasonic.c` for exactly this reason;
each was exposed only after fixing the one before it.

Verify these numbers against the container source, never from memory:
`esp_driver_rmt/src/rmt_rx.c` + `hal/esp32s3/include/hal/rmt_ll.h`.

## 1. `mem_block_symbols` must be even and ≥ 48 (on ESP32-S3)

`rmt_new_rx_channel()` requires `mem_block_symbols` to be **even and at least one
channel memory block** — 48 on the ESP32-S3 (`SOC_RMT_MEM_WORDS_PER_CHANNEL`). A
smaller value fails init with:

```
rmt: rmt_new_rx_channel(...): mem_block_symbols must be even and at least 48
```

`ESP_ERR_INVALID_ARG` → the driver drops to the GPIO-poll fallback on **every**
boot. Set it to 48 (one block) even though you only need ~2 symbols per echo.

## 2. `signal_range_*` map onto different clocks with different register widths

`rmt_receive()` converts both bounds to register values and rejects overflow:

| Field | Counted on | Register | Max value | So the ns bound is |
|---|---|---|---|---|
| `signal_range_min_ns` (glitch filter) | RMT **source** clock (~80 MHz) | 8-bit | 255 | ≤ ~3187 ns |
| `signal_range_max_ns` (idle threshold) | **channel** resolution (e.g. 1 MHz) | 15-bit | 32767 | ≤ 32.767 ms |

`10 µs` min mapped to 800 (> 255) → `signal_range_min_ns too big`; `38 ms` max
mapped to 38000 (> 32767) → `signal_range_max_ns too big`. Note the min bound is
tighter than intuition suggests because it counts on the fast **source** clock,
not the divided channel clock. Working values here: `min = 1 µs` (still rejects
electrical glitches, passes any echo ≥ 58 µs = 1 cm), `max = 30 ms` (covers the
4 m / ~23 ms max echo under the 32.767 ms cap).

## 3. A no-echo receive wedges the channel — re-arm it on timeout

`rmt_receive()` requires the channel FSM in `RMT_FSM_ENABLE` and moves it to
`RUN`. When no echo ever completes the receive (no sensor fitted, or target out
of range) the RX-done callback never fires, so the FSM stays stuck in `RUN` and
**every subsequent `rmt_receive()` fails** with `ESP_ERR_INVALID_STATE` ("channel
not in enable state"). One missed reading wedges the sensor **permanently**, even
with real hardware. Aborting the pending receive is not automatic — do it in the
timeout branch:

```c
if (ulTaskNotifyTake(pdTRUE, timeout_ticks) == 0) {  // no echo
    rmt_disable(s_rx_chan);   // RUN -> INIT (rmt_rx_disable handles the RUN state)
    rmt_enable(s_rx_chan);    // INIT -> ENABLE, re-armed for the next measurement
    ...
}
```

## Meta-lesson: a fallback path masks the primary path's bugs

The GPIO-poll fallback made all three RMT bugs invisible for a long time, and
fixing them surfaced in a chain — init → receive-signal-range → channel-rearm —
each masked by the failure before it. When a driver has a "degraded but working"
fallback, assume the primary path is **untested** and read the boot log to
confirm which mode is actually active (`Ultrasonic driver ready (RMT RX mode …)`
vs `(GPIO poll mode)`). Expect to peel several bugs once you get the primary path
past its first error.
