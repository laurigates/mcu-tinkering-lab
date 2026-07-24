# FreeRTOS Task Gotchas on ESP-IDF: Yield-on-Overrun and TLS Stack Sizing

Two independent task-authoring traps found bringing up the robocar-unified
firmware. Both crash (task watchdog / stack overflow), both are intermittent
enough to pass a quick test and fail later.

## 1. A periodic task above IDLE must guarantee a yield when the loop overruns

`vTaskDelayUntil()` / `xTaskDelayUntil()` returns **without blocking** when the
loop body already overran the period (the wake deadline is in the past). A task
pinned above the idle task that overruns *every* iteration therefore never
yields — it starves IDLE on that core and trips the **Core-N task watchdog**:

```
task_wdt: ... IDLE0 (CPU 0) ... CPU 0: <your task>
```

This bites whenever the loop body can take longer than its period: here a 30 Hz
(33 ms) reactive loop calling an ultrasonic measurement whose GPIO-poll fallback
busy-waited its 40 ms echo timeout (40 > 33 → guaranteed overrun with no sensor).

**Fix** — guard the overrun case so IDLE always gets a slot; the healthy path is
unchanged:

```c
if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS)) == pdFALSE) {
    vTaskDelay(1);                    // let IDLE run
    last_wake = xTaskGetTickCount();  // resync; don't fire a catch-up burst
}
```

**Also a lesson about masking:** this watchdog was *exposed* by making
`init_camera()` non-fatal — the board used to panic at camera init and reboot
*before* the reactive loop ever ran. Making a system boot further routinely
surfaces latent bugs downstream of the point that used to abort. Expect them.

## 2. A task that does an HTTPS/TLS call needs ≥ 8 KB stack

`esp_http_client` + mbedTLS + cJSON burns several KB of stack in the **TLS
handshake**. A 4 KB task stack overflows there:

```
***ERROR*** A stack overflow in task <name> has been detected.
```

It's insidiously **intermittent**: a DNS failure short-circuits the request
*before* the deep handshake path, so the task survives while DNS is broken and
then crashes every time once DNS resolves. Size any task that makes an HTTPS call
at **8 KB** (`xTaskCreatePinnedToCore(..., 8192, ...)`), matching the sibling
task that makes the same call rather than guessing low.

### Related: TLS cert-date validation needs a clock — unless the build disables it

ESP-IDF's cert-bundle TLS validates certificate `notBefore`/`notAfter` only when
`CONFIG_MBEDTLS_HAVE_TIME_DATE=y`. This firmware builds with it **unset**, so TLS
skips date validation and works with an **unsynced clock** — no SNTP/NTP client
is required for HTTPS. Before adding SNTP (or opening an NTP egress rule) to fix a
TLS failure, check `sdkconfig` for `MBEDTLS_HAVE_TIME_DATE`: if it's off, the
clock is not your problem.
