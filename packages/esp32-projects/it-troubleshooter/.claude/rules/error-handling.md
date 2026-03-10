# Error Handling

**Confidence**: High — derived from recurring fix patterns in git history
**Source**: commits c871c18, 392897c, fix(auto) series, it-troubleshooter main.c patterns

## Graceful Degradation Over Crash

Optional features (USB enumeration, WiFi connection) should fail gracefully — log a warning, set an error LED state, and continue or return. Do not `ESP_ERROR_CHECK` on optional init paths.

```c
// Good — graceful degradation
esp_err_t ret = usb_composite_init();
if (ret != ESP_OK) {
    ESP_LOGW(TAG, "USB init failed (%s)", esp_err_to_name(ret));
    status_led_set_mode(STATUS_LED_ERROR);
    return;   // return from app_main, not reboot
}

// Avoid — fatal crash on optional feature
ESP_ERROR_CHECK(usb_composite_init());
```

## Timeout-Based Polling (Not Infinite Blocking)

Use bounded polling loops for all wait conditions. Always yield inside the loop.

```c
// Good — timeout + LED feedback
int wait_ms = 0;
while (!usb_composite_is_mounted() && wait_ms < USB_MOUNT_TIMEOUT_MS) {
    status_led_update();
    vTaskDelay(pdMS_TO_TICKS(10));
    wait_ms += 10;
}
if (!usb_composite_is_mounted()) {
    ESP_LOGW(TAG, "USB mount timeout after %d ms", USB_MOUNT_TIMEOUT_MS);
    status_led_set_mode(STATUS_LED_ERROR);
    return;
}
```

## Mutex + Bounded Timeout

Use short timeouts on mutex takes (never `portMAX_DELAY` for shared state). Log if the mutex cannot be taken.

```c
// Good — bounded timeout
if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    s_shared_state = new_value;
    xSemaphoreGive(s_mutex);
} else {
    ESP_LOGW(TAG, "Failed to take mutex — skipping update");
}
```

## Always Log Error Return Values

Every `esp_err_t`-returning function call should have its result logged if it may fail, even if recovery is not possible.

```c
esp_err_t ret = some_operation();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Operation failed: %s", esp_err_to_name(ret));
}
```

## HID/CDC Per-Operation Readiness Check

Check peripheral readiness before each send; return `ESP_ERR_TIMEOUT` if not ready — let the caller decide recovery.

```c
int retries = HID_READY_TIMEOUT_MS / HID_READY_POLL_MS;
while (!tud_hid_ready() && retries-- > 0) {
    vTaskDelay(pdMS_TO_TICKS(HID_READY_POLL_MS));
}
if (!tud_hid_ready()) {
    ESP_LOGW(TAG, "HID not ready after timeout");
    return ESP_ERR_TIMEOUT;
}
```

## FreeRTOS Timer Handles

Always store timer handles in static variables. Clean up on re-initialization to prevent orphaned timers.

```c
static TimerHandle_t s_stability_timer = NULL;

if (s_stability_timer != NULL) {
    xTimerDelete(s_stability_timer, pdMS_TO_TICKS(100));
    s_stability_timer = NULL;
}
s_stability_timer = xTimerCreate(...);
```
