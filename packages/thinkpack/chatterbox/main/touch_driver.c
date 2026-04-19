/**
 * @file touch_driver.c
 * @brief Minimal touch_pad polling driver — one pad, edge callbacks.
 *
 * Calibrates on boot by sampling the untouched baseline for 200 ms, then
 * polls at 50 Hz.  A reading below (baseline × TOUCH_THRESHOLD_RATIO) is
 * treated as a press.
 *
 * Hardware-unverified.  Uses the legacy touch_pad API which is still the
 * supported path on S3 as of ESP-IDF 5.4; migrate to touch_element if/when
 * the legacy API is deprecated.
 */

#include "touch_driver.h"

#include "driver/touch_pad.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "touch";

/* Single pad — GPIO14 on ESP32-S3 is touch channel 14. Update pin + channel
 * together if you move the pad.  See WIRING.md. */
#define TOUCH_CHANNEL TOUCH_PAD_NUM14

#define TOUCH_BASELINE_SAMPLES 10
#define TOUCH_BASELINE_DELAY_MS 20
#define TOUCH_POLL_PERIOD_MS 20
/* Press threshold — a touched pad reads ~40 % of the untouched baseline. */
#define TOUCH_THRESHOLD_NUM 60
#define TOUCH_THRESHOLD_DEN 100

static touch_driver_cb_t s_cb;
static uint32_t s_baseline;
static bool s_pressed;

static uint32_t sample_pad(void)
{
    uint32_t v = 0;
    touch_pad_read_raw_data(TOUCH_CHANNEL, &v);
    return v;
}

static void calibrate_baseline(void)
{
    uint64_t sum = 0;
    for (int i = 0; i < TOUCH_BASELINE_SAMPLES; i++) {
        sum += sample_pad();
        vTaskDelay(pdMS_TO_TICKS(TOUCH_BASELINE_DELAY_MS));
    }
    s_baseline = (uint32_t)(sum / TOUCH_BASELINE_SAMPLES);
    ESP_LOGI(TAG, "touch baseline = %u", (unsigned)s_baseline);
}

static void touch_task(void *arg)
{
    (void)arg;
    calibrate_baseline();
    uint32_t threshold =
        (uint32_t)((uint64_t)s_baseline * TOUCH_THRESHOLD_NUM / TOUCH_THRESHOLD_DEN);

    for (;;) {
        uint32_t v = sample_pad();
        bool now_pressed = (v != 0 && v < threshold);
        if (now_pressed != s_pressed) {
            s_pressed = now_pressed;
            if (s_cb != NULL) {
                s_cb(now_pressed);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_PERIOD_MS));
    }
}

esp_err_t touch_driver_init(touch_driver_cb_t cb)
{
    s_cb = cb;

    esp_err_t err = touch_pad_init();
    if (err != ESP_OK) {
        return err;
    }
    err = touch_pad_config(TOUCH_CHANNEL);
    if (err != ESP_OK) {
        return err;
    }
    err = touch_pad_fsm_start();
    if (err != ESP_OK) {
        return err;
    }

    BaseType_t rc = xTaskCreate(touch_task, "touch", 3072, NULL, 4, NULL);
    if (rc != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool touch_driver_is_pressed(void)
{
    return s_pressed;
}
