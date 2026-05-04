/**
 * @file main.c
 * @brief Melody Detector — Phase 1 hardware bring-up.
 *
 * Flow:
 *   app_main
 *     -> status_led_init
 *     -> button_init
 *     -> camera_capture_init
 *     -> audio_test_init
 *     -> capture_loop()
 *
 * The capture loop waits for a debounced button press, captures a JPEG
 * frame, then plays a 1 kHz test tone. Phase 2+ replaces the tone path
 * with the CV → ESP-DL → synth pipeline (see PRD-011 + ADR-017).
 */

#include "audio_test.h"
#include "button.h"
#include "camera_capture.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_led.h"

static const char *TAG = "melody";

static void capture_loop(void)
{
    while (true) {
        status_led_set(LED_STATE_IDLE);
        if (!button_wait_press(0)) {
            continue;
        }
        ESP_LOGI(TAG, "button pressed");

        status_led_set(LED_STATE_CAPTURING);
        camera_fb_t *fb = camera_capture_snapshot();

        status_led_set(LED_STATE_PROCESSING);
        // Phase 2+ runs the CV pipeline + ESP-DL inference here.
        vTaskDelay(pdMS_TO_TICKS(200));

        if (fb) {
            esp_camera_fb_return(fb);
        }

        status_led_set(LED_STATE_PLAYING);
        esp_err_t err = audio_test_tone(1000, 500);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "audio_test_tone: %s", esp_err_to_name(err));
            status_led_set(LED_STATE_ERROR);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "melody-detector boot — phase 1 (camera + I2S + button)");

    if (status_led_init() != ESP_OK) {
        ESP_LOGE(TAG, "status LED init failed; continuing without indicator");
    }
    status_led_set(LED_STATE_PROCESSING);  // solid-on while peripherals come up

    ESP_ERROR_CHECK(button_init());
    ESP_ERROR_CHECK(camera_capture_init());
    ESP_ERROR_CHECK(audio_test_init());

    ESP_LOGI(TAG, "phase 1 ready — press the capture button");
    capture_loop();
}
