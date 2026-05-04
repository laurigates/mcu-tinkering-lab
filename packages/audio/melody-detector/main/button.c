#include "button.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "button";

static inline bool button_pressed(void)
{
    return gpio_get_level(BUTTON_PIN) == BUTTON_ACTIVE_LEVEL;
}

esp_err_t button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (BUTTON_ACTIVE_LEVEL == 0) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (BUTTON_ACTIVE_LEVEL == 1) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "capture button on GPIO%d (active %s, %dms debounce)", BUTTON_PIN,
             BUTTON_ACTIVE_LEVEL == 0 ? "LOW" : "HIGH", BUTTON_DEBOUNCE_MS);
    return ESP_OK;
}

bool button_wait_press(uint32_t timeout_ms)
{
    const int64_t deadline =
        (timeout_ms == 0) ? INT64_MAX : esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    bool was_released = !button_pressed();

    while (esp_timer_get_time() < deadline) {
        if (was_released && button_pressed()) {
            // candidate edge — confirm after debounce window
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (button_pressed()) {
                return true;
            }
        }
        was_released = !button_pressed();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}
