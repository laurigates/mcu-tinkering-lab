#include "status_led.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "led";

static volatile led_state_t s_state = LED_STATE_IDLE;

static inline void led_write(int on)
{
    int level = on ? STATUS_LED_ACTIVE_LEVEL : !STATUS_LED_ACTIVE_LEVEL;
    gpio_set_level(STATUS_LED_PIN, level);
}

// Each pattern is a sequence of (on?, ms) pairs. The task replays the
// pattern matching the current state until the state changes.
static void led_task(void *arg)
{
    led_state_t last = (led_state_t)-1;
    int step = 0;
    TickType_t until = 0;
    int on = 0;
    int ms = 100;

    while (true) {
        led_state_t now = s_state;
        if (now != last) {
            last = now;
            step = 0;
            until = 0;
        }

        if (xTaskGetTickCount() >= until) {
            switch (now) {
                case LED_STATE_IDLE:
                    on = (step % 2 == 0);
                    ms = 500;
                    break;
                case LED_STATE_CAPTURING:
                    on = (step % 2 == 0);
                    ms = 100;
                    break;
                case LED_STATE_PROCESSING:
                    on = 1;
                    ms = 200;
                    break;
                case LED_STATE_PLAYING: {
                    // double-pulse: on, off, on, long-off
                    static const int pat[4] = {1, 0, 1, 0};
                    static const int dur[4] = {80, 80, 80, 600};
                    on = pat[step % 4];
                    ms = dur[step % 4];
                    break;
                }
                case LED_STATE_ERROR:
                    on = (step % 2 == 0);
                    ms = 60;
                    break;
            }
            led_write(on);
            until = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
            step++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t status_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << STATUS_LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config: %s", esp_err_to_name(err));
        return err;
    }
    led_write(0);

    BaseType_t ok = xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 2, NULL, 0);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "status LED on GPIO%d (active %s)", STATUS_LED_PIN,
             STATUS_LED_ACTIVE_LEVEL == 0 ? "LOW" : "HIGH");
    return ESP_OK;
}

void status_led_set(led_state_t state)
{
    s_state = state;
}
