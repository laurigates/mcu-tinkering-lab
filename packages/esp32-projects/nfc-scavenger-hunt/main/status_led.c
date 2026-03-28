#include "status_led.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led";

#if defined(BOARD_SUPERMINI)
/*
 * SuperMini uses WS2812 RGB LED on GPIO48.
 * For simplicity in v1, we treat it as a simple on/off GPIO.
 * A proper WS2812 driver (led_strip) would enable color feedback.
 */
#define PIN_LED 48
#else  // XIAO ESP32-S3
#define PIN_LED 2
#endif

#define LED_TASK_STACK 2048

static volatile led_mode_t current_mode = LED_OFF;

static void led_set(bool on)
{
    gpio_set_level(PIN_LED, on ? 1 : 0);
}

static void led_task(void *arg)
{
    bool state = false;

    while (true) {
        switch (current_mode) {
            case LED_OFF:
                led_set(false);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;

            case LED_SLOW_BLINK:
                state = !state;
                led_set(state);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case LED_RAPID_BLINK:
                state = !state;
                led_set(state);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case LED_SOLID_ON:
                led_set(true);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;

            case LED_PULSE:
                /* Simple pulse: fast blink 3x then pause */
                for (int i = 0; i < 6 && current_mode == LED_PULSE; i++) {
                    state = !state;
                    led_set(state);
                    vTaskDelay(pdMS_TO_TICKS(80));
                }
                led_set(false);
                vTaskDelay(pdMS_TO_TICKS(300));
                break;
        }
    }
}

esp_err_t status_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    led_set(false);
    ESP_LOGI(TAG, "Status LED initialized on GPIO%d", PIN_LED);
    return ESP_OK;
}

void status_led_set_mode(led_mode_t mode)
{
    current_mode = mode;
}

esp_err_t status_led_start_task(void)
{
    BaseType_t ret = xTaskCreate(led_task, "led", LED_TASK_STACK, NULL, 2, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
