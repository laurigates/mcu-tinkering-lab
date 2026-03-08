/**
 * @file status_led.c
 * @brief WS2812 status LED driver using RMT-based led_strip component.
 */
#include "status_led.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

static const char *TAG = "status_led";

#define LED_GPIO 21
#define LED_COUNT 1
#define LED_BRIGHTNESS 32 /* 0-255, keep low — WS2812 LEDs are very bright */
#define BLINK_PERIOD_MS 500

static led_strip_handle_t s_strip;
static volatile status_led_mode_t s_mode = STATUS_LED_OFF;

static void set_color(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

static void led_task(void *arg)
{
    (void)arg;
    bool blink_on = false;
    TickType_t last_toggle = xTaskGetTickCount();

    while (1) {
        switch (s_mode) {
            case STATUS_LED_SCANNING:
                if ((xTaskGetTickCount() - last_toggle) >= pdMS_TO_TICKS(BLINK_PERIOD_MS)) {
                    blink_on = !blink_on;
                    last_toggle = xTaskGetTickCount();
                }
                if (blink_on) {
                    set_color(0, 0, LED_BRIGHTNESS); /* Blue */
                } else {
                    set_color(0, 0, 0);
                }
                break;

            case STATUS_LED_CONNECTED:
                set_color(LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0); /* Yellow-orange */
                break;

            case STATUS_LED_BRIDGING:
                set_color(0, LED_BRIGHTNESS, 0); /* Green */
                break;

            case STATUS_LED_OFF:
            default:
                set_color(0, 0, 0);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t status_led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_strip);
    led_strip_refresh(s_strip);

    /* Run on core 1 alongside the bridge task (core 0 = BTstack) */
    xTaskCreatePinnedToCore(led_task, "status_led", 4096, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "Status LED initialized on GPIO%d", LED_GPIO);
    return ESP_OK;
}

void status_led_set_mode(status_led_mode_t mode)
{
    s_mode = mode;
}
