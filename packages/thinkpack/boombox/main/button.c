/**
 * @file button.c
 * @brief GPIO 9 button — interrupt + debounce + long-press detection.
 *
 * Debounce window: 30 ms. Long press threshold: 2000 ms.
 * ISR sets a flag; button_tick() processes state in the tone task context.
 */

#include "button.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "button";

#define BUTTON_GPIO 9
#define DEBOUNCE_MS 30u
#define LONG_PRESS_MS 2000u

static button_cb_t s_short_cb;
static button_cb_t s_long_cb;

static volatile bool s_isr_fired;
static uint32_t s_press_start_ms;
static bool s_pressed;
static bool s_long_fired;
static uint32_t s_last_isr_ms;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    (void)arg;
    s_isr_fired = true;
}

esp_err_t button_init(button_cb_t on_short_press, button_cb_t on_long_press)
{
    s_short_cb = on_short_press;
    s_long_cb = on_long_press;
    s_isr_fired = false;
    s_pressed = false;
    s_long_fired = false;
    s_press_start_ms = 0;
    s_last_isr_ms = 0;

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means ISR service already installed — that's fine */
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISR handler add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Button initialized on GPIO%d (active LOW, pull-up)", BUTTON_GPIO);
    return ESP_OK;
}

void button_tick(uint32_t now_ms)
{
    if (s_isr_fired) {
        s_isr_fired = false;
        /* Debounce: ignore edges within DEBOUNCE_MS of last ISR */
        if ((now_ms - s_last_isr_ms) < DEBOUNCE_MS) {
            return;
        }
        s_last_isr_ms = now_ms;

        int level = gpio_get_level(BUTTON_GPIO);
        if (level == 0 && !s_pressed) {
            /* Falling edge — button pressed */
            s_pressed = true;
            s_long_fired = false;
            s_press_start_ms = now_ms;
        } else if (level == 1 && s_pressed) {
            /* Rising edge — button released */
            s_pressed = false;
            if (!s_long_fired && s_short_cb) {
                s_short_cb();
            }
        }
    }

    /* Long-press detection */
    if (s_pressed && !s_long_fired) {
        if ((now_ms - s_press_start_ms) >= LONG_PRESS_MS) {
            s_long_fired = true;
            if (s_long_cb) {
                s_long_cb();
            }
        }
    }
}
