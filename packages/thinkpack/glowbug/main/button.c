/**
 * @file button.c
 * @brief Debounced button driver — GPIO9, active LOW, internal pull-up.
 *
 * ISR posts a notification to a dedicated task that enforces a 50 ms
 * debounce window before firing the user callback.  The task also
 * polls the pin in 10 ms increments while the button is held to fire
 * a long-press callback at BUTTON_LONG_PRESS_MS; a long press inhibits
 * the short-press callback for the same press.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "button.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "button";

static button_callback_t s_short_cb;
static button_callback_t s_long_cb;
static TaskHandle_t s_task_handle;

/* ------------------------------------------------------------------ */
/* ISR                                                                 */
/* ------------------------------------------------------------------ */

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t higher_prio_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

/* ------------------------------------------------------------------ */
/* Debounce + long-press task                                          */
/* ------------------------------------------------------------------ */

static void button_task(void *arg)
{
    (void)arg;

    for (;;) {
        /* Block until ISR fires */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Debounce: wait then verify pin is still low */
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        if (gpio_get_level(BUTTON_GPIO) != 0) {
            /* Spurious edge — drain and wait for the next one */
            ulTaskNotifyTake(pdTRUE, 0);
            continue;
        }

        /* Pin confirmed LOW. Poll in 10 ms increments until release to
         * detect long-press crossing.  Once the long-press threshold is
         * hit we fire the long callback and suppress the short callback
         * for this press. */
        uint32_t held_ms = BUTTON_DEBOUNCE_MS;
        bool long_fired = false;

        while (gpio_get_level(BUTTON_GPIO) == 0) {
            if (!long_fired && held_ms >= BUTTON_LONG_PRESS_MS) {
                long_fired = true;
                ESP_LOGI(TAG, "Long press detected (%u ms)", (unsigned)held_ms);
                if (s_long_cb != NULL) {
                    s_long_cb();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            held_ms += 10;
        }

        if (!long_fired && s_short_cb != NULL) {
            s_short_cb();
        }

        /* Drain any spurious notifications accumulated during hold */
        ulTaskNotifyTake(pdTRUE, 0);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t button_init(button_callback_t on_short, button_callback_t on_long)
{
    s_short_cb = on_short;
    s_long_cb = on_long;

    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    esp_err_t ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t task_ret = xTaskCreate(button_task, "btn_debounce", 2048, NULL, 10, &s_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Debounce task creation failed");
        return ESP_ERR_NO_MEM;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means ISR service already installed — fine */
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISR handler add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Button initialized on GPIO%d (active LOW, debounce %d ms, long-press %d ms)",
             BUTTON_GPIO, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS);
    return ESP_OK;
}
