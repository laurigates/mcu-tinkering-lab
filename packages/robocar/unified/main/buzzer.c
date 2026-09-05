/**
 * @file buzzer.c
 * @brief Piezo buzzer control via GPIO toggle (square wave generation)
 *
 * Extracted from robocar-main/main/main.c play_sound() function.
 */

#include "buzzer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "buzzer";

static bool s_initialized = false;

esp_err_t buzzer_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIEZO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    /* Returned rather than ESP_ERROR_CHECK'd: an abort here reboots the board
     * no matter how forgiving init_hardware() is, which is the failure mode
     * issue #500 exists to remove. The caller logs it and carries on; the
     * tone routines below then no-op instead of toggling an unconfigured pin. */
    const esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(GPIO%d) failed: %s", PIEZO_PIN, esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(PIEZO_PIN, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "Buzzer initialized on GPIO%d", PIEZO_PIN);
    return ESP_OK;
}

bool buzzer_is_initialized(void)
{
    return s_initialized;
}

void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    if (!s_initialized || frequency_hz == 0)
        return;

    uint32_t half_period_us = 500000 / frequency_hz;
    int64_t end_time = esp_timer_get_time() + (int64_t)duration_ms * 1000;

    while (esp_timer_get_time() < end_time) {
        gpio_set_level(PIEZO_PIN, 1);
        esp_rom_delay_us(half_period_us);
        gpio_set_level(PIEZO_PIN, 0);
        esp_rom_delay_us(half_period_us);
    }
}

void buzzer_beep(void)
{
    buzzer_play_tone(1000, 200);
}

void buzzer_melody(void)
{
    // C4 - E4 - G4 - C5
    buzzer_play_tone(262, 200);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(330, 200);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(392, 200);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(523, 400);
}

void buzzer_alert(void)
{
    for (int i = 0; i < 3; i++) {
        buzzer_play_tone(1500, 100);
        vTaskDelay(pdMS_TO_TICKS(50));
        buzzer_play_tone(800, 100);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
