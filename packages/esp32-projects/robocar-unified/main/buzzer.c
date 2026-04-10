/**
 * @file buzzer.c
 * @brief Piezo buzzer control via GPIO toggle (square wave generation)
 *
 * Extracted from robocar-main/main/main.c play_sound() function.
 */

#include "buzzer.h"
#include "pin_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "buzzer";

esp_err_t buzzer_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIEZO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(PIEZO_PIN, 0);

    ESP_LOGI(TAG, "Buzzer initialized on GPIO%d", PIEZO_PIN);
    return ESP_OK;
}

void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    if (frequency_hz == 0) return;

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
