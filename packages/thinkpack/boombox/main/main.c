/**
 * @file main.c
 * @brief ThinkPack Boombox — music + rhythm box for the ThinkPack modular toy mesh.
 *
 * Hardware (ESP32-S3 SuperMini):
 *   GPIO 4  — WS2812 single LED
 *   GPIO 5  — Passive piezo (LEDC PWM channel 0)
 *   GPIO 1  — Pot 1: tempo (ADC1_CH0)
 *   GPIO 2  — Pot 2: pitch (ADC1_CH1)
 *   GPIO 9  — Button (active LOW, internal pull-up)
 *
 * Standalone: pots drive BPM and semitone shift; button cycles melody
 *   patterns; 2-second hold resets to MARCH at beat 0.
 * Group: mesh sync-pulse aligns beat index to leader; leader broadcasts
 *   quarter-beat sync pulses so followers stay in musical-round lock-step.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include <stdint.h>
#include <string.h>

#include "button.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "group_mode.h"
#include "led_single.h"
#include "melody_gen.h"
#include "pot_reader.h"
#include "standalone_mode.h"
#include "thinkpack_ota.h"
#include "thinkpack_power.h"
#include "thinkpack_protocol.h"
#include "tone_engine.h"

static const char *TAG = "boombox";

/* ------------------------------------------------------------------ */
/* Button callbacks                                                    */
/* ------------------------------------------------------------------ */

static void on_short_press(void)
{
    /* Cycle pattern: MARCH → WALTZ → PENTATONIC → SILENCE → MARCH */
    melody_pattern_t cur = melody_gen_get_pattern();
    melody_pattern_t next = (melody_pattern_t)((int)cur + 1);
    if (next >= MELODY_COUNT)
        next = MELODY_SILENCE;
    melody_gen_set_pattern(next);
    ESP_LOGI(TAG, "Pattern -> %d", (int)next);
}

static void on_long_press(void)
{
    /* 2-second hold: soft factory reset — back to MARCH at beat 0 */
    standalone_mode_reset();
    ESP_LOGI(TAG, "Long press: reset to MARCH beat 0");
}

/* ------------------------------------------------------------------ */
/* Tone task — Core 1, 100 Hz control loop                            */
/* ------------------------------------------------------------------ */

static void tone_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        pot_reader_tick(now_ms);
        button_tick(now_ms);
        group_mode_tick(now_ms);
        standalone_mode_tick(now_ms);
        led_single_tick(now_ms);
        vTaskDelay(pdMS_TO_TICKS(10)); /* 100 Hz */
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "ThinkPack Boombox starting");

    ESP_ERROR_CHECK(led_single_init());
    ESP_ERROR_CHECK(tone_engine_init());
    ESP_ERROR_CHECK(pot_reader_init());
    ESP_ERROR_CHECK(button_init(on_short_press, on_long_press));

    group_mode_init();
    standalone_mode_init();

    thinkpack_mesh_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.box_type = BOX_BOOMBOX;
    cfg.capabilities = CAP_AUDIO_OUT | CAP_POTS | CAP_RHYTHM;
    cfg.channel = 1;
    cfg.battery_level = 100;
    cfg.beacon_interval_ms = 500;
    strncpy(cfg.name, "boombox", THINKPACK_BOX_NAME_LEN - 1);

    ESP_ERROR_CHECK(thinkpack_mesh_init(&cfg));
    thinkpack_ota_receiver_chain_callback((thinkpack_ota_chained_cb_t)group_mode_on_event, NULL);
    ESP_ERROR_CHECK(thinkpack_ota_receiver_init(BOX_BOOMBOX));
    ESP_ERROR_CHECK(thinkpack_mesh_start());

    /* Power monitor — after mesh.  GPIO1/ADC1_CH0 by default: verify against
     * WIRING.md before committing to a real board. */
    (void)thinkpack_power_init(
        &(power_config_t){.adc_gpio = 1, .tick_interval_ms = 5000, .divider_ratio_x10 = 20});

    ESP_LOGI(TAG, "Mesh started — spawning tone task on Core 1");

    xTaskCreatePinnedToCore(tone_task, "box_tone", 4096, NULL, 5, NULL, 1);

    /* Main task parks here; tone_task owns all periodic work */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "peers=%u beat=%lu", (unsigned)thinkpack_mesh_peer_count(),
                 (unsigned long)standalone_mode_get_beat_index());
    }
}
