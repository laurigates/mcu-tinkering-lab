/**
 * @file power_manager.c
 * @brief Pack monitor, power button and shutdown (see power_manager.h).
 */

#include "power_manager.h"

#include <stdatomic.h>
#include <stdio.h>

#include "audio_player.h"
#include "config.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_expander.h"
#include "mqtt_logger.h"
#include "pack_sense.h"
#include "pin_config.h"
#include "power_core.h"
#include "power_latch.h"
#include "reactive_controller.h"
#include "servo_controller.h"
#include "speech_queue.h"

static const char *TAG = "power";

static atomic_uint_fast32_t s_pack_mv; /* filtered; 0 = no reading */
static atomic_bool s_shutting_down;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint32_t power_manager_pack_mv(void)
{
    return (uint32_t)atomic_load(&s_pack_mv);
}

/** Wait until the line has played, the player has gone quiet, or the cap. */
static void wait_for_speech(void)
{
    /* The fetch takes a moment to reach the player: give it a second before
     * "not active" can mean "finished". */
    const uint32_t start = now_ms();
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (audio_player_is_active() && (uint32_t)(now_ms() - start) < POWER_SPEAK_MAX_MS) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(300)); /* the amplifier's tail */
}

static void publish_shutdown(const char *reason, uint32_t pack_mv)
{
    char json[96];
    snprintf(json, sizeof(json), "{\"event\":\"shutdown\",\"reason\":\"%s\",\"pack_mv\":%u}",
             reason, (unsigned)pack_mv);
    /* Best effort: MQTT may be down, and nothing here waits for it. */
    (void)mqtt_logger_publish(POWER_MQTT_TOPIC, json, 1, false);
}

/**
 * Switch the robot off, once. The order matters: the wheels stop and the head
 * parks first, so a robot that is still talking is not still driving; HOLD is
 * released last. The carrier then cuts the supply when the button is up, and a
 * robot that is still powered after the grace period (USB power, or the button
 * still down) goes to deep sleep instead of running on with the planner awake.
 */
static void shutdown_robot(const char *reason, const char *spoken, uint32_t pack_mv)
{
    if (atomic_exchange(&s_shutting_down, true)) {
        return;
    }
    ESP_LOGW(TAG, "switching off: %s (pack %u mV)", reason, (unsigned)pack_mv);

    reactive_controller_set_hold(true); /* wheels coast on every executor tick */
    (void)reactive_controller_head_hold(REACTIVE_HEAD_PAN, SERVO_PAN_CENTER, 0);
    (void)reactive_controller_head_hold(REACTIVE_HEAD_TILT, SERVO_TILT_CENTER, 0);

    publish_shutdown(reason, pack_mv);

    speech_queue_flush();
    if (speech_queue_post(spoken) == ESP_OK) {
        wait_for_speech();
    }

    power_latch_release();
    vTaskDelay(pdMS_TO_TICKS(POWER_OFF_GRACE_MS));

    ESP_LOGW(TAG, "still powered after HOLD was released: deep sleep");
    esp_deep_sleep_start();
}

static void publish_pack(power_pack_state_t state, uint32_t pack_mv)
{
    char json[64];
    snprintf(json, sizeof(json), "{\"pack_mv\":%u,\"state\":\"%s\"}", (unsigned)pack_mv,
             power_pack_state_name(state));
    (void)mqtt_logger_publish(POWER_MQTT_TOPIC, json, 0, false);
}

/** Read the button: SENSE is low while it is down (the carrier's pull-up and
 *  d_sense). @return false when the read failed. */
static bool read_button(bool *pressed)
{
    bool level = true;
    if (gpio_expander_read(POWER_BUTTON_EXPANDER_PIN, &level) != ESP_OK) {
        return false;
    }
    *pressed = !level;
    return true;
}

static void power_task(void *arg)
{
    (void)arg;

    const bool adc_ok = (pack_sense_init() == ESP_OK);
    if (!adc_ok) {
        ESP_LOGW(TAG, "no pack reading: low-pack cutoff off");
    }

    bool button_ok = false;
    if (gpio_expander_available() &&
        gpio_expander_set_mode(POWER_BUTTON_EXPANDER_PIN, GPIO_EXPANDER_INPUT_PULLUP) == ESP_OK) {
        button_ok = true;
    } else {
        ESP_LOGW(TAG, "no expander button: switching off from the button is unavailable");
    }

    const power_pack_config_t cfg = POWER_PACK_CONFIG_DEFAULT;
    power_pack_t pack;
    power_pack_init(&pack, &cfg);
    power_button_t button;
    power_button_init(&button, POWER_BUTTON_LONG_PRESS_MS);

    uint32_t last_publish = now_ms();
    power_pack_state_t last_logged = POWER_PACK_NO_SAMPLE;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(POWER_POLL_MS)) == pdFALSE) {
            vTaskDelay(1);
            wake = xTaskGetTickCount();
        }
        const uint32_t now = now_ms();

        if (adc_ok) {
            uint32_t mv = 0;
            if (pack_sense_read_mv(&mv) == ESP_OK) {
                const power_pack_state_t state = power_pack_update(&pack, now, mv);
                atomic_store(&s_pack_mv, power_pack_filtered_mv(&pack));
                if (state != last_logged) {
                    ESP_LOGI(TAG, "pack %s, %u mV (raw %u mV)", power_pack_state_name(state),
                             (unsigned)power_pack_filtered_mv(&pack), (unsigned)mv);
                    last_logged = state;
                }
                if (state == POWER_PACK_TRIPPED) {
                    shutdown_robot("low_pack", "Battery low. Switching off.",
                                   power_pack_filtered_mv(&pack));
                }
            } else {
                power_pack_read_failed(&pack);
            }
        }

        if (button_ok) {
            bool pressed = false;
            const bool valid = read_button(&pressed);
            if (power_button_update(&button, now, valid, pressed) == POWER_BUTTON_LONG_PRESS) {
                shutdown_robot("button", "Switching off.", power_manager_pack_mv());
            }
        }

        if ((uint32_t)(now - last_publish) >= POWER_MQTT_PERIOD_MS) {
            last_publish = now;
            if (adc_ok) {
                publish_pack(pack.state, power_manager_pack_mv());
            }
        }
    }
}

esp_err_t power_manager_start(void)
{
    if (!FEATURE_POWER_LATCH_ENABLED) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (xTaskCreatePinnedToCore(power_task, "power", POWER_TASK_STACK_SIZE, NULL,
                                POWER_TASK_PRIORITY, NULL, POWER_TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
