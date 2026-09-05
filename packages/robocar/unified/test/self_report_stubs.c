/**
 * @file self_report_stubs.c
 * @brief Host stubs for everything self_report.c links against.
 *
 * Only the health accessors have behaviour — they read g_stub, which the test
 * sets per case. Everything else (narration, the speech queue, MQTT, the
 * FreeRTOS task) is inert: self_report_collect() and
 * self_report_format_facts() are what the test exercises, and the monitor task
 * is never started.
 */

#include "self_report_stubs.h"

#include <stddef.h>
#include <stdint.h>

#include "audio_player.h"
#include "credentials_loader.h"
#include "dialogue_style.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gemini_backend.h"
#include "gpio_expander.h"
#include "i2c_bus.h"
#include "led_controller.h"
#include "motor_controller.h"
#include "mqtt_logger.h"
#include "servo_controller.h"
#include "speech_queue.h"
#include "voice_persona.h"
#include "wifi_manager.h"

self_report_stub_state_t g_stub;

void stub_reset_healthy(void)
{
    g_stub = (self_report_stub_state_t){
        .i2c_bus_ready = true,
        .motors_initialized = true,
        .leds_initialized = true,
        .servos_initialized = true,
        .buzzer_initialized = true,
        .expander_available = false,
        .audio_ready = true,
        .wifi_connected = true,
        .ssid = "testnet",
        .api_key = "test-key",
        .version = "0.1.0",
    };
}

/* -------------------------------------------------------------------------- */
/* Health accessors — the only stubs with behaviour                            */
/* -------------------------------------------------------------------------- */

bool i2c_bus_is_ready(void)
{
    return g_stub.i2c_bus_ready;
}
bool motor_is_initialized(void)
{
    return g_stub.motors_initialized;
}
bool led_is_initialized(void)
{
    return g_stub.leds_initialized;
}
bool servo_is_initialized(void)
{
    return g_stub.servos_initialized;
}
bool buzzer_is_initialized(void)
{
    return g_stub.buzzer_initialized;
}
bool gpio_expander_available(void)
{
    return g_stub.expander_available;
}
bool audio_player_is_ready(void)
{
    return g_stub.audio_ready;
}
bool wifi_is_connected(void)
{
    return g_stub.wifi_connected;
}
const char *get_wifi_ssid(void)
{
    return g_stub.ssid;
}
const char *get_gemini_api_key(void)
{
    return g_stub.api_key;
}

static esp_app_desc_t s_desc;

const esp_app_desc_t *esp_app_get_description(void)
{
    /* The version string is a fixed-size array in the real descriptor, so copy
     * rather than alias — the test uses a 30-character worst case. */
    size_t i = 0;
    for (; g_stub.version != NULL && g_stub.version[i] != '\0' && i < sizeof(s_desc.version) - 1;
         ++i) {
        s_desc.version[i] = g_stub.version[i];
    }
    s_desc.version[i] = '\0';
    return &s_desc;
}

/* -------------------------------------------------------------------------- */
/* Inert                                                                       */
/* -------------------------------------------------------------------------- */

int64_t esp_timer_get_time(void)
{
    return 0;
}

bool mqtt_logger_is_connected(void)
{
    return false;
}
esp_err_t mqtt_logger_publish_status(const char *status_json)
{
    (void)status_json;
    return ESP_OK;
}

esp_err_t speech_queue_post(const char *text)
{
    (void)text;
    return ESP_OK;
}

esp_err_t gemini_backend_narrate(const char *facts, bool is_update, char *out, size_t out_len)
{
    (void)facts;
    (void)is_update;
    if (out != NULL && out_len > 0) {
        out[0] = '\0';
    }
    return ESP_FAIL;
}

const char *dialogue_style_pick(const dialogue_pool_t *pool, dialogue_slot_t slot)
{
    (void)pool;
    (void)slot;
    return "";
}

void dialogue_style_note_spoken(const char *line)
{
    (void)line;
}

static const voice_persona_t s_persona;

const voice_persona_t *voice_persona_get(void)
{
    return &s_persona;
}

/* FreeRTOS: the monitor task is never started by this test. */
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack, void *arg,
                                   UBaseType_t prio, TaskHandle_t *handle, BaseType_t core)
{
    (void)fn;
    (void)name;
    (void)stack;
    (void)arg;
    (void)prio;
    (void)handle;
    (void)core;
    return pdFAIL;
}

void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}

BaseType_t xPortGetCoreID(void)
{
    return 0;
}
