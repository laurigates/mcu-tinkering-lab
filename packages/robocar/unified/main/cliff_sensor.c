/**
 * @file cliff_sensor.c
 * @brief Front and rear floor sensors on the MCP23017. See cliff_sensor.h.
 */

#include "cliff_sensor.h"

#ifndef CLIFF_SENSOR_HOST_TEST

#include "esp_log.h"
#include "gpio_expander.h"
#include "pin_config.h"

static const char *TAG = "cliff_sensor";

_Static_assert(((GPIO_EXPANDER_OUTPUT_ONLY_MASK >> CLIFF_FRONT_EXPANDER_PIN) & 1u) == 0u,
               "CLIFF_FRONT_EXPANDER_PIN is output-only (GPA7/GPB7)");
_Static_assert(((GPIO_EXPANDER_OUTPUT_ONLY_MASK >> CLIFF_REAR_EXPANDER_PIN) & 1u) == 0u,
               "CLIFF_REAR_EXPANDER_PIN is output-only (GPA7/GPB7)");
_Static_assert(CLIFF_FRONT_EXPANDER_PIN != CLIFF_REAR_EXPANDER_PIN,
               "the two floor sensors need their own expander pins");

static bool s_available = false;

esp_err_t cliff_sensor_init(void)
{
    s_available = false;
    if (!gpio_expander_available()) {
        ESP_LOGW(TAG, "no MCP23017 — running without the cliff reflex");
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t ret = gpio_expander_set_mode(CLIFF_FRONT_EXPANDER_PIN, GPIO_EXPANDER_INPUT_PULLUP);
    if (ret == ESP_OK) {
        ret = gpio_expander_set_mode(CLIFF_REAR_EXPANDER_PIN, GPIO_EXPANDER_INPUT_PULLUP);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "configuring expander pins %u/%u failed: %s", CLIFF_FRONT_EXPANDER_PIN,
                 CLIFF_REAR_EXPANDER_PIN, esp_err_to_name(ret));
        return ret;
    }
    s_available = true;
    ESP_LOGI(TAG, "floor sensors on MCP23017 pins %u (front) and %u (rear)",
             CLIFF_FRONT_EXPANDER_PIN, CLIFF_REAR_EXPANDER_PIN);
    return ESP_OK;
}

bool cliff_sensor_available(void)
{
    return s_available;
}

void cliff_sensor_read(cliff_sample_t *out)
{
    out->valid = false;
    out->floor[CLIFF_FRONT] = false;
    out->floor[CLIFF_REAR] = false;
    if (!s_available) {
        return;
    }
    uint16_t port = 0;
    if (gpio_expander_read_port(&port) != ESP_OK) {
        return;
    }
    out->valid = true;
    out->floor[CLIFF_FRONT] = (((port >> CLIFF_FRONT_EXPANDER_PIN) & 1u) == CLIFF_FLOOR_LEVEL);
    out->floor[CLIFF_REAR] = (((port >> CLIFF_REAR_EXPANDER_PIN) & 1u) == CLIFF_FLOOR_LEVEL);
}

#else /* CLIFF_SENSOR_HOST_TEST */

static bool s_available = false;
static cliff_sample_t s_sample = {.valid = false};

void cliff_sensor_test_set(bool available, bool valid, bool front_floor, bool rear_floor)
{
    s_available = available;
    s_sample.valid = valid;
    s_sample.floor[CLIFF_FRONT] = front_floor;
    s_sample.floor[CLIFF_REAR] = rear_floor;
}

esp_err_t cliff_sensor_init(void)
{
    return s_available ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool cliff_sensor_available(void)
{
    return s_available;
}

void cliff_sensor_read(cliff_sample_t *out)
{
    *out = s_sample;
    if (!s_available) {
        out->valid = false;
    }
}

#endif /* CLIFF_SENSOR_HOST_TEST */
