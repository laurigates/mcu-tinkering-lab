/**
 * @file ultrasonic.c
 * @brief Front and rear RCWL-1601 rangefinders over I2C. See ultrasonic.h.
 *
 * The read cycle is ultrasonic_core.c, shared with the host tests. This file
 * supplies its two bus operations, each a complete select → transfer → release
 * through i2c_bus.c, and the per-sensor table.
 *
 * Both sensors answer at the same fixed address on different mux channels, so
 * one i2c_dev_t serves both: the channel select decides which one hears it.
 */

#include "ultrasonic.h"

#include <stddef.h>

#include "pin_config.h"

/* Per-sensor table, shared by both builds. */
static const struct {
    uint8_t channel;
    const char *name;
} k_sensors[ULTRASONIC_COUNT] = {
    [ULTRASONIC_FRONT] = {I2C_BUS_CHANNEL_RANGER_FRONT, "front"},
    [ULTRASONIC_REAR] = {I2C_BUS_CHANNEL_RANGER_REAR, "rear"},
};

uint8_t ultrasonic_channel(ultrasonic_id_t id)
{
    return ((unsigned)id < ULTRASONIC_COUNT) ? k_sensors[id].channel : 0xFFu;
}

const char *ultrasonic_name(ultrasonic_id_t id)
{
    return ((unsigned)id < ULTRASONIC_COUNT) ? k_sensors[id].name : "?";
}

#ifndef ULTRASONIC_HOST_TEST

#include <string.h>

#include "esp_log.h"
#include "i2c_bus.h"
#include "i2cdev.h"

static const char *TAG = "ultrasonic";

static i2c_dev_t s_dev;
static bool s_initialized = false;
static ultrasonic_core_t s_state[ULTRASONIC_COUNT];

/* -------------------------------------------------------------------------
 * Bus operations — each holds the bus for one short transaction only
 * ---------------------------------------------------------------------- */

static bool bus_trigger(void *ctx, uint8_t channel)
{
    (void)ctx;
    if (i2c_bus_select_channel(channel) != ESP_OK) {
        return false;
    }
    const uint8_t cmd = RCWL1601_CMD_TRIGGER;
    esp_err_t ret = i2c_dev_take_mutex(&s_dev);
    if (ret == ESP_OK) {
        ret = i2c_dev_write(&s_dev, NULL, 0, &cmd, sizeof(cmd));
        i2c_dev_give_mutex(&s_dev);
    }
    i2c_bus_release();
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "ch%u trigger failed: %s", (unsigned)channel, esp_err_to_name(ret));
    }
    return ret == ESP_OK;
}

static bool bus_read(void *ctx, uint8_t channel, uint8_t raw[RCWL1601_READ_LEN])
{
    (void)ctx;
    if (i2c_bus_select_channel(channel) != ESP_OK) {
        return false;
    }
    esp_err_t ret = i2c_dev_take_mutex(&s_dev);
    if (ret == ESP_OK) {
        ret = i2c_dev_read(&s_dev, NULL, 0, raw, RCWL1601_READ_LEN);
        i2c_dev_give_mutex(&s_dev);
    }
    i2c_bus_release();
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "ch%u read failed: %s", (unsigned)channel, esp_err_to_name(ret));
    }
    return ret == ESP_OK;
}

static const ultrasonic_bus_t k_bus = {
    .trigger = bus_trigger,
    .read = bus_read,
    .ctx = NULL,
};

/* -------------------------------------------------------------------------
 * Host-test injection (on-target: the setter is a no-op)
 * ---------------------------------------------------------------------- */
void ultrasonic_test_set_distance(ultrasonic_id_t id, uint16_t cm)
{
    (void)id;
    (void)cm;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t ultrasonic_init(void)
{
    if (s_initialized) {
        return ESP_OK; /* idempotent */
    }
    if (!i2c_bus_is_ready()) {
        ESP_LOGW(TAG, "I2C bus not up — rangefinders unavailable");
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_dev, 0, sizeof(s_dev));
    s_dev.port = I2C_NUM_0;
    s_dev.addr = RCWL1601_I2C_ADDR;
    s_dev.cfg.sda_io_num = I2C_SDA_PIN;
    s_dev.cfg.scl_io_num = I2C_SCL_PIN;
    s_dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;
    esp_err_t ret = i2c_dev_create_mutex(&s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_dev_create_mutex failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        ultrasonic_core_reset(&s_state[i]);
    }
    s_initialized = true;
    ESP_LOGI(TAG, "RCWL-1601 x2 at 0x%02X: front ch%u, rear ch%u, %u ms conversion",
             RCWL1601_I2C_ADDR, (unsigned)I2C_BUS_CHANNEL_RANGER_FRONT,
             (unsigned)I2C_BUS_CHANNEL_RANGER_REAR, (unsigned)ULTRASONIC_CONVERSION_MS);
    return ESP_OK;
}

ultrasonic_result_t ultrasonic_poll(ultrasonic_id_t id, uint32_t now_ms, uint16_t *distance_cm)
{
    if (distance_cm == NULL) {
        return ULTRASONIC_ERROR;
    }
    if (!s_initialized || (unsigned)id >= ULTRASONIC_COUNT) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ULTRASONIC_ERROR;
    }
    return ultrasonic_core_step(&s_state[id], k_sensors[id].channel, now_ms, &k_bus, distance_cm);
}

esp_err_t ultrasonic_deinit(void)
{
    if (s_initialized) {
        i2c_dev_delete_mutex(&s_dev);
        s_initialized = false;
    }
    return ESP_OK;
}

/* =========================================================================
 * Host-test build (ULTRASONIC_HOST_TEST=1) — a stub per sensor
 * ========================================================================= */
#else /* ULTRASONIC_HOST_TEST */

#include <stdbool.h>

static uint16_t s_injected[ULTRASONIC_COUNT] = {ULTRASONIC_DIST_ERROR, ULTRASONIC_DIST_ERROR};
static bool s_initialized = false;

void ultrasonic_test_set_distance(ultrasonic_id_t id, uint16_t cm)
{
    if ((unsigned)id < ULTRASONIC_COUNT) {
        s_injected[id] = cm;
    }
}

esp_err_t ultrasonic_init(void)
{
    s_initialized = true;
    return ESP_OK;
}

ultrasonic_result_t ultrasonic_poll(ultrasonic_id_t id, uint32_t now_ms, uint16_t *distance_cm)
{
    (void)now_ms;
    if (distance_cm == NULL) {
        return ULTRASONIC_ERROR;
    }
    if (!s_initialized || (unsigned)id >= ULTRASONIC_COUNT) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ULTRASONIC_ERROR;
    }
    *distance_cm = s_injected[id];
    return (s_injected[id] == ULTRASONIC_DIST_ERROR) ? ULTRASONIC_ERROR : ULTRASONIC_READY;
}

esp_err_t ultrasonic_deinit(void)
{
    s_initialized = false;
    return ESP_OK;
}

#endif /* ULTRASONIC_HOST_TEST */
