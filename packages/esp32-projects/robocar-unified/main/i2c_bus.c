/**
 * @file i2c_bus.c
 * @brief TCA9548A-based I2C bus multiplexer with thread-safe access
 */

#include "i2c_bus.h"
#include "pin_config.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <i2cdev.h>
#include <pca9685.h>
#include <tca9548.h>

static const char *TAG = "i2c_bus";

static i2c_dev_t s_tca9548;
static i2c_dev_t s_pca9685;
static SemaphoreHandle_t s_bus_mutex;
static bool s_initialized = false;

esp_err_t i2c_bus_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Initialize i2cdev library
    esp_err_t ret = i2cdev_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create bus mutex
    s_bus_mutex = xSemaphoreCreateRecursiveMutex();
    if (!s_bus_mutex) {
        ESP_LOGE(TAG, "Failed to create bus mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize TCA9548A multiplexer
    ret = tca9548_init_desc(&s_tca9548, TCA9548A_ADDR, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9548A init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Disable all channels initially
    ret = tca9548_set_channels(&s_tca9548, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9548A set_channels failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "TCA9548A initialized at 0x%02X", TCA9548A_ADDR);

    // Initialize PCA9685 via TCA9548A channel 0
    ret = tca9548_set_channels(&s_tca9548, TCA9548_CHANNEL0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select PCA9685 channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pca9685_init_desc(&s_pca9685, PCA9685_ADDR, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pca9685_init(&s_pca9685);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pca9685_set_pwm_frequency(&s_pca9685, PCA9685_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 set_pwm_frequency failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // All channels off initially
    ret = pca9685_set_pwm_value(&s_pca9685, PCA9685_CHANNEL_ALL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 all-off failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Disable all TCA9548A channels after init
    tca9548_set_channels(&s_tca9548, 0x00);

    s_initialized = true;
    ESP_LOGI(TAG, "I2C bus initialized: TCA9548A(0x%02X) + PCA9685(0x%02X) @ %dHz", TCA9548A_ADDR,
             PCA9685_ADDR, PCA9685_FREQ_HZ);
    return ESP_OK;
}

esp_err_t i2c_bus_select_channel(uint8_t channel)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTakeRecursive(s_bus_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Bus mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = tca9548_set_channels(&s_tca9548, (1 << channel));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Channel select failed: %s", esp_err_to_name(ret));
        xSemaphoreGiveRecursive(s_bus_mutex);
        return ret;
    }

    return ESP_OK;
}

esp_err_t i2c_bus_release(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGiveRecursive(s_bus_mutex);
    return ESP_OK;
}

i2c_dev_t *i2c_bus_get_pca9685(void)
{
    return &s_pca9685;
}

esp_err_t i2c_bus_pca9685_set(uint8_t channel, uint16_t value)
{
    esp_err_t ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_PCA9685);
    if (ret != ESP_OK)
        return ret;

    ret = pca9685_set_pwm_value(&s_pca9685, channel, value);

    i2c_bus_release();
    return ret;
}

esp_err_t i2c_bus_pca9685_set_multi(uint8_t first_ch, uint8_t count, const uint16_t *values)
{
    esp_err_t ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_PCA9685);
    if (ret != ESP_OK)
        return ret;

    ret = pca9685_set_pwm_values(&s_pca9685, first_ch, count, values);

    i2c_bus_release();
    return ret;
}

void i2c_bus_deinit(void)
{
    if (!s_initialized)
        return;

    // All PCA9685 channels off
    tca9548_set_channels(&s_tca9548, TCA9548_CHANNEL0);
    pca9685_set_pwm_value(&s_pca9685, PCA9685_CHANNEL_ALL, 0);
    tca9548_set_channels(&s_tca9548, 0x00);

    pca9685_free_desc(&s_pca9685);
    tca9548_free_desc(&s_tca9548);

    if (s_bus_mutex) {
        vSemaphoreDelete(s_bus_mutex);
        s_bus_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "I2C bus deinitialized");
}
