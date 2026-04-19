/**
 * @file imu.c
 * @brief MPU6050 driver using ESP-IDF v5.4 new I2C master API.
 *
 * Inline register-level driver — no managed MPU6050 component required.
 * Uses i2c_master.h (i2c_new_master_bus, i2c_master_bus_add_device) not
 * the deprecated i2c.h legacy API.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "imu.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "imu";

/* MPU6050 register addresses */
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_SCALE_2G 16384.0f /* LSB/g at ±2 g range */

/* Shake detection threshold in g */
#define SHAKE_THRESHOLD_G 1.5f

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

/* Previous acceleration for delta computation in imu_detect_shake() */
static float s_prev_ax = 0.0f;
static float s_prev_ay = 0.0f;
static float s_prev_az = 1.0f; /* assume resting upright */

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/** Write a single byte to an MPU6050 register. */
static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

/** Read `len` bytes starting at `reg` into `out`. */
static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    esp_err_t ret = i2c_master_transmit(s_dev, &reg, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_receive(s_dev, out, len, pdMS_TO_TICKS(100));
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t imu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = IMU_SDA_GPIO,
        .scl_io_num = IMU_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wake MPU6050: clear SLEEP bit in PWR_MGMT_1 */
    ret = write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized on SDA=%d SCL=%d", IMU_SDA_GPIO, IMU_SCL_GPIO);
    return ESP_OK;
}

esp_err_t imu_read_accel(float *ax, float *ay, float *az)
{
    uint8_t raw[6];
    esp_err_t ret = read_regs(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Accel read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t rx = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ry = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t rz = (int16_t)((raw[4] << 8) | raw[5]);

    *ax = (float)rx / MPU6050_ACCEL_SCALE_2G;
    *ay = (float)ry / MPU6050_ACCEL_SCALE_2G;
    *az = (float)rz / MPU6050_ACCEL_SCALE_2G;

    return ESP_OK;
}

bool imu_detect_shake(void)
{
    float ax, ay, az;
    if (imu_read_accel(&ax, &ay, &az) != ESP_OK) {
        return false;
    }

    float dx = ax - s_prev_ax;
    float dy = ay - s_prev_ay;
    float dz = az - s_prev_az;

    s_prev_ax = ax;
    s_prev_ay = ay;
    s_prev_az = az;

    float delta_mag = sqrtf(dx * dx + dy * dy + dz * dz);
    return delta_mag > SHAKE_THRESHOLD_G;
}

uint8_t imu_tilt_to_hue(void)
{
    float ax, ay, az;
    if (imu_read_accel(&ax, &ay, &az) != ESP_OK) {
        return 0;
    }

    /* roll angle from atan2(ay, az): range -π..π */
    float roll = atan2f(ay, az);

    /* Map -π..π → 0..255 */
    float normalized = (roll + (float)M_PI) / (2.0f * (float)M_PI);
    return (uint8_t)(normalized * 255.0f);
}
