#include "mpu6050.h"

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pin_config.h"

#define MPU_I2C i2c1
#define MPU_ADDR 0x68  // AD0 low

// Registers
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

// Scale factors for the ranges configured in mpu6050_init()
#define ACCEL_LSB_PER_G 8192.0f  // +/-4 g
#define GYRO_LSB_PER_DPS 65.5f   // +/-500 dps
#define TEMP_LSB_PER_C 340.0f

static float gyro_bias_dps[3];

static bool reg_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(MPU_I2C, MPU_ADDR, buf, 2, false) == 2;
}

static bool reg_read(uint8_t reg, uint8_t *dst, size_t len)
{
    if (i2c_write_blocking(MPU_I2C, MPU_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(MPU_I2C, MPU_ADDR, dst, len, false) == (int)len;
}

bool mpu6050_init(void)
{
    i2c_init(MPU_I2C, IMU_I2C_BAUD);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    gpio_init(PIN_IMU_INT);
    gpio_set_dir(PIN_IMU_INT, GPIO_IN);
    // No internal pull: the MPU6050 INT output is push-pull, and on the A2
    // stepping internal pull-downs can't hold an undriven input low anyway
    // (RP2350-E9) — a disconnected sensor floats high, which the control
    // loop's deadline pacing tolerates.
    gpio_disable_pulls(PIN_IMU_INT);

    sleep_ms(100);  // sensor power-up

    uint8_t who = 0;
    if (!reg_read(REG_WHO_AM_I, &who, 1) || who != 0x68) {
        return false;
    }

    // Wake from sleep, clock = gyro X PLL (more stable than the internal RC).
    if (!reg_write(REG_PWR_MGMT_1, 0x01)) {
        return false;
    }
    sleep_ms(10);

    // DLPF 2: 94 Hz accel / 98 Hz gyro bandwidth, gyro output rate 1 kHz.
    reg_write(REG_CONFIG, 0x02);
    // Sample rate = 1 kHz / (1 + SMPLRT_DIV) = 500 Hz.
    reg_write(REG_SMPLRT_DIV, 1);
    reg_write(REG_GYRO_CONFIG, 0x08);   // +/-500 dps
    reg_write(REG_ACCEL_CONFIG, 0x08);  // +/-4 g
    // INT: push-pull, active high, latched until any read (a 50 us pulse
    // could fire and vanish while the control loop is still processing).
    reg_write(REG_INT_PIN_CFG, 0x30);
    reg_write(REG_INT_ENABLE, 0x01);  // data ready

    gyro_bias_dps[0] = gyro_bias_dps[1] = gyro_bias_dps[2] = 0.0f;
    return true;
}

bool mpu6050_read(mpu6050_sample_t *out)
{
    uint8_t raw[14];
    if (!reg_read(REG_ACCEL_XOUT_H, raw, sizeof raw)) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        int16_t a = (int16_t)((raw[i * 2] << 8) | raw[i * 2 + 1]);
        int16_t g = (int16_t)((raw[8 + i * 2] << 8) | raw[8 + i * 2 + 1]);
        out->accel_g[i] = (float)a / ACCEL_LSB_PER_G;
        out->gyro_dps[i] = (float)g / GYRO_LSB_PER_DPS - gyro_bias_dps[i];
    }
    int16_t t = (int16_t)((raw[6] << 8) | raw[7]);
    out->temp_c = (float)t / TEMP_LSB_PER_C + 36.53f;
    return true;
}

void mpu6050_calibrate_gyro(int samples)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int got = 0;
    const float saved[3] = {gyro_bias_dps[0], gyro_bias_dps[1], gyro_bias_dps[2]};
    gyro_bias_dps[0] = gyro_bias_dps[1] = gyro_bias_dps[2] = 0.0f;

    for (int i = 0; i < samples; i++) {
        mpu6050_sample_t s;
        // Calibration takes ~1 s; keep a possibly-running watchdog fed
        // (harmless no-op when the watchdog hasn't been enabled).
        watchdog_update();
        sleep_ms(2);
        if (mpu6050_read(&s)) {
            for (int j = 0; j < 3; j++) {
                sum[j] += s.gyro_dps[j];
            }
            got++;
        }
    }
    if (got > samples / 2) {
        for (int j = 0; j < 3; j++) {
            gyro_bias_dps[j] = sum[j] / (float)got;
        }
    } else {
        // Too many read failures — keep the previous bias.
        for (int j = 0; j < 3; j++) {
            gyro_bias_dps[j] = saved[j];
        }
    }
}

bool mpu6050_data_ready(void)
{
    return gpio_get(PIN_IMU_INT);
}
