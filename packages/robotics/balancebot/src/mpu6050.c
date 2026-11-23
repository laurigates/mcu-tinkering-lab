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

// Per-transfer I2C deadline. The longest transfer, the 14-byte burst at
// 400 kHz, takes about 0.35 ms. Without a deadline the SDK's blocking calls
// wait forever for TX_EMPTY / STOP_DET, so a slave holding SCL low (or a
// controller stuck mid-transfer) hangs core 1 before or inside its loop.
#define I2C_TIMEOUT_US 1000
#define BUS_RECOVERY_HALF_PERIOD_US 5  // ~100 kHz bit-banged clock

static float gyro_bias_dps[3];
static uint32_t bus_recoveries;  // stuck-bus recoveries since boot

static void bus_attach(void)
{
    i2c_init(MPU_I2C, IMU_I2C_BAUD);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);
}

// Open-drain line: driving it means pulling low; releasing means input, held
// high by the pull-ups (internal plus the GY-521's own).
static void line_low(uint pin)
{
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

static void line_release(uint pin)
{
    gpio_set_dir(pin, GPIO_IN);
}

// Free a bus a slave is holding. A reset in the middle of a read leaves the
// MPU6050 driving SDA low, waiting for clocks that never come, and it keeps
// that state across a firmware reboot because USB keeps it powered. Detach
// the controller, clock SCL up to nine times until SDA is released, send a
// STOP, and re-attach. Returns true if a line was found held low.
static bool bus_recover(void)
{
    i2c_deinit(MPU_I2C);
    const uint pins[2] = {PIN_IMU_SDA, PIN_IMU_SCL};
    for (int i = 0; i < 2; i++) {
        gpio_init(pins[i]);
        gpio_pull_up(pins[i]);
        line_release(pins[i]);
    }
    busy_wait_us(BUS_RECOVERY_HALF_PERIOD_US);
    const bool stuck = !gpio_get(PIN_IMU_SDA) || !gpio_get(PIN_IMU_SCL);
    if (stuck) {
        for (int i = 0; i < 9 && !gpio_get(PIN_IMU_SDA); i++) {
            line_low(PIN_IMU_SCL);
            busy_wait_us(BUS_RECOVERY_HALF_PERIOD_US);
            line_release(PIN_IMU_SCL);
            // A slave may stretch the clock; wait a bounded time for SCL.
            for (int w = 0; w < 100 && !gpio_get(PIN_IMU_SCL); w++) {
                busy_wait_us(1);
            }
            busy_wait_us(BUS_RECOVERY_HALF_PERIOD_US);
        }
        // STOP: SDA rises while SCL is high.
        line_low(PIN_IMU_SDA);
        busy_wait_us(BUS_RECOVERY_HALF_PERIOD_US);
        line_release(PIN_IMU_SDA);
        busy_wait_us(BUS_RECOVERY_HALF_PERIOD_US);
        bus_recoveries++;
    }
    bus_attach();
    return stuck;
}

// A timed-out transfer leaves the controller mid-transaction: recover the
// bus and reset the controller before the next one.
static bool transfer_ok(int ret, int expected)
{
    if (ret == PICO_ERROR_TIMEOUT) {
        bus_recover();
        return false;
    }
    return ret == expected;
}

static bool reg_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return transfer_ok(i2c_write_timeout_us(MPU_I2C, MPU_ADDR, buf, 2, false, I2C_TIMEOUT_US), 2);
}

static bool reg_read(uint8_t reg, uint8_t *dst, size_t len)
{
    if (!transfer_ok(i2c_write_timeout_us(MPU_I2C, MPU_ADDR, &reg, 1, true, I2C_TIMEOUT_US), 1)) {
        return false;
    }
    return transfer_ok(i2c_read_timeout_us(MPU_I2C, MPU_ADDR, dst, len, false, I2C_TIMEOUT_US),
                       (int)len);
}

uint32_t mpu6050_bus_recoveries(void)
{
    return bus_recoveries;
}

bool mpu6050_init(void)
{
    // Check the bus before the first transfer: after a reboot mid-read the
    // MPU6050 can still be holding SDA.
    bus_recover();

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

imu_cal_status_t mpu6050_calibrate_gyro(int samples)
{
    gyro_cal_t cal;
    gyro_cal_init(&cal);
    float bias[3] = {gyro_bias_dps[0], gyro_bias_dps[1], gyro_bias_dps[2]};
    gyro_bias_dps[0] = gyro_bias_dps[1] = gyro_bias_dps[2] = 0.0f;  // read raw

    for (int i = 0; i < samples; i++) {
        mpu6050_sample_t s;
        // Calibration takes ~1 s; keep a possibly-running watchdog fed
        // (harmless no-op when the watchdog hasn't been enabled).
        watchdog_update();
        sleep_ms(2);
        if (mpu6050_read(&s)) {
            gyro_cal_add(&cal, s.accel_g, s.gyro_dps);
        }
    }
    // On anything but IMU_CAL_OK, bias still holds the previous one.
    const imu_cal_status_t st = gyro_cal_finish(&cal, samples, bias);
    for (int j = 0; j < 3; j++) {
        gyro_bias_dps[j] = bias[j];
    }
    return st;
}

bool mpu6050_data_ready(void)
{
    return gpio_get(PIN_IMU_INT);
}
