/**
 * @file i2cdev.h — host-test shim for the vendored esp-idf-lib pca9685 driver.
 *
 * Only visible on test_pca9685's include path (it is added ahead of the
 * vendored component so pca9685.h's `#include <i2cdev.h>` resolves here). It
 * carries just enough of i2cdev's vocabulary for pca9685.c to compile
 * UNMODIFIED; the transport functions are implemented by the test, which
 * records what the driver asked to write.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_I2CDEV_H
#define ROBOCAR_UNIFIED_HOST_TEST_I2CDEV_H

#include <stddef.h>
#include <stdint.h>
#include <esp_err.h>

typedef int i2c_port_t;
typedef int gpio_num_t;

/* Just the fields pca9685.c touches; the real struct carries the whole
 * legacy-driver i2c_config_t. */
typedef struct {
    i2c_port_t port;
    struct {
        gpio_num_t sda_io_num;
        gpio_num_t scl_io_num;
        struct {
            uint32_t clk_speed;
        } master;
    } cfg;
    uint8_t addr;
    void *mutex;
} i2c_dev_t;

esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev);
esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev);
esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev);
esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev);
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size);
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data,
                            size_t out_size);

#define I2C_DEV_TAKE_MUTEX(dev)                       \
    do {                                              \
        esp_err_t __ = i2c_dev_take_mutex(dev);       \
        if (__ != ESP_OK) return __;                  \
    } while (0)

#define I2C_DEV_GIVE_MUTEX(dev)                       \
    do {                                              \
        esp_err_t __ = i2c_dev_give_mutex(dev);       \
        if (__ != ESP_OK) return __;                  \
    } while (0)

#define I2C_DEV_CHECK(dev, X)                         \
    do {                                              \
        esp_err_t ___ = X;                            \
        if (___ != ESP_OK) {                          \
            I2C_DEV_GIVE_MUTEX(dev);                  \
            return ___;                               \
        }                                             \
    } while (0)

#define I2C_DEV_CHECK_LOGE(dev, X, msg, ...)          \
    do {                                              \
        esp_err_t ___ = X;                            \
        if (___ != ESP_OK) {                          \
            I2C_DEV_GIVE_MUTEX(dev);                  \
            return ___;                               \
        }                                             \
    } while (0)

#endif /* ROBOCAR_UNIFIED_HOST_TEST_I2CDEV_H */
