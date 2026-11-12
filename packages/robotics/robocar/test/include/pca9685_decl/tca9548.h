/**
 * @file tca9548.h
 * @brief Host-test shim declaring only what i2c_bus.c uses from the
 *        esp-idf-lib TCA9548A driver.
 *
 * Same reason as the pca9685.h beside it: the driver is a managed component
 * now, so its real header only exists under managed_components/ once an
 * ESP-IDF build has run, and the host tests must configure in a fresh
 * checkout. A declaration stub, not a copy of the driver — the firmware build
 * uses the real header and is what catches any disagreement.
 */

#ifndef TCA9548_HOST_DECL_H
#define TCA9548_HOST_DECL_H

#include <i2cdev.h>
#include <stdint.h>

/** Channel 0 as a bitmask — the driver takes a mask, not an index. */
#define TCA9548_CHANNEL0 (1 << 0)

esp_err_t tca9548_init_desc(i2c_dev_t *dev, uint8_t addr, int port, int sda, int scl);
esp_err_t tca9548_free_desc(i2c_dev_t *dev);
esp_err_t tca9548_set_channels(i2c_dev_t *dev, uint8_t channels);

#endif  // TCA9548_HOST_DECL_H
