/**
 * @file pca9685.h
 * @brief Host-test shim declaring only what i2c_bus.c and servo_controller.c
 *        use from the esp-idf-lib PCA9685 driver.
 *
 * Since the driver moved from a vendored tree to a managed component
 * (main/idf_component.yml), its real header only exists under
 * managed_components/, which is gitignored and absent until an ESP-IDF build
 * has run. The host tests must configure in a fresh checkout, so the two
 * targets that merely need the *declarations* get them from here.
 *
 * Deliberately NOT in test/include/pca9685_host/: that directory is on the
 * include path of test_pca9685_multi, which compiles the real driver and must
 * see the real header. A shim there would shadow it and the test would then be
 * exercising this file instead of the shipped code.
 *
 * This is a declaration stub, not a copy of the driver. If a signature here
 * ever disagrees with the real header, the firmware build — which uses the
 * real one — is what catches it.
 */

#ifndef PCA9685_HOST_DECL_H
#define PCA9685_HOST_DECL_H

#include <i2cdev.h>
#include <stdint.h>

/** Write to every channel at once (the ALL_LED registers). */
#define PCA9685_CHANNEL_ALL 16

esp_err_t pca9685_init_desc(i2c_dev_t *dev, uint8_t addr, int port, int sda, int scl);
esp_err_t pca9685_free_desc(i2c_dev_t *dev);
esp_err_t pca9685_init(i2c_dev_t *dev);
esp_err_t pca9685_set_pwm_frequency(i2c_dev_t *dev, uint16_t freq);
esp_err_t pca9685_set_pwm_value(i2c_dev_t *dev, uint8_t channel, uint16_t value);
esp_err_t pca9685_set_pwm_values(i2c_dev_t *dev, uint8_t first_ch, uint8_t channels,
                                 const uint16_t *values);

#endif  // PCA9685_HOST_DECL_H
