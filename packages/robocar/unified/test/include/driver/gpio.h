/**
 * @file driver/gpio.h — host-test shim.
 *
 * pin_config.h includes this unconditionally for the GPIO_NUM_* constants it
 * assigns to the pin macros. Most host tests never touch a pin; they only need
 * the names to have values.
 *
 * motor_controller.c is the exception — it drives the TB6612FNG STBY line
 * directly — so the output-configuration subset it uses is declared here and
 * backed by recording stubs in test_motor_controller.c.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_GPIO_H
#define ROBOCAR_UNIFIED_HOST_TEST_GPIO_H

typedef int gpio_num_t;

#define GPIO_NUM_0 0
#define GPIO_NUM_1 1
#define GPIO_NUM_2 2
#define GPIO_NUM_3 3
#define GPIO_NUM_4 4
#define GPIO_NUM_5 5
#define GPIO_NUM_6 6
#define GPIO_NUM_7 7
#define GPIO_NUM_8 8
#define GPIO_NUM_9 9
#define GPIO_NUM_41 41
#define GPIO_NUM_42 42
#define GPIO_NUM_43 43
#define GPIO_NUM_44 44

/* Output-configuration subset used by motor_controller.c. Values are
 * arbitrary; nothing on the host interprets them. */
#include <stdint.h>

#include "esp_err.h"

#define GPIO_MODE_OUTPUT 2
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0

typedef struct {
    uint64_t pin_bit_mask;
    int mode;
    int pull_up_en;
    int pull_down_en;
    int intr_type;
} gpio_config_t;

esp_err_t gpio_config(const gpio_config_t *cfg);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_GPIO_H */
