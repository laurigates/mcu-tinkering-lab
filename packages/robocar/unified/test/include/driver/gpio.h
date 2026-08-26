/**
 * @file driver/gpio.h — host-test shim.
 *
 * pin_config.h includes this unconditionally for the GPIO_NUM_* constants it
 * assigns to the pin macros. The host tests never touch a pin; they only need
 * the names to have values.
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

#endif /* ROBOCAR_UNIFIED_HOST_TEST_GPIO_H */
