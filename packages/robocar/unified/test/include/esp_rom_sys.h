/**
 * @file esp_rom_sys.h — host-test shim.
 *
 * i2c_bus.c holds the TCA9548A's RESET low with esp_rom_delay_us(). The test
 * that compiles it records the requested duration, so the pulse width is
 * asserted rather than slept.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_ROM_SYS_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_ROM_SYS_H

#include <stdint.h>

void esp_rom_delay_us(uint32_t us);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_ROM_SYS_H */
