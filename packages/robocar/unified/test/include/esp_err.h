/**
 * @file esp_err.h — host-test shim.
 *
 * motor_controller.h (and a few other target headers) unconditionally include
 * "esp_err.h" from ESP-IDF. This file is only visible on the test include
 * path; it provides just enough of the real API to let those headers and
 * their implementations compile on the host.
 *
 * Companion host-test guards (GOAL_STATE_HOST_TEST, ULTRASONIC_HOST_TEST,
 * REACTIVE_CONTROLLER_HOST_TEST) also typedef esp_err_t under their `#else`
 * branches. Use a single include-guard macro here so both entry points remain
 * compatible: whichever file is first wins, and the other is a no-op.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_ERR_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_ERR_H

#include <stdio.h>
#include <stdlib.h>

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_ARG (-2)
#define ESP_ERR_INVALID_STATE (-3)
#define ESP_ERR_NOT_FOUND (-4)
#define ESP_ERR_NO_MEM (-5)
#define ESP_ERR_TIMEOUT (-6)

/** Implemented in host_shims.c. audio_player.c logs it on an I2S write error;
 *  the ESP_LOG* shims discard the string, but it still has to link. */
const char *esp_err_to_name(esp_err_t err);

/** Aborts on the host rather than silently swallowing, so a stub that starts
 *  failing fails the test run instead of the assertion below it. */
#define ESP_ERROR_CHECK(x)                                                                  \
    do {                                                                                    \
        const esp_err_t _esp_err_rc = (x);                                                  \
        if (_esp_err_rc != ESP_OK) {                                                        \
            fprintf(stderr, "ESP_ERROR_CHECK failed: %d at %s:%d\n", _esp_err_rc, __FILE__, \
                    __LINE__);                                                              \
            abort();                                                                        \
        }                                                                                   \
    } while (0)

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_ERR_H */
