/**
 * @file test_ultrasonic_stub.c
 * @brief Regression test for the ULTRASONIC_HOST_TEST stub.
 *
 * The stub is a single-variable latch: the value passed to
 * ultrasonic_test_set_distance() is returned by the next (and every
 * subsequent) ultrasonic_measure() call until overwritten. If this contract
 * breaks, every reactive_controller test that depends on it silently
 * misbehaves — hence this small guard.
 */

#include "ultrasonic.h"

#include <assert.h>
#include <stdio.h>

#define ASSERT(cond)                                               \
    do {                                                           \
        if (!(cond)) {                                             \
            printf("FAIL: %s:%d %s\n", __FILE__, __LINE__, #cond); \
            assert(cond);                                          \
        }                                                          \
    } while (0)

int main(void)
{
    printf("=== ultrasonic stub host tests ===\n\n");

    ASSERT(ultrasonic_init() == ESP_OK);

    /* Basic set→read */
    ultrasonic_test_set_distance(42);
    uint16_t out = 0;
    ASSERT(ultrasonic_measure(&out) == ESP_OK);
    ASSERT(out == 42);

    /* Overwrite */
    ultrasonic_test_set_distance(200);
    ASSERT(ultrasonic_measure(&out) == ESP_OK);
    ASSERT(out == 200);

    /* Latch persistence: second read returns same value */
    ASSERT(ultrasonic_measure(&out) == ESP_OK);
    ASSERT(out == 200);

    /* ULTRASONIC_DIST_ERROR simulates a timeout → ESP_FAIL */
    ultrasonic_test_set_distance(ULTRASONIC_DIST_ERROR);
    ASSERT(ultrasonic_measure(&out) == ESP_FAIL);
    ASSERT(out == ULTRASONIC_DIST_ERROR);

    /* NULL arg → ESP_ERR_INVALID_ARG */
    ASSERT(ultrasonic_measure(NULL) == ESP_ERR_INVALID_ARG);

    printf("\nAll ultrasonic stub tests passed.\n");
    return 0;
}
