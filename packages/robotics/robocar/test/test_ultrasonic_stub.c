/**
 * @file test_ultrasonic_stub.c
 * @brief Regression test for the ULTRASONIC_HOST_TEST stub.
 *
 * The stub is one latch per sensor: the value passed to
 * ultrasonic_test_set_distance(id, …) is returned by every later
 * ultrasonic_poll(id, …) until overwritten, and never by the other sensor. If
 * this contract breaks, every reactive_controller test that depends on it
 * silently misbehaves — hence this small guard.
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

    uint16_t out = 0;

    /* Before init every poll is an error. */
    ASSERT(ultrasonic_poll(ULTRASONIC_FRONT, 0, &out) == ULTRASONIC_ERROR);
    ASSERT(out == ULTRASONIC_DIST_ERROR);

    ASSERT(ultrasonic_init() == ESP_OK);

    /* Nothing injected yet: both sensors read as errors. */
    ASSERT(ultrasonic_poll(ULTRASONIC_FRONT, 0, &out) == ULTRASONIC_ERROR);
    ASSERT(ultrasonic_poll(ULTRASONIC_REAR, 0, &out) == ULTRASONIC_ERROR);

    /* Per-index set→read, and no leakage between the two. */
    ultrasonic_test_set_distance(ULTRASONIC_FRONT, 42);
    ultrasonic_test_set_distance(ULTRASONIC_REAR, 180);
    ASSERT(ultrasonic_poll(ULTRASONIC_FRONT, 0, &out) == ULTRASONIC_READY);
    ASSERT(out == 42);
    ASSERT(ultrasonic_poll(ULTRASONIC_REAR, 0, &out) == ULTRASONIC_READY);
    ASSERT(out == 180);

    /* Latch persistence. */
    ASSERT(ultrasonic_poll(ULTRASONIC_FRONT, 33, &out) == ULTRASONIC_READY);
    ASSERT(out == 42);

    /* ULTRASONIC_DIST_ERROR on one sensor leaves the other alone. */
    ultrasonic_test_set_distance(ULTRASONIC_REAR, ULTRASONIC_DIST_ERROR);
    ASSERT(ultrasonic_poll(ULTRASONIC_REAR, 0, &out) == ULTRASONIC_ERROR);
    ASSERT(out == ULTRASONIC_DIST_ERROR);
    ASSERT(ultrasonic_poll(ULTRASONIC_FRONT, 0, &out) == ULTRASONIC_READY);
    ASSERT(out == 42);

    /* Bad arguments. */
    ASSERT(ultrasonic_poll(ULTRASONIC_FRONT, 0, NULL) == ULTRASONIC_ERROR);
    ASSERT(ultrasonic_poll(ULTRASONIC_COUNT, 0, &out) == ULTRASONIC_ERROR);

    /* The channel table follows pin_config.h. */
    ASSERT(ultrasonic_channel(ULTRASONIC_FRONT) == 3);
    ASSERT(ultrasonic_channel(ULTRASONIC_REAR) == 4);

    printf("\nAll ultrasonic stub tests passed.\n");
    return 0;
}
