/**
 * @file pca9685_phase.c
 * @brief Phase-staggered encoding of the PCA9685's per-channel LEDn registers.
 *
 * See pca9685_phase.h for why this exists. No ESP-IDF dependency on purpose —
 * test/test_pca9685_phase.c compiles this file directly.
 */

#include "pca9685_phase.h"

#include <stddef.h>

uint16_t pca9685_phase_on(uint8_t ch)
{
    return (uint16_t)(((uint32_t)(ch % PCA9685_CHANNELS) * PCA9685_PERIOD_COUNTS /
                       PCA9685_CHANNELS) &
                      0x0FFFu);
}

void pca9685_encode(uint8_t ch, uint16_t value, uint8_t *buf)
{
    if (buf == NULL) {
        return;
    }

    if (value == 0u) { /* full off */
        buf[0] = 0u;
        buf[1] = 0u;
        buf[2] = 0u;
        buf[3] = PCA9685_BIT_FULL;
        return;
    }

    if (value >= PCA9685_PERIOD_COUNTS) { /* full on */
        buf[0] = 0u;
        buf[1] = PCA9685_BIT_FULL;
        buf[2] = 0u;
        buf[3] = 0u;
        return;
    }

    const uint16_t on = pca9685_phase_on(ch);
    /* The counter wraps within the period, so an ON late in the period simply
     * puts the falling edge early in the next one — the chip handles OFF < ON
     * natively. Masking to 12 bits is the whole of it, and it is what keeps
     * the ON-time equal to `value` for every channel regardless of phase. */
    const uint16_t off = (uint16_t)((on + value) & 0x0FFFu);

    buf[0] = (uint8_t)(on & 0xFFu);
    buf[1] = (uint8_t)(on >> 8);
    buf[2] = (uint8_t)(off & 0xFFu);
    buf[3] = (uint8_t)(off >> 8);
}
