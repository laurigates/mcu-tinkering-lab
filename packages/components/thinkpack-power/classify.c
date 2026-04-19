/**
 * @file classify.c
 * @brief Pure-logic battery voltage classifier + adaptive beacon mapping.
 *
 * Runs on host builds as well as on-device. No ESP-IDF dependencies.
 */

#include "thinkpack_power.h"

thinkpack_power_battery_state_t thinkpack_power_classify(uint16_t mv)
{
    if (mv >= THINKPACK_POWER_OK_MV) {
        return THINKPACK_POWER_OK;
    }
    if (mv >= THINKPACK_POWER_LOW_MV) {
        return THINKPACK_POWER_LOW;
    }
    return THINKPACK_POWER_CRITICAL;
}

uint16_t thinkpack_power_adaptive_beacon_interval_ms(thinkpack_power_battery_state_t state)
{
    switch (state) {
        case THINKPACK_POWER_OK:
            return 500;
        case THINKPACK_POWER_LOW:
            return 2000;
        case THINKPACK_POWER_CRITICAL:
            return 5000;
        default:
            /* Fail-safe toward lower radio duty cycle on unknown states. */
            return 5000;
    }
}
