/**
 * @file ultrasonic_core.c
 * @brief RCWL-1601 two-phase I2C read state machine. See ultrasonic_core.h.
 */

#include "ultrasonic_core.h"

#include <stddef.h>

void ultrasonic_core_reset(ultrasonic_core_t *s)
{
    if (s == NULL) {
        return;
    }
    s->converting = false;
    s->ever_triggered = false;
    s->trigger_ms = 0u;
}

ultrasonic_result_t ultrasonic_core_decode(const uint8_t raw[RCWL1601_READ_LEN],
                                           uint16_t *distance_cm)
{
    if (raw == NULL || distance_cm == NULL) {
        return ULTRASONIC_ERROR;
    }
    const uint32_t um = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[2];
    /* Rounded to the nearest centimetre: 10 000 um per cm. */
    const uint32_t cm = (um + 5000u) / 10000u;
    if (um == 0u || cm < ULTRASONIC_MIN_RANGE_CM || cm > ULTRASONIC_MAX_RANGE_CM) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ULTRASONIC_ERROR;
    }
    *distance_cm = (uint16_t)cm;
    return ULTRASONIC_READY;
}

ultrasonic_result_t ultrasonic_core_step(ultrasonic_core_t *s, uint8_t channel, uint32_t now_ms,
                                         const ultrasonic_bus_t *bus, uint16_t *distance_cm)
{
    if (s == NULL || bus == NULL || distance_cm == NULL || bus->trigger == NULL ||
        bus->read == NULL) {
        return ULTRASONIC_ERROR;
    }

    /* Unsigned subtraction, so the interval survives the uint32 wrap. */
    const uint32_t since_trigger = now_ms - s->trigger_ms;

    if (s->converting) {
        if (since_trigger < ULTRASONIC_CONVERSION_MS) {
            return ULTRASONIC_PENDING;
        }
        s->converting = false;
        uint8_t raw[RCWL1601_READ_LEN] = {0};
        if (!bus->read(bus->ctx, channel, raw)) {
            *distance_cm = ULTRASONIC_DIST_ERROR;
            return ULTRASONIC_ERROR;
        }
        return ultrasonic_core_decode(raw, distance_cm);
    }

    /* Idle. Pace the next attempt from the last one, success or not. */
    if (s->ever_triggered && since_trigger < ULTRASONIC_CONVERSION_MS) {
        return ULTRASONIC_PENDING;
    }
    s->ever_triggered = true;
    s->trigger_ms = now_ms;
    if (!bus->trigger(bus->ctx, channel)) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ULTRASONIC_ERROR;
    }
    s->converting = true;
    return ULTRASONIC_PENDING;
}
