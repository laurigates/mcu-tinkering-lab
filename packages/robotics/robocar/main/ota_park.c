/**
 * @file ota_park.c
 * @brief Find open floor before an OTA install. See ota_park.h.
 */

#include "ota_park.h"

#include <string.h>

static void enter(ota_park_t *p, ota_park_phase_t phase, uint32_t now_ms)
{
    p->phase = phase;
    p->phase_start_ms = now_ms;
}

static void finish(ota_park_t *p, ota_park_result_t r, uint32_t now_ms)
{
    p->result = r;
    enter(p, OTA_PARK_PHASE_DONE, now_ms);
}

void ota_park_start(ota_park_t *p, const ota_park_config_t *cfg, uint32_t now_ms)
{
    memset(p, 0, sizeof(*p));
    p->cfg = *cfg;
    enter(p, OTA_PARK_PHASE_SETTLE, now_ms);
}

ota_park_cmd_t ota_park_step(ota_park_t *p, uint32_t now_ms, uint16_t front_cm, bool front_ok,
                             uint16_t rear_cm, bool rear_ok)
{
    const uint32_t elapsed = now_ms - p->phase_start_ms; /* wrap-safe */

    switch (p->phase) {
        case OTA_PARK_PHASE_SETTLE:
            if (elapsed < p->cfg.settle_ms) {
                return OTA_PARK_CMD_STOP;
            }
            /* A failed sensor reads as max range: never take that as open. */
            if (!front_ok) {
                finish(p, OTA_PARK_RESULT_NO_SENSOR, now_ms);
                return OTA_PARK_CMD_STOP;
            }
            if (front_cm >= p->cfg.open_cm) {
                if (rear_ok && rear_cm < p->cfg.rear_min_cm) {
                    enter(p, OTA_PARK_PHASE_NUDGE, now_ms);
                    return OTA_PARK_CMD_FORWARD;
                }
                finish(p, OTA_PARK_RESULT_OPEN, now_ms);
                return OTA_PARK_CMD_STOP;
            }
            if (p->steps >= p->cfg.max_steps) {
                finish(p, OTA_PARK_RESULT_BEST_EFFORT, now_ms);
                return OTA_PARK_CMD_STOP;
            }
            p->steps++;
            enter(p, OTA_PARK_PHASE_ROTATE, now_ms);
            return OTA_PARK_CMD_ROTATE;

        case OTA_PARK_PHASE_ROTATE:
            if (elapsed < p->cfg.rotate_ms) {
                return OTA_PARK_CMD_ROTATE;
            }
            enter(p, OTA_PARK_PHASE_SETTLE, now_ms);
            return OTA_PARK_CMD_STOP;

        case OTA_PARK_PHASE_NUDGE:
            if (elapsed < p->cfg.nudge_ms) {
                return OTA_PARK_CMD_FORWARD;
            }
            finish(p, OTA_PARK_RESULT_OPEN, now_ms);
            return OTA_PARK_CMD_STOP;

        case OTA_PARK_PHASE_DONE:
        default:
            return OTA_PARK_CMD_STOP;
    }
}

ota_park_result_t ota_park_result(const ota_park_t *p)
{
    return p->result;
}

const char *ota_park_result_name(ota_park_result_t r)
{
    switch (r) {
        case OTA_PARK_RESULT_OPEN:
            return "open";
        case OTA_PARK_RESULT_BEST_EFFORT:
            return "best_effort";
        case OTA_PARK_RESULT_NO_SENSOR:
            return "no_sensor";
        case OTA_PARK_RESULT_NONE:
        default:
            return "parking";
    }
}
