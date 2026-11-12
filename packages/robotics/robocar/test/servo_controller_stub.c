/**
 * @file servo_controller_stub.c
 * @brief Recording stub of the servo_controller.h subset reactive_controller.c uses.
 *
 * Mirrors the real driver's refusals — not initialised, output released, angle
 * outside the live limits — so a test that exercises the executor sees the same
 * failures the hardware path would. Every accepted write is counted, and any
 * attempt outside the limits is counted separately so a test can assert it
 * never happened.
 */

#include "servo_controller.h"

static struct {
    bool initialized;
    bool pan_enabled;
    bool tilt_enabled;
    int16_t pan, tilt;
    int16_t pan_min, pan_max, tilt_min, tilt_max;
    int writes;
    int out_of_limit_attempts;
    bool fail_writes;
} g;

void servo_stub_reset(bool initialized)
{
    g.initialized = initialized;
    g.pan_enabled = initialized;
    g.tilt_enabled = initialized;
    g.pan = 0;
    g.tilt = 0;
    g.pan_min = SERVO_PAN_LIMIT_MIN_DEFAULT;
    g.pan_max = SERVO_PAN_LIMIT_MAX_DEFAULT;
    g.tilt_min = SERVO_TILT_LIMIT_MIN_DEFAULT;
    g.tilt_max = SERVO_TILT_LIMIT_MAX_DEFAULT;
    g.writes = 0;
    g.out_of_limit_attempts = 0;
    g.fail_writes = false;
}

int servo_stub_writes(void)
{
    return g.writes;
}

int servo_stub_out_of_limit_attempts(void)
{
    return g.out_of_limit_attempts;
}

void servo_stub_set_enabled(bool pan, bool tilt)
{
    g.pan_enabled = pan;
    g.tilt_enabled = tilt;
}

void servo_stub_set_limits(int16_t pan_min, int16_t pan_max, int16_t tilt_min, int16_t tilt_max)
{
    g.pan_min = pan_min;
    g.pan_max = pan_max;
    g.tilt_min = tilt_min;
    g.tilt_max = tilt_max;
}

bool servo_is_initialized(void)
{
    return g.initialized;
}

bool servo_is_enabled(servo_id_t id)
{
    if (!g.initialized)
        return false;
    return (id == SERVO_PAN) ? g.pan_enabled : g.tilt_enabled;
}

bool servo_is_angle_valid(servo_id_t id, int16_t angle)
{
    if (id == SERVO_PAN)
        return angle >= g.pan_min && angle <= g.pan_max;
    return angle >= g.tilt_min && angle <= g.tilt_max;
}

esp_err_t servo_get_limits(servo_id_t id, int16_t *min_deg, int16_t *max_deg)
{
    if (!min_deg || !max_deg)
        return ESP_ERR_INVALID_ARG;
    *min_deg = (id == SERVO_PAN) ? g.pan_min : g.tilt_min;
    *max_deg = (id == SERVO_PAN) ? g.pan_max : g.tilt_max;
    return ESP_OK;
}

esp_err_t servo_get_position(servo_position_t *position)
{
    if (!g.initialized || !position)
        return ESP_ERR_INVALID_STATE;
    position->pan_angle = g.pan;
    position->tilt_angle = g.tilt;
    return ESP_OK;
}

esp_err_t servo_set_angle(servo_id_t id, int16_t angle)
{
    if (!g.initialized)
        return ESP_ERR_INVALID_STATE;
    if (!servo_is_angle_valid(id, angle)) {
        g.out_of_limit_attempts++;
        return ESP_ERR_INVALID_ARG;
    }
    if (!servo_is_enabled(id))
        return ESP_ERR_INVALID_STATE;
    if (g.fail_writes)
        return ESP_FAIL;
    g.writes++;
    if (id == SERVO_PAN)
        g.pan = angle;
    else
        g.tilt = angle;
    return ESP_OK;
}
