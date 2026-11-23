#include "drive.h"

#include <math.h>

#include "config.h"

static float clamp_sym(float v, float max)
{
    if (!isfinite(v)) {
        return 0.0f;
    }
    if (v > max) {
        return max;
    }
    if (v < -max) {
        return -max;
    }
    return v;
}

static float approach(float value, float target, float step)
{
    if (value < target) {
        return value + step < target ? value + step : target;
    }
    return value - step > target ? value - step : target;
}

void drive_reset(drive_t *d)
{
    d->speed_target_sps = 0.0f;
    d->turn_target_sps = 0.0f;
    d->speed_ref_sps = 0.0f;
    d->turn_ref_sps = 0.0f;
    d->remaining_s = 0.0f;
}

void drive_command(drive_t *d, float speed_sps, float turn_sps, float seconds, float speed_max,
                   float turn_max)
{
    d->speed_target_sps = clamp_sym(speed_sps, fabsf(speed_max));
    d->turn_target_sps = clamp_sym(turn_sps, fabsf(turn_max));
    float s = isfinite(seconds) ? seconds : 0.0f;
    d->remaining_s = s < 0.0f ? 0.0f : (s > DRIVE_MAX_S ? DRIVE_MAX_S : s);
}

void drive_step(drive_t *d, float dt, float speed_accel_sps2, float turn_accel_sps2)
{
    if (d->remaining_s > 0.0f) {
        d->remaining_s -= dt;
    }
    if (d->remaining_s <= 0.0f) {
        d->remaining_s = 0.0f;
        d->speed_target_sps = 0.0f;
        d->turn_target_sps = 0.0f;
    }
    d->speed_ref_sps = approach(d->speed_ref_sps, d->speed_target_sps, speed_accel_sps2 * dt);
    d->turn_ref_sps = approach(d->turn_ref_sps, d->turn_target_sps, turn_accel_sps2 * dt);
}

float drive_advance_ref(float x_ref_steps, float x_steps, float speed_ref_sps, float dt,
                        float max_lead_steps)
{
    float ref = x_ref_steps + speed_ref_sps * dt;
    float lead = fabsf(max_lead_steps);
    if (ref > x_steps + lead) {
        ref = x_steps + lead;
    } else if (ref < x_steps - lead) {
        ref = x_steps - lead;
    }
    return ref;
}

drive_wheels_t drive_mix(float mean_sps, float turn_sps, float max_rate)
{
    float headroom = fabsf(max_rate) - fabsf(mean_sps);
    if (headroom < 0.0f) {
        headroom = 0.0f;
    }
    float turn = clamp_sym(turn_sps, headroom);
    return (drive_wheels_t){.left_sps = mean_sps + turn, .right_sps = mean_sps - turn};
}
