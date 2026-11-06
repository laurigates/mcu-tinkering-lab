#include "imu_filter.h"

#include <math.h>

#define RAD_TO_DEG 57.29577951308232f

void imu_filter_init(imu_filter_t *f, float alpha)
{
    f->alpha = alpha;
    imu_filter_reset(f);
}

void imu_filter_reset(imu_filter_t *f)
{
    f->pitch_deg = 0.0f;
    f->initialized = 0;
}

imu_body_accel_t imu_mount_accel(uint32_t mount, float accel_x_g, float accel_z_g)
{
    // A 180 degree turn about Y negates X and Z together.
    const float sign = (mount == IMU_MOUNT_DOWN) ? -1.0f : 1.0f;
    return (imu_body_accel_t){.forward_g = sign * accel_x_g, .up_g = sign * accel_z_g};
}

float imu_filter_accel_pitch_deg(float accel_forward_g, float accel_up_g)
{
    return atan2f(accel_forward_g, accel_up_g) * RAD_TO_DEG;
}

float imu_filter_update(imu_filter_t *f, float accel_forward_g, float accel_up_g,
                        float gyro_rate_dps, float dt)
{
    float accel_pitch = imu_filter_accel_pitch_deg(accel_forward_g, accel_up_g);
    if (!f->initialized) {
        f->pitch_deg = accel_pitch;
        f->initialized = 1;
        return f->pitch_deg;
    }
    float gyro_pitch = f->pitch_deg + gyro_rate_dps * dt;
    f->pitch_deg = f->alpha * gyro_pitch + (1.0f - f->alpha) * accel_pitch;
    return f->pitch_deg;
}
