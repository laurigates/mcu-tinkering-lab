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

imu_gravity_check_t imu_gravity_check(uint32_t mount, const float accel_g[3],
                                      const float gyro_dps[3])
{
    const float a_mag =
        sqrtf(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2]);
    const float w_mag =
        sqrtf(gyro_dps[0] * gyro_dps[0] + gyro_dps[1] * gyro_dps[1] + gyro_dps[2] * gyro_dps[2]);
    const float up_g = imu_mount_accel(mount, accel_g[0], accel_g[2]).up_g;
    return (imu_gravity_check_t){
        .still = fabsf(a_mag - 1.0f) < IMU_STILL_ACCEL_TOL_G && w_mag < IMU_STILL_GYRO_DPS,
        .upside_down = up_g < IMU_UPSIDE_DOWN_G,
        .up_g = up_g,
    };
}

bool imu_gravity_refuses_arm(const imu_gravity_check_t *g)
{
    return g->still && g->upside_down;
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
