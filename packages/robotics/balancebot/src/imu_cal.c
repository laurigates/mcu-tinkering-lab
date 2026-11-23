#include "imu_cal.h"

#include <math.h>

#include "param_record.h"

void gyro_cal_init(gyro_cal_t *c)
{
    c->n = 0;
    for (int j = 0; j < 3; j++) {
        c->sum[j] = 0.0f;
        c->gmin[j] = INFINITY;
        c->gmax[j] = -INFINITY;
    }
    c->amin = INFINITY;
    c->amax = -INFINITY;
}

void gyro_cal_add(gyro_cal_t *c, const float accel_g[3], const float gyro_raw_dps[3])
{
    for (int j = 0; j < 3; j++) {
        c->sum[j] += gyro_raw_dps[j];
        c->gmin[j] = fminf(c->gmin[j], gyro_raw_dps[j]);
        c->gmax[j] = fmaxf(c->gmax[j], gyro_raw_dps[j]);
    }
    const float a =
        sqrtf(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2]);
    c->amin = fminf(c->amin, a);
    c->amax = fmaxf(c->amax, a);
    c->n++;
}

imu_cal_status_t gyro_cal_finish(const gyro_cal_t *c, int requested, float bias_dps[3])
{
    if (c->n == 0 || c->n <= requested / 2) {
        return IMU_CAL_NO_DATA;
    }
    for (int j = 0; j < 3; j++) {
        if (c->gmax[j] - c->gmin[j] > GYRO_CAL_MAX_SPREAD_DPS) {
            return IMU_CAL_MOVED;
        }
    }
    if (c->amax - c->amin > GYRO_CAL_MAX_ACCEL_SPREAD_G) {
        return IMU_CAL_MOVED;
    }
    for (int j = 0; j < 3; j++) {
        bias_dps[j] = c->sum[j] / (float)c->n;
    }
    return IMU_CAL_OK;
}

void trim_capture_init(trim_capture_t *t)
{
    t->n = 0;
    t->attempted = 0;
    t->sum = 0.0f;
    t->moved = false;
}

void trim_capture_add(trim_capture_t *t, float pitch_deg, bool imu_ok, bool still)
{
    t->attempted++;
    if (!imu_ok) {
        return;
    }
    if (!still) {
        t->moved = true;
        return;
    }
    t->sum += pitch_deg;
    t->n++;
}

imu_cal_status_t trim_capture_finish(const trim_capture_t *t, float current_trim, float *new_trim)
{
    if (t->moved) {
        return IMU_CAL_MOVED;
    }
    if (t->n == 0 || t->n <= t->attempted / 2) {
        return IMU_CAL_NO_DATA;
    }
    const float trim = current_trim + t->sum / (float)t->n;
    if (!pitch_trim_valid(trim)) {
        return IMU_CAL_RANGE;
    }
    *new_trim = trim;
    return IMU_CAL_OK;
}

const char *imu_cal_status_name(uint32_t status)
{
    switch ((imu_cal_status_t)status) {
        case IMU_CAL_PENDING:
            return "pending";
        case IMU_CAL_OK:
            return "ok";
        case IMU_CAL_MOVED:
            return "moved";
        case IMU_CAL_NO_DATA:
            return "no IMU data";
        case IMU_CAL_RANGE:
            return "out of range";
        default:
            return "unknown";
    }
}
