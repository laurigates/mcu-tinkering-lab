// Complementary filter: fuses accelerometer tilt with integrated gyro rate
// into a pitch estimate. Pure C (no SDK dependencies) for host unit tests.
#ifndef BALANCEBOT_IMU_FILTER_H
#define BALANCEBOT_IMU_FILTER_H

typedef struct {
    float alpha;      // gyro weight, e.g. 0.98
    float pitch_deg;  // current estimate
    int initialized;  // first sample snaps to the accel angle
} imu_filter_t;

void imu_filter_init(imu_filter_t *f, float alpha);

// Reset so the next update snaps to the accelerometer angle.
void imu_filter_reset(imu_filter_t *f);

// Accelerometer-only tilt angle in degrees: atan2(forward, up).
float imu_filter_accel_pitch_deg(float accel_forward_g, float accel_up_g);

// One fusion step. gyro_rate_dps is the pitch rate (deg/s), dt in seconds.
// Returns the new pitch estimate in degrees.
float imu_filter_update(imu_filter_t *f, float accel_forward_g, float accel_up_g,
                        float gyro_rate_dps, float dt);

#endif  // BALANCEBOT_IMU_FILTER_H
