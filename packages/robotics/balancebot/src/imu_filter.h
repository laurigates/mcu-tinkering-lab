// Complementary filter: fuses accelerometer tilt with integrated gyro rate
// into a pitch estimate. Pure C (no SDK dependencies) for host unit tests.
#ifndef BALANCEBOT_IMU_FILTER_H
#define BALANCEBOT_IMU_FILTER_H

#include <stdint.h>

// How the GY-521 sits on the robot. Stored in balance_params_t as uint32_t.
//  UP:   component side up, X forward, Z up (the filter's convention).
//  DOWN: component side down, turned 180 degrees about Y (the axle axis), as
//        on the delivered 8d18cd8d carrier: X points back and Z down, so both
//        read inverted. Gyro Y is the rotation axis and keeps its sign.
// A sign flip on the filter output cannot absorb the turn: atan2(-x, -z) is
// 180 degrees away from atan2(x, z), not its negative.
typedef enum {
    IMU_MOUNT_UP = 0,
    IMU_MOUNT_DOWN = 1,
} imu_mount_t;

// Sensor accel X/Z mapped to the body's forward/up axes.
typedef struct {
    float forward_g;
    float up_g;
} imu_body_accel_t;

// Map the sensor's accel X and Z to body forward/up for `mount`. Any value
// other than IMU_MOUNT_DOWN maps as IMU_MOUNT_UP. Float only, no branches
// beyond one select, so core 1 can call it every tick.
imu_body_accel_t imu_mount_accel(uint32_t mount, float accel_x_g, float accel_z_g);

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
