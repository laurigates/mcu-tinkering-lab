// Complementary filter: fuses accelerometer tilt with integrated gyro rate
// into a pitch estimate. Pure C (no SDK dependencies) for host unit tests.
#ifndef BALANCEBOT_IMU_FILTER_H
#define BALANCEBOT_IMU_FILTER_H

#include <stdbool.h>
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

// Gravity check before arming. A sample counts as still when the accel
// magnitude is within IMU_STILL_ACCEL_TOL_G of 1 g (no linear acceleration)
// and the gyro magnitude is below IMU_STILL_GYRO_DPS (not being turned): only
// then is the accel vector gravity and its direction worth trusting. The
// mounted body Z below IMU_UPSIDE_DOWN_G means the board reads more than 120
// degrees from upright, which at rest is a wrong `mount` setting rather than a
// lean the controller could recover from.
#define IMU_STILL_ACCEL_TOL_G 0.15f
#define IMU_STILL_GYRO_DPS 10.0f
#define IMU_UPSIDE_DOWN_G (-0.5f)

typedef struct {
    bool still;        // accel reads gravity alone and the gyro is quiet
    bool upside_down;  // body Z (mounted) below IMU_UPSIDE_DOWN_G
    float up_g;        // body Z (mounted), for the refusal message
} imu_gravity_check_t;

// Classify one raw sample (sensor axes X, Y, Z: accel in g, gyro in deg/s)
// for `mount`. Pure, so the host tests can drive it.
imu_gravity_check_t imu_gravity_check(uint32_t mount, const float accel_g[3],
                                      const float gyro_dps[3]);

// Arming is refused only on a still sample that reads upside down; a moving
// one says nothing about the mounting, so the existing arming rules decide.
bool imu_gravity_refuses_arm(const imu_gravity_check_t *g);

// The arm decision itself: classify a raw sample under `mount` and apply
// imu_gravity_refuses_arm(). Core 1 calls it with the mount in effect when it
// handles CMD_ARM, so a `mount` change drained earlier in the same tick
// counts; a check computed on an earlier tick would still carry the old mount.
bool imu_arm_refused(uint32_t mount, const float accel_g[3], const float gyro_dps[3]);

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
