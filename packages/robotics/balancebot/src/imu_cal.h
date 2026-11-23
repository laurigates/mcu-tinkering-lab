// Calibration decisions that need the robot still: the gyro bias average and
// the pitch-trim capture. Pure C (no SDK dependencies) so the host tests can
// drive them; mpu6050.c feeds the gyro one on core 1, cli.c the trim one on
// core 0.
#ifndef BALANCEBOT_IMU_CAL_H
#define BALANCEBOT_IMU_CAL_H

#include <stdbool.h>
#include <stdint.h>

// A gyro calibration is rejected when any gyro axis spreads more than this
// peak to peak over the window, or the accel magnitude more than
// GYRO_CAL_MAX_ACCEL_SPREAD_G. The spread, not the magnitude, is judged: the
// raw gyro carries the very bias being measured (MPU6050 parts reach tens of
// deg/s). Sensor noise at the configured 98 Hz bandwidth is well below both;
// a hand holding the robot is not.
#define GYRO_CAL_MAX_SPREAD_DPS 3.0f
#define GYRO_CAL_MAX_ACCEL_SPREAD_G 0.1f

typedef enum {
    IMU_CAL_PENDING = 0,  // not run yet
    IMU_CAL_OK = 1,       // still for the whole window; result applied
    IMU_CAL_MOVED = 2,    // moved during the window; result discarded
    IMU_CAL_NO_DATA = 3,  // too few good samples; result discarded
    IMU_CAL_RANGE = 4,    // trim only: the result is outside +/- PITCH_TRIM_MAX_DEG
} imu_cal_status_t;

typedef struct {
    int n;
    float sum[3];
    float gmin[3];
    float gmax[3];
    float amin;  // accel magnitude, g
    float amax;
} gyro_cal_t;

void gyro_cal_init(gyro_cal_t *c);
// One raw sample: accel in g, gyro in deg/s with no bias subtracted.
void gyro_cal_add(gyro_cal_t *c, const float accel_g[3], const float gyro_raw_dps[3]);
// Judge the window. `requested` is how many samples were attempted; fewer
// than half of them read counts as IMU_CAL_NO_DATA. On IMU_CAL_OK writes the
// mean gyro to bias_dps; otherwise leaves bias_dps untouched.
imu_cal_status_t gyro_cal_finish(const gyro_cal_t *c, int requested, float bias_dps[3]);

// Pitch-trim capture: average the published (trimmed) pitch while the robot
// is held still at its balance point.
typedef struct {
    int n;
    int attempted;
    float sum;
    bool moved;
} trim_capture_t;

void trim_capture_init(trim_capture_t *t);
// One snapshot: the published pitch (deg, already trimmed), whether the IMU
// read, and the gravity check's `still`. A sample that read but was not still
// spoils the capture; one that did not read is only skipped.
void trim_capture_add(trim_capture_t *t, float pitch_deg, bool imu_ok, bool still);
// Judge the capture against the trim that was in effect while it ran. On
// IMU_CAL_OK writes the new trim (current trim + mean residual pitch) to
// *new_trim; otherwise leaves it untouched. IMU_CAL_RANGE reports a result
// pitch_trim_valid() would refuse.
imu_cal_status_t trim_capture_finish(const trim_capture_t *t, float current_trim, float *new_trim);

// Short name for `stat` and CLI replies.
const char *imu_cal_status_name(uint32_t status);

#endif  // BALANCEBOT_IMU_CAL_H
