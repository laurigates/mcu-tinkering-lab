// MPU6050 register driver over I2C1. Configured for the balancebot control
// loop: 500 Hz data-ready interrupt, +/-500 dps gyro, +/-4 g accel, DLPF on.
// All functions run on core 1 (the control loop owns the I2C bus).
#ifndef BALANCEBOT_MPU6050_H
#define BALANCEBOT_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_cal.h"

typedef struct {
    float accel_g[3];   // X, Y, Z in g
    float gyro_dps[3];  // X, Y, Z in deg/s, bias-corrected
    float temp_c;
} mpu6050_sample_t;

// Init the I2C bus + sensor. Frees a bus the sensor is holding first (see
// mpu6050_bus_recoveries). Returns false if WHO_AM_I doesn't answer 0x68.
bool mpu6050_init(void);

// Burst read of accel + temp + gyro (14 bytes), each transfer bounded to
// about 1 ms. Returns false on I2C error or timeout (sample left untouched);
// a timeout also recovers the bus for the next read.
bool mpu6050_read(mpu6050_sample_t *out);

// Times a held I2C line was found and freed since boot (at init or after a
// timed-out transfer).
uint32_t mpu6050_bus_recoveries(void);

// Average `samples` gyro readings (robot at rest) into the bias that
// mpu6050_read() subtracts. Takes samples/500 s. Applies the new bias only
// on IMU_CAL_OK; if the robot moved (gyro_cal_finish) or too few samples
// read, the previous bias stays (zero before the first success).
imu_cal_status_t mpu6050_calibrate_gyro(int samples);

// True while the data-ready INT line is asserted.
bool mpu6050_data_ready(void);

#endif  // BALANCEBOT_MPU6050_H
