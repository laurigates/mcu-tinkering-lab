// MPU6050 register driver over I2C1. Configured for the balancebot control
// loop: 500 Hz data-ready interrupt, +/-500 dps gyro, +/-4 g accel, DLPF on.
// All functions run on core 1 (the control loop owns the I2C bus).
#ifndef BALANCEBOT_MPU6050_H
#define BALANCEBOT_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float accel_g[3];   // X, Y, Z in g
    float gyro_dps[3];  // X, Y, Z in deg/s, bias-corrected
    float temp_c;
} mpu6050_sample_t;

// Init the I2C bus + sensor. Returns false if WHO_AM_I doesn't answer 0x68.
bool mpu6050_init(void);

// Blocking burst read of accel + temp + gyro (14 bytes). Returns false on
// I2C error (sample left untouched).
bool mpu6050_read(mpu6050_sample_t *out);

// Average `samples` gyro readings (robot at rest) into the bias that
// mpu6050_read() subtracts. Takes samples/500 s.
void mpu6050_calibrate_gyro(int samples);

// True while the data-ready INT line is asserted.
bool mpu6050_data_ready(void);

#endif  // BALANCEBOT_MPU6050_H
