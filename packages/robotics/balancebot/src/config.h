// Balancebot compile-time configuration: loop rates, default gains, limits.
// Runtime-tunable parameters (PID gains, filter alpha, clamps) start from
// these defaults and can be overridden via the CLI and persisted to flash.
#ifndef BALANCEBOT_CONFIG_H
#define BALANCEBOT_CONFIG_H

// Control loop
#define CONTROL_LOOP_HZ 500
#define CONTROL_DT_S (1.0f / CONTROL_LOOP_HZ)
#define CONTROL_TICK_TIMEOUT_US 3000  // IMU INT fallback deadline (nominal tick is 2000 us)
#define VELOCITY_LOOP_DIV 5           // outer velocity loop at CONTROL_LOOP_HZ / 5 = 100 Hz

// IMU
#define IMU_I2C_BAUD 400000
#define IMU_CAL_SAMPLES 500     // gyro bias calibration, ~1 s at 500 Hz
#define IMU_PITCH_SIGN (+1.0f)  // flip if the robot drives into the fall

// Default filter / PID parameters (runtime-tunable)
#define DEFAULT_FILTER_ALPHA 0.98f
#define DEFAULT_ANGLE_KP 28.0f  // (steps/s^2) per degree
#define DEFAULT_ANGLE_KI 0.0f
#define DEFAULT_ANGLE_KD 1.2f
#define DEFAULT_VEL_KP 0.004f  // degrees of lean per (steps/s) of wheel velocity error
#define DEFAULT_VEL_KI 0.0005f
#define DEFAULT_MAX_RATE_SPS 4000.0f  // step-rate clamp (steps/s), 2.5 rev/s at 1/8 ustep

// Fixed limits (not runtime-tunable)
#define MAX_ACCEL_SPS2 40000.0f      // acceleration clamp (steps/s^2)
#define MIN_RATE_SPS 30.0f           // below this the PIO SM is parked
#define MAX_LEAN_OFFSET_DEG 3.0f     // velocity loop authority over the angle setpoint
#define ANGLE_PID_OUT_MAX 40000.0f   // angle PID output clamp (steps/s^2)
#define ANGLE_INTEGRAL_MAX 10000.0f  // anti-windup clamp

// Safety
#define TILT_CUTOFF_DEG 45.0f  // beyond this: motors off, FAULT
#define ARM_GESTURE_DEG 1.0f   // hold within +/- this ...
#define ARM_GESTURE_S 3.0f     // ... for this long to auto-arm from IDLE
#define FAULT_CLEAR_DEG 5.0f   // upright within this ...
#define FAULT_CLEAR_S 2.0f     // ... for this long clears FAULT to IDLE
#define WATCHDOG_TIMEOUT_MS 100

// Telemetry
#define TELEMETRY_DEFAULT_HZ 25
#define BATTERY_POLL_MS 1000

// Parameter store: last 4 KB sector of a 2 MB flash. The pico-sdk board
// header claims PICO_FLASH_SIZE_BYTES = 4 MB but Seeed specs the XIAO
// RP2350 at 2 MB — pin the sector at 2 MB - 4 KB, valid either way.
#define PARAM_FLASH_OFFSET ((2u * 1024u * 1024u) - 4096u)

#endif  // BALANCEBOT_CONFIG_H
