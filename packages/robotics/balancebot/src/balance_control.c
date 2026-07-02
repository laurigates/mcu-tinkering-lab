#include "balance_control.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "hardware/watchdog.h"
#include "imu_filter.h"
#include "mpu6050.h"
#include "param_store.h"
#include "pico/flash.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pid.h"
#include "stepper.h"

// ---- shared state ----------------------------------------------------------

static queue_t cmd_queue;
static volatile balance_snapshot_t snapshot;

static balance_params_t params;  // live copy, core1-owned after launch

// ---- core-1 private state --------------------------------------------------

static imu_filter_t filter;
static pid_t angle_pid;
static pid_t vel_pid;
static balance_state_t state = BAL_STATE_IDLE;
static float wheel_rate_sps;  // integrated PID output (signed, steps/s)
static float angle_setpoint;  // deg, 0 + velocity-loop lean offset
static float open_loop_sps[STEPPER_COUNT];
static uint32_t gesture_ticks;  // consecutive ticks satisfying arm/clear condition
static bool save_ok;
static uint32_t save_count;
static bool watchdog_started;

static void apply_gains(void)
{
    angle_pid.kp = params.angle_kp;
    angle_pid.ki = params.angle_ki;
    angle_pid.kd = params.angle_kd;
    vel_pid.kp = params.vel_kp;
    vel_pid.ki = params.vel_ki;
    filter.alpha = params.filter_alpha;
}

void balance_control_init(const balance_params_t *initial)
{
    params = *initial;
    queue_init(&cmd_queue, sizeof(balance_cmd_t), 16);
    memset((void *)&snapshot, 0, sizeof snapshot);
}

bool balance_control_send_cmd(const balance_cmd_t *cmd)
{
    return queue_try_add(&cmd_queue, cmd);
}

void balance_control_get_snapshot(balance_snapshot_t *out)
{
    uint32_t seq1;
    uint32_t seq2;
    do {
        seq1 = snapshot.seq;
        __compiler_memory_barrier();
        memcpy(out, (const void *)&snapshot, sizeof *out);
        __compiler_memory_barrier();
        seq2 = snapshot.seq;
    } while (seq1 != seq2 || (seq1 & 1u));
}

// ---- core-1 implementation -------------------------------------------------

static void publish(uint32_t tick, bool imu_ok, float pitch, float gyro, float loop_us)
{
    snapshot.seq++;  // odd: write in progress
    __compiler_memory_barrier();
    snapshot.tick = tick;
    snapshot.state = (uint8_t)state;
    snapshot.imu_ok = imu_ok;
    snapshot.save_ok = save_ok;
    snapshot.save_count = save_count;
    snapshot.pitch_deg = pitch;
    snapshot.gyro_dps = gyro;
    snapshot.rate_cmd_sps = wheel_rate_sps;
    snapshot.angle_setpoint = angle_setpoint;
    snapshot.loop_us = loop_us;
    __compiler_memory_barrier();
    snapshot.seq++;  // even: stable
}

static void enter_state(balance_state_t next)
{
    if (next == state) {
        return;
    }
    if (next == BAL_STATE_RUN) {
        pid_reset(&angle_pid);
        pid_reset(&vel_pid);
        wheel_rate_sps = 0.0f;
        angle_setpoint = 0.0f;
        stepper_enable(true);
        if (!watchdog_started) {
            // Enabled on first arm so bench work (flash saves, gyro cal)
            // isn't racing a 100 ms deadline. A stalled loop reboots the
            // chip; the external nENABLE pull-up keeps motors off through
            // the reboot.
            watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
            watchdog_started = true;
        }
    } else {
        stepper_enable(false);
        open_loop_sps[STEPPER_LEFT] = 0.0f;
        open_loop_sps[STEPPER_RIGHT] = 0.0f;
    }
    gesture_ticks = 0;
    state = next;
}

static void handle_command(const balance_cmd_t *cmd)
{
    switch ((command_id_t)cmd->cmd) {
        case CMD_SET_PARAM:
            switch ((param_id_t)cmd->param_id) {
                case PARAM_ANGLE_KP:
                    params.angle_kp = cmd->value;
                    break;
                case PARAM_ANGLE_KI:
                    params.angle_ki = cmd->value;
                    break;
                case PARAM_ANGLE_KD:
                    params.angle_kd = cmd->value;
                    break;
                case PARAM_VEL_KP:
                    params.vel_kp = cmd->value;
                    break;
                case PARAM_VEL_KI:
                    params.vel_ki = cmd->value;
                    break;
                case PARAM_FILTER_ALPHA:
                    params.filter_alpha = cmd->value;
                    break;
                case PARAM_MAX_RATE:
                    params.max_rate_sps = cmd->value;
                    break;
                default:
                    break;
            }
            apply_gains();
            break;
        case CMD_ARM:
            if (state == BAL_STATE_IDLE) {
                enter_state(BAL_STATE_RUN);
            }
            break;
        case CMD_DISARM:
            enter_state(BAL_STATE_IDLE);
            break;
        case CMD_CALIBRATE:
            if (state == BAL_STATE_IDLE) {
                mpu6050_calibrate_gyro(IMU_CAL_SAMPLES);
                imu_filter_reset(&filter);
            }
            break;
        case CMD_SAVE:
            if (state != BAL_STATE_RUN) {
                // Sector erase stalls this core ~45 ms; start with a full
                // watchdog budget (no-op if the watchdog isn't running yet).
                watchdog_update();
                save_ok = param_store_save(&params);
                save_count++;
            }
            break;
        case CMD_OPEN_LOOP_RATE:
            if (state == BAL_STATE_IDLE && cmd->param_id < STEPPER_COUNT) {
                open_loop_sps[cmd->param_id] = cmd->value;
                stepper_enable(open_loop_sps[STEPPER_LEFT] != 0.0f ||
                               open_loop_sps[STEPPER_RIGHT] != 0.0f);
            }
            break;
        default:
            break;
    }
}

// Wait for the IMU data-ready rising edge, with a deadline fallback. Edge
// detection (rather than level) keeps the loop paced even if the INT line
// is stuck high — e.g. a disconnected sensor floating at ~2.2 V via the
// RP2350-E9 leakage — in which case ticks degrade to the timeout rate.
static void wait_for_sample(void)
{
    static bool armed;  // saw INT low since the last serviced sample
    absolute_time_t deadline = make_timeout_time_us(CONTROL_TICK_TIMEOUT_US);
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        if (!mpu6050_data_ready()) {
            armed = true;  // INT is latched until read; low means "serviced"
        } else if (armed) {
            armed = false;
            return;
        }
        tight_loop_contents();
    }
}

void balance_control_core1_main(void)
{
    // The other core writes flash (via CMD_SAVE handled *here* — but core 0
    // may also be interrupted); register this core as flash-safe cooperative.
    flash_safe_execute_core_init();

    stepper_init();

    imu_filter_init(&filter, params.filter_alpha);
    pid_init(&angle_pid, params.angle_kp, params.angle_ki, params.angle_kd, ANGLE_INTEGRAL_MAX,
             ANGLE_PID_OUT_MAX);
    pid_init(&vel_pid, params.vel_kp, params.vel_ki, 0.0f, MAX_LEAN_OFFSET_DEG,
             MAX_LEAN_OFFSET_DEG);

    bool imu_present = mpu6050_init();
    if (imu_present) {
        mpu6050_calibrate_gyro(IMU_CAL_SAMPLES);
    }

    uint32_t tick = 0;
    const float dt = CONTROL_DT_S;

    while (true) {
        wait_for_sample();
        absolute_time_t t0 = get_absolute_time();

        balance_cmd_t cmd;
        while (queue_try_remove(&cmd_queue, &cmd)) {
            handle_command(&cmd);
        }

        mpu6050_sample_t s;
        bool imu_ok = imu_present && mpu6050_read(&s);

        float pitch = snapshot.pitch_deg;
        float gyro_rate = 0.0f;
        if (imu_ok) {
            // Mounting: X forward, Z up; pitch rotates about Y. The filter
            // runs in sensor convention; IMU_PITCH_SIGN maps it to "positive
            // = falling forward" once, here.
            float raw = imu_filter_update(&filter, s.accel_g[0], s.accel_g[2], s.gyro_dps[1], dt);
            pitch = IMU_PITCH_SIGN * raw;
            gyro_rate = IMU_PITCH_SIGN * s.gyro_dps[1];
        }

        switch (state) {
            case BAL_STATE_RUN:
                if (!imu_ok || fabsf(pitch) > TILT_CUTOFF_DEG) {
                    enter_state(BAL_STATE_FAULT);
                    break;
                }
                // Outer loop (100 Hz): lean against wheel velocity to hold
                // station. Positive wheel rate = drifting forward -> lean back.
                if (tick % VELOCITY_LOOP_DIV == 0) {
                    angle_setpoint =
                        pid_update(&vel_pid, 0.0f, wheel_rate_sps, NAN, dt * VELOCITY_LOOP_DIV);
                }
                // Inner loop: PID output is wheel acceleration (steps/s^2),
                // integrated into a step rate.
                {
                    float accel = pid_update(&angle_pid, angle_setpoint, pitch, gyro_rate, dt);
                    wheel_rate_sps += accel * dt;
                    if (wheel_rate_sps > params.max_rate_sps) {
                        wheel_rate_sps = params.max_rate_sps;
                    }
                    if (wheel_rate_sps < -params.max_rate_sps) {
                        wheel_rate_sps = -params.max_rate_sps;
                    }
                    stepper_set_rate(STEPPER_LEFT, wheel_rate_sps, params.max_rate_sps, dt);
                    stepper_set_rate(STEPPER_RIGHT, wheel_rate_sps, params.max_rate_sps, dt);
                }
                break;

            case BAL_STATE_IDLE:
                // Bench mode: open-loop rates if any, else motors stay off.
                if (stepper_is_enabled()) {
                    stepper_set_rate(STEPPER_LEFT, open_loop_sps[STEPPER_LEFT], params.max_rate_sps,
                                     dt);
                    stepper_set_rate(STEPPER_RIGHT, open_loop_sps[STEPPER_RIGHT],
                                     params.max_rate_sps, dt);
                }
                // Arm gesture: held upright for ARM_GESTURE_S.
                if (imu_ok && fabsf(pitch) < ARM_GESTURE_DEG && !stepper_is_enabled()) {
                    gesture_ticks++;
                    if (gesture_ticks >= (uint32_t)(ARM_GESTURE_S * CONTROL_LOOP_HZ)) {
                        enter_state(BAL_STATE_RUN);
                    }
                } else {
                    gesture_ticks = 0;
                }
                break;

            case BAL_STATE_FAULT:
                if (imu_ok && fabsf(pitch) < FAULT_CLEAR_DEG) {
                    gesture_ticks++;
                    if (gesture_ticks >= (uint32_t)(FAULT_CLEAR_S * CONTROL_LOOP_HZ)) {
                        enter_state(BAL_STATE_IDLE);
                    }
                } else {
                    gesture_ticks = 0;
                }
                break;
        }

        if (watchdog_started) {
            watchdog_update();
        }

        tick++;
        float loop_us = (float)absolute_time_diff_us(t0, get_absolute_time());
        publish(tick, imu_ok, pitch, gyro_rate, loop_us);
    }
}
