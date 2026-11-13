/**
 * @file twin_exec.c
 * @brief What sits under the firmware's executor in the twin (ADR-036 §1).
 *
 * Three pieces, each standing in for hardware-facing firmware the host build
 * leaves out, with the firmware lines it mirrors:
 *
 *   - ultrasonic.h over the real ultrasonic_core.c with a twin-owned bus, in
 *     place of main/ultrasonic.c (its target branch, ultrasonic.c:56-160);
 *   - i2c_bus_pca9685_set_multi() (i2c_bus.c:458) and the STBY GPIO calls,
 *     under the real motor_controller.c;
 *   - the bring-up order of main.c init_hardware() and init_hierarchical_ai()
 *     (main.c:2242-2245, :2396), and one executor tick with the state the
 *     trace reports.
 */

#include "twin_exec.h"

#include <stddef.h>
#include <string.h>

#include "cliff_sensor.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "i2c_bus.h"
#include "motor_controller.h"
#include "pin_config.h"
#include "reactive_controller.h"
#include "servo_controller.h"

/** test/servo_controller_stub.c: the servo_controller.h subset the executor
 *  uses. The stub has no header; this is its reset. */
void servo_stub_reset(bool initialized);

/* ========================================================================== */
/* Rangefinders: ultrasonic.h over ultrasonic_core.c                          */
/* ========================================================================== */

/** What is in front of each sensor, in cm, or TWIN_RANGE_FAILED. Failed until
 *  the first twin_exec_set_range(): a trigger latched before the world has
 *  said anything is a read that fails, paced like any other (ADR-036 §1
 *  "Rangefinders"). */
static int s_world_cm[ULTRASONIC_COUNT] = {
    [ULTRASONIC_FRONT] = TWIN_RANGE_FAILED,
    [ULTRASONIC_REAR] = TWIN_RANGE_FAILED,
};
/** The distance each sensor's last trigger ranged: the RCWL-1601 measures at
 *  the trigger and the firmware reads the result ULTRASONIC_CONVERSION_MS
 *  later, so a change after the trigger is not in that reading. */
static int s_latched_cm[ULTRASONIC_COUNT] = {
    [ULTRASONIC_FRONT] = TWIN_RANGE_FAILED,
    [ULTRASONIC_REAR] = TWIN_RANGE_FAILED,
};
static ultrasonic_core_t s_core[ULTRASONIC_COUNT];
static bool s_ranger_init;

void twin_exec_set_range(ultrasonic_id_t sensor, int cm)
{
    if ((unsigned)sensor >= ULTRASONIC_COUNT) {
        return;
    }
    /* The bounds ultrasonic_core_decode() accepts. A value outside them cannot
     * come back from the sensor as a distance, so it is a failed read here
     * rather than three bytes the decoder would reject or misread. */
    const bool in_range = cm >= (int)ULTRASONIC_MIN_RANGE_CM && cm <= (int)ULTRASONIC_MAX_RANGE_CM;
    s_world_cm[sensor] = in_range ? cm : TWIN_RANGE_FAILED;
}

/** ultrasonic.c bus_trigger(): the 0x01 write. Always acknowledged; a failed
 *  measurement shows up at the read, so the cycle keeps its length. */
static bool bus_trigger(void *ctx, uint8_t channel)
{
    (void)ctx;
    s_latched_cm[channel] = s_world_cm[channel];
    return true;
}

/** ultrasonic.c bus_read(): three bytes, most significant first, the distance
 *  in micrometres. */
static bool bus_read(void *ctx, uint8_t channel, uint8_t raw[RCWL1601_READ_LEN])
{
    (void)ctx;
    if (s_latched_cm[channel] == TWIN_RANGE_FAILED) {
        return false;
    }
    const uint32_t um = (uint32_t)s_latched_cm[channel] * 10000u;
    raw[0] = (uint8_t)(um >> 16);
    raw[1] = (uint8_t)(um >> 8);
    raw[2] = (uint8_t)um;
    return true;
}

static const ultrasonic_bus_t k_bus = {
    .trigger = bus_trigger,
    .read = bus_read,
    .ctx = NULL,
};

esp_err_t ultrasonic_init(void)
{
    for (int i = 0; i < (int)ULTRASONIC_COUNT; i++) {
        ultrasonic_core_reset(&s_core[i]);
        s_latched_cm[i] = TWIN_RANGE_FAILED;
    }
    s_ranger_init = true;
    return ESP_OK;
}

/** The target ultrasonic_poll() (ultrasonic.c:144); the channel is the sensor
 *  id here, since nothing sits behind a mux. */
ultrasonic_result_t ultrasonic_poll(ultrasonic_id_t id, uint32_t now_ms, uint16_t *distance_cm)
{
    if (distance_cm == NULL) {
        return ULTRASONIC_ERROR;
    }
    if (!s_ranger_init || (unsigned)id >= ULTRASONIC_COUNT) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ULTRASONIC_ERROR;
    }
    return ultrasonic_core_step(&s_core[id], (uint8_t)id, now_ms, &k_bus, distance_cm);
}

uint8_t ultrasonic_channel(ultrasonic_id_t id)
{
    return (uint8_t)id;
}

const char *ultrasonic_name(ultrasonic_id_t id)
{
    return (id == ULTRASONIC_FRONT) ? "front" : "rear";
}

esp_err_t ultrasonic_deinit(void)
{
    s_ranger_init = false;
    return ESP_OK;
}

/* ========================================================================== */
/* Floor sensors                                                              */
/* ========================================================================== */

/** Whether the scenario fitted the floor sensors (config `cliff`). */
static bool s_cliff_fitted;

void twin_exec_set_floor(bool valid, bool front, bool rear)
{
    /* cliff_sensor.c's host branch returns this from every cliff_sensor_read(),
     * where the target reads the MCP23017 port. */
    cliff_sensor_test_set(s_cliff_fitted, valid, front, rear);
}

/* ========================================================================== */
/* PCA9685 and STBY, under motor_controller.c                                 */
/* ========================================================================== */

#define PCA9685_CHANNELS 16

/** The chip's channel registers as last written. motor_controller.c writes the
 *  six motor channels as one run from MOTOR_FIRST_CHANNEL; values are stored by
 *  absolute channel and read back by channel name. */
static uint16_t s_pca[PCA9685_CHANNELS];

esp_err_t i2c_bus_pca9685_set_multi(uint8_t first_ch, uint8_t count, const uint16_t *values)
{
    if (values == NULL || (unsigned)first_ch + count > PCA9685_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(&s_pca[first_ch], values, count * sizeof(values[0]));
    return ESP_OK;
}

/* motor_controller_init() configures and raises TB6612FNG STBY
 * (motor_controller.c:186-196). The twin has no standby state to model. */
esp_err_t gpio_config(const gpio_config_t *cfg)
{
    (void)cfg;
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level)
{
    (void)pin;
    (void)level;
    return ESP_OK;
}

/** One motor from its three channels, by the TB6612FNG truth table the driver
 *  writes (motor_controller.h motor_brake()): IN1 high / IN2 low is forward
 *  (motor_set_individual() direction 1), the reverse is reverse, both low is
 *  coast and both high is short brake. */
static twin_wheel_t decode_wheel(uint8_t in1_ch, uint8_t in2_ch, uint8_t pwm_ch)
{
    const bool in1 = s_pca[in1_ch] == PCA9685_FULL_ON;
    const bool in2 = s_pca[in2_ch] == PCA9685_FULL_ON;
    twin_wheel_t w = {.duty = s_pca[pwm_ch]};
    if (in1 && in2) {
        w.drive = "brake";
    } else if (in1) {
        w.drive = "forward";
    } else if (in2) {
        w.drive = "reverse";
    } else {
        w.drive = "coast";
    }
    return w;
}

void twin_exec_wheels(twin_wheel_t *left, twin_wheel_t *right)
{
    *left = decode_wheel(MOTOR_LEFT_IN1_CHANNEL, MOTOR_LEFT_IN2_CHANNEL, MOTOR_LEFT_PWM_CHANNEL);
    *right =
        decode_wheel(MOTOR_RIGHT_IN1_CHANNEL, MOTOR_RIGHT_IN2_CHANNEL, MOTOR_RIGHT_PWM_CHANNEL);
}

/* ========================================================================== */
/* Bring-up and the tick                                                      */
/* ========================================================================== */

static bool s_head;
static uint64_t s_ticks;

/* The state the last trace record reported. */
static twin_wheel_t s_last_left;
static twin_wheel_t s_last_right;
static bool s_head_traced;
static int s_last_pan;
static int s_last_tilt;
static const char *s_last_mode;
static bool s_last_settled;

static bool wheel_differs(const twin_wheel_t *a, const twin_wheel_t *b)
{
    return a->duty != b->duty || strcmp(a->drive, b->drive) != 0;
}

const char *twin_exec_init(uint32_t seed, bool head, bool cliff)
{
    s_head = head;
    s_cliff_fitted = cliff;
    s_ticks = 0;
    s_head_traced = false;

    /* main.c init_hardware(): cliff sensors, motors, servos (main.c:2242-2245).
     * A board without servos leaves servo_is_initialized() false, and the
     * executor reports the head as unavailable. */
    twin_exec_set_floor(false, false, false);
    if (motor_controller_init() != ESP_OK) {
        /* Every other motor_* call refuses with ESP_ERR_INVALID_STATE until
         * this succeeds and the executor ignores the refusal, so a twin that
         * carried on would trace a robot that never moves. */
        return "motor_controller_init";
    }
    servo_stub_reset(head);

    /* main.c init_hierarchical_ai(), after goal_state_init() (main.c:2396). */
    if (reactive_controller_init() != ESP_OK) {
        return "reactive_controller_init";
    }
    /* After init, which resets the generator. On the robot the gaze draws
     * from esp_random(). */
    reactive_controller_seed_gaze_for_test(seed);
    if (cliff) {
        /* The firmware arms at boot when CLIFF_REFLEX_AT_BOOT is set, and with
         * `cliff on` otherwise (main.c:1671). Armed, the state starts as "no
         * floor" under both sensors (cliff_reset()). */
        reactive_controller_set_cliff(true);
    }

    /* The reference for the first `wheels` record: both wheels coast at duty
     * 0, which motor_controller_init()'s motor_stop() wrote. Not traced. */
    twin_exec_wheels(&s_last_left, &s_last_right);
    return NULL;
}

void twin_exec_tick(bool voice_busy, twin_exec_report_t *out)
{
    memset(out, 0, sizeof(*out));

    /* reactive_controller.c head_face_forward() returns voice_turn_is_busy()
     * on the robot (reactive_controller.c:891); the host path reads this flag. */
    reactive_controller_set_face_forward_for_test(voice_busy);
    reactive_controller_tick_for_test();
    s_ticks++;

    twin_exec_wheels(&out->left, &out->right);
    out->wheels_changed =
        wheel_differs(&out->left, &s_last_left) || wheel_differs(&out->right, &s_last_right);
    s_last_left = out->left;
    s_last_right = out->right;

    if (!s_head) {
        return;
    }
    reactive_telemetry_t tele = {0};
    servo_position_t pos = {0};
    (void)reactive_controller_get_telemetry(&tele);
    (void)servo_get_position(&pos);
    out->pan_deg = pos.pan_angle;
    out->tilt_deg = pos.tilt_angle;
    out->mode = reactive_head_mode_name(tele.head_mode);
    out->settled = tele.head_settled;
    /* The first record is the state after tick 0, whatever it is. */
    out->head_changed = !s_head_traced || out->pan_deg != s_last_pan ||
                        out->tilt_deg != s_last_tilt || out->settled != s_last_settled ||
                        strcmp(out->mode, s_last_mode) != 0;
    s_head_traced = true;
    s_last_pan = out->pan_deg;
    s_last_tilt = out->tilt_deg;
    s_last_mode = out->mode;
    s_last_settled = out->settled;
}

uint64_t twin_exec_ticks(void)
{
    return s_ticks;
}
