/**
 * @file test_motor_controller.c
 *
 * Pins the idle re-write suppression in motor_controller.c's set_motors().
 *
 * reactive_controller's 30 Hz loop calls motor_stop() unconditionally on every
 * iteration it is not driving, so a parked robot re-wrote six identical
 * PCA9685 registers 30 times a second — the firmware's only continuous I2C
 * traffic, and the reason the bus goes from silent to permanently busy the
 * moment the PCA9685 is fitted.
 *
 * What is worth pinning is not the suppression but its bounds, and none of
 * them can be staged on a bench: that a failed write is never remembered as
 * applied, that the suppression expires so a chip whose contents drifted is
 * re-asserted, and that the expiry survives the uint32 millisecond wrap at
 * day 49 rather than latching the robot into permanent suppression.
 *
 * motor_controller.c is compiled unmodified; the i2c_bus it writes through and
 * the clock it reads are the stubs at the top of this file.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c_bus.h"
#include "motor_controller.h"
#include "pin_config.h"

/* ---- injectable clock --------------------------------------------------- */

static int64_t s_now_us;

int64_t esp_timer_get_time(void)
{
    return s_now_us;
}

static void advance_ms(uint32_t ms)
{
    s_now_us += (int64_t)ms * 1000;
}

/* ---- recording i2c_bus stub --------------------------------------------- */

#define MAX_WRITES 64

static struct {
    uint8_t first_ch;
    uint8_t count;
    uint16_t values[8];
} s_writes[MAX_WRITES];
static int s_write_count;
static esp_err_t s_next_result = ESP_OK;

/* Values in a write are indexed by CHANNEL, mirroring set_motors(). Indexing
 * them positionally would quietly test the wrong pin the next time the block is
 * renumbered to follow the motor driver's header. */
#define SLOT(ch) ((ch) - MOTOR_FIRST_CHANNEL)

esp_err_t i2c_bus_pca9685_set_multi(uint8_t first_ch, uint8_t count, const uint16_t *values)
{
    if (s_write_count < MAX_WRITES) {
        s_writes[s_write_count].first_ch = first_ch;
        s_writes[s_write_count].count = count;
        memcpy(s_writes[s_write_count].values, values, (count > 8 ? 8 : count) * sizeof(uint16_t));
    }
    s_write_count++;
    return s_next_result;
}

/* ---- GPIO stubs (TB6612FNG STBY) ---------------------------------------- */

static int s_stby_level = -1;

esp_err_t gpio_config(const gpio_config_t *cfg)
{
    (void)cfg;
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level)
{
    if (pin == MOTOR_STBY_PIN) {
        s_stby_level = (int)level;
    }
    return ESP_OK;
}

/* ---- harness ------------------------------------------------------------ */

static int s_failures;

#define CHECK(cond, ...)                                           \
    do {                                                           \
        if (!(cond)) {                                             \
            s_failures++;                                          \
            fprintf(stderr, "  FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                          \
            fputc('\n', stderr);                                   \
        }                                                          \
    } while (0)

static void reset_bus(void)
{
    s_write_count = 0;
    s_next_result = ESP_OK;
}

/** The executor's tick. */
#define TICK_MS 33U

/** Largest rise the ramp allows on one call, at the capped interval. */
#define RAMP_STEP_CAP \
    ((uint16_t)(PCA9685_PWM_MAX * MOTOR_RAMP_DT_CAP_MS / MOTOR_RAMP_FULL_SCALE_MS))

static uint16_t speed_pwm(uint8_t speed)
{
    return (uint16_t)((uint32_t)speed * PCA9685_PWM_MAX / 255);
}

/** The last write the stub recorded. */
static const uint16_t *last_write(void)
{
    return s_writes[s_write_count - 1].values;
}

/** Re-issue a command once per tick, as the executor does, for long enough to
 *  finish any ramp. The ramp from 0 to full takes MOTOR_RAMP_FULL_SCALE_MS. */
#define SETTLE_TICKS (MOTOR_RAMP_FULL_SCALE_MS / TICK_MS + 4)
#define DRIVE_FOR_TICKS(n, call)                \
    do {                                        \
        for (int i_ = 0; i_ < (int)(n); i_++) { \
            (void)(call);                       \
            advance_ms(TICK_MS);                \
        }                                       \
    } while (0)

/* The module keeps its state in file statics with no reset hook, so the tests
 * run in one process in a fixed order and each leaves the motors stopped. */

static void test_init_writes_the_first_stop(void)
{
    reset_bus();

    CHECK(motor_controller_init() == ESP_OK, "init should succeed");
    CHECK(s_stby_level == 1, "init must take the TB6612FNG out of standby, got %d", s_stby_level);
    CHECK(s_write_count == 1, "init should issue exactly one write, got %d", s_write_count);
    CHECK(s_writes[0].first_ch == MOTOR_FIRST_CHANNEL && s_writes[0].count == 6,
          "init should write 6 channels from %d, got %d from %d", MOTOR_FIRST_CHANNEL,
          s_writes[0].count, s_writes[0].first_ch);
}

static void test_repeated_stop_does_not_touch_the_bus(void)
{
    reset_bus();

    /* What the 30 Hz idle loop actually does: one second of it, minus the tick
     * that would fall due for a refresh. */
    for (int i = 0; i < 30; i++) {
        CHECK(motor_stop() == ESP_OK, "suppressed stop must still report success");
        advance_ms(33);
    }

    CHECK(s_write_count == 0, "an unchanged stop must not reach the bus, got %d writes",
          s_write_count);
}

static void test_suppression_expires(void)
{
    reset_bus();
    advance_ms(MOTOR_REFRESH_INTERVAL_MS);

    CHECK(motor_stop() == ESP_OK, "refresh stop should succeed");
    CHECK(s_write_count == 1, "an expired cache must be re-asserted, got %d writes", s_write_count);
}

static void test_a_changed_state_always_writes(void)
{
    reset_bus();

    CHECK(motor_move_forward(200) == ESP_OK, "forward should succeed");
    CHECK(s_write_count == 1, "a changed state must write, got %d", s_write_count);
    CHECK(s_writes[0].values[SLOT(MOTOR_RIGHT_PWM_CHANNEL)] != 0,
          "forward should carry a non-zero PWM");

    /* No clock advance: the guard must key on the values, not only on time. */
    CHECK(motor_stop() == ESP_OK, "stop should succeed");
    CHECK(s_write_count == 2, "stopping from forward must write, got %d", s_write_count);
    CHECK(s_writes[1].values[SLOT(MOTOR_RIGHT_PWM_CHANNEL)] == 0, "stop should carry a zero PWM");
}

static void test_a_failed_write_is_not_remembered_as_applied(void)
{
    reset_bus();
    s_next_result = ESP_FAIL;

    CHECK(motor_move_forward(200) == ESP_FAIL, "a failed bus write must be reported");
    CHECK(s_write_count == 1, "the failing attempt should reach the bus, got %d", s_write_count);

    /* Same values, same instant. A cache that recorded the failed write would
     * suppress this and report success for a state the chip never received. */
    s_next_result = ESP_OK;
    CHECK(motor_move_forward(200) == ESP_OK, "the retry should succeed");
    CHECK(s_write_count == 2, "a failed write must not suppress the retry, got %d", s_write_count);

    reset_bus();
    motor_stop();
}

static void test_refresh_survives_the_uint32_millisecond_wrap(void)
{
    reset_bus();

    /* Park the clock just under the uint32 ms wrap and write there, so the
     * cache's timestamp is near UINT32_MAX. Brake, not a drive command: a
     * drive ramps over several writes, and this test is about suppressing a
     * state that has stopped changing. */
    s_now_us = ((int64_t)0xFFFFFF00u) * 1000;
    motor_brake();
    reset_bus();

    /* Cross the wrap: now_ms() is small, written_ms is huge. A signed or
     * naively-compared elapsed time reads as enormously negative here and
     * either refreshes on every call or never refreshes again. */
    advance_ms(100);
    CHECK(motor_brake() == ESP_OK, "unchanged command should succeed");
    CHECK(s_write_count == 0, "100 ms after the write, across the wrap, must still suppress: %d",
          s_write_count);

    advance_ms(MOTOR_REFRESH_INTERVAL_MS);
    CHECK(motor_brake() == ESP_OK, "refresh should succeed");
    CHECK(s_write_count == 1, "the refresh must fall due across the wrap, got %d", s_write_count);

    reset_bus();
    s_now_us = 0;
    motor_stop();
}

/* The bench cannot stage this one: a mis-slotted value still writes six
 * consecutive channels and still looks like a working transaction on a scope.
 * It shows up as a wheel that spins when it was told to stop, by which point
 * the suspicion is on the wiring rather than on the array literal. */
static void test_each_value_lands_on_its_own_channel(void)
{
    reset_bus();

    /* Distinguishable speeds, and opposite directions so the four direction
     * slots cannot all hold the same sentinel and pass by luck. */
    const uint8_t left_speed = 60;
    const uint8_t right_speed = 240;
    CHECK(motor_set_individual(left_speed, right_speed, 1, 0) == ESP_OK,
          "individual drive should succeed");
    CHECK(s_write_count == 1, "a changed state must write, got %d", s_write_count);

    /* Hold the command until the ramp has delivered both targets, then read
     * the placement off the settled write. */
    DRIVE_FOR_TICKS(SETTLE_TICKS, motor_set_individual(left_speed, right_speed, 1, 0));

    const uint16_t *v = last_write();
    const uint16_t left_pwm = speed_pwm(left_speed);
    const uint16_t right_pwm = speed_pwm(right_speed);

    CHECK(v[SLOT(MOTOR_LEFT_PWM_CHANNEL)] == left_pwm,
          "left PWM belongs on ch%d: expected %u, got %u", MOTOR_LEFT_PWM_CHANNEL, left_pwm,
          v[SLOT(MOTOR_LEFT_PWM_CHANNEL)]);
    CHECK(v[SLOT(MOTOR_RIGHT_PWM_CHANNEL)] == right_pwm,
          "right PWM belongs on ch%d: expected %u, got %u", MOTOR_RIGHT_PWM_CHANNEL, right_pwm,
          v[SLOT(MOTOR_RIGHT_PWM_CHANNEL)]);

    /* forward = IN1 high, IN2 low; the right motor was asked for reverse. */
    CHECK(v[SLOT(MOTOR_LEFT_IN1_CHANNEL)] == PCA9685_FULL_ON &&
              v[SLOT(MOTOR_LEFT_IN2_CHANNEL)] == PCA9685_FULL_OFF,
          "left forward should be IN1 on ch%d high, IN2 on ch%d low", MOTOR_LEFT_IN1_CHANNEL,
          MOTOR_LEFT_IN2_CHANNEL);
    CHECK(v[SLOT(MOTOR_RIGHT_IN1_CHANNEL)] == PCA9685_FULL_OFF &&
              v[SLOT(MOTOR_RIGHT_IN2_CHANNEL)] == PCA9685_FULL_ON,
          "right reverse should be IN1 on ch%d low, IN2 on ch%d high", MOTOR_RIGHT_IN1_CHANNEL,
          MOTOR_RIGHT_IN2_CHANNEL);

    reset_bus();
    motor_stop();
}

/* Brake and coast are DIFFERENT electrical states, and the difference lives
 * entirely in the two direction channels. IN1 = IN2 = high shorts the windings
 * (short brake); IN1 = IN2 = low leaves the outputs high-impedance and the
 * robot rolls (Stop/coast). Both write PWM = 0, so a test that only checked
 * PWM would pass against either. See the truth table in
 * docs/wiring-card-motors.typ. */
static void test_brake_and_coast_differ_on_the_direction_channels(void)
{
    reset_bus();

    CHECK(motor_brake() == ESP_OK, "brake should succeed");
    CHECK(s_write_count == 1, "brake must reach the bus, got %d", s_write_count);

    const uint16_t *b = s_writes[0].values;
    CHECK(b[SLOT(MOTOR_LEFT_IN1_CHANNEL)] == PCA9685_FULL_ON &&
              b[SLOT(MOTOR_LEFT_IN2_CHANNEL)] == PCA9685_FULL_ON &&
              b[SLOT(MOTOR_RIGHT_IN1_CHANNEL)] == PCA9685_FULL_ON &&
              b[SLOT(MOTOR_RIGHT_IN2_CHANNEL)] == PCA9685_FULL_ON,
          "short brake needs all four direction channels HIGH");
    CHECK(b[SLOT(MOTOR_LEFT_PWM_CHANNEL)] == 0 && b[SLOT(MOTOR_RIGHT_PWM_CHANNEL)] == 0,
          "brake should write PWM 0 on both channels");

    reset_bus();

    CHECK(motor_stop() == ESP_OK, "stop should succeed");
    CHECK(s_write_count == 1, "brake -> stop is a state change and must write, got %d",
          s_write_count);

    const uint16_t *c = s_writes[0].values;
    CHECK(c[SLOT(MOTOR_LEFT_IN1_CHANNEL)] == PCA9685_FULL_OFF &&
              c[SLOT(MOTOR_LEFT_IN2_CHANNEL)] == PCA9685_FULL_OFF &&
              c[SLOT(MOTOR_RIGHT_IN1_CHANNEL)] == PCA9685_FULL_OFF &&
              c[SLOT(MOTOR_RIGHT_IN2_CHANNEL)] == PCA9685_FULL_OFF,
          "coast needs all four direction channels LOW");
}

/* ---- start-up ramp ---------------------------------------------------------
 *
 * Stepping a standing robot straight to full duty browned out the XIAO twice
 * on 2026-11-07 (BROWNOUT_RST on J15, both at a motor start). None of this can
 * be staged on a bench without a scope on the rail; what can be pinned is that
 * a rise is spread over MOTOR_RAMP_FULL_SCALE_MS and that nothing which slows
 * or stops the robot waits for it. */

/* Leave the motors coasting, with the bus record cleared, after a settle that
 * cannot be suppressed into skipping the stop. */
static void park(void)
{
    motor_stop();
    advance_ms(TICK_MS);
    reset_bus();
}

static void test_a_start_from_rest_ramps_up(void)
{
    park();
    /* Long idle: the step must still be capped, not credited 5 s of ramp. */
    advance_ms(5000);

    const uint16_t target = speed_pwm(255);
    CHECK(motor_move_forward(255) == ESP_OK, "forward should succeed");
    CHECK(s_write_count == 1, "the first call must write, got %d", s_write_count);

    const uint16_t first = s_writes[0].values[SLOT(MOTOR_RIGHT_PWM_CHANNEL)];
    CHECK(first > 0, "the first step must move the motor, got PWM 0");
    CHECK(first <= RAMP_STEP_CAP, "the first step from rest is capped at %u, got %u", RAMP_STEP_CAP,
          first);
    CHECK(s_writes[0].values[SLOT(MOTOR_RIGHT_IN1_CHANNEL)] == PCA9685_FULL_ON &&
              s_writes[0].values[SLOT(MOTOR_RIGHT_IN2_CHANNEL)] == PCA9685_FULL_OFF,
          "direction pins are not ramped: forward must be set on the first write");

    /* Then rise monotonically, both wheels together, and arrive. */
    uint16_t prev = first;
    int ticks_to_target = 0;
    for (int i = 1; i <= (int)SETTLE_TICKS; i++) {
        advance_ms(TICK_MS);
        motor_move_forward(255);
        const uint16_t r = last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)];
        const uint16_t l = last_write()[SLOT(MOTOR_LEFT_PWM_CHANNEL)];
        CHECK(r == l, "both wheels ramp together, got right %u left %u", r, l);
        CHECK(r >= prev, "the ramp must not fall back: %u after %u", r, prev);
        prev = r;
        if (r == target && ticks_to_target == 0) {
            ticks_to_target = i;
        }
    }
    CHECK(prev == target, "the ramp must reach the commanded %u, ended at %u", target, prev);
    /* 0 -> full in about MOTOR_RAMP_FULL_SCALE_MS: not instant, not sluggish. */
    CHECK((uint32_t)ticks_to_target * TICK_MS >= MOTOR_RAMP_FULL_SCALE_MS - 2 * TICK_MS,
          "reached full scale after %d ticks, faster than the %u ms ramp", ticks_to_target,
          MOTOR_RAMP_FULL_SCALE_MS);
    CHECK((uint32_t)ticks_to_target * TICK_MS <= MOTOR_RAMP_FULL_SCALE_MS + 2 * TICK_MS,
          "reached full scale after %d ticks, slower than the %u ms ramp", ticks_to_target,
          MOTOR_RAMP_FULL_SCALE_MS);
}

/* The obstacle reflex brakes from whatever the wheels are doing. Waiting on a
 * ramp there would carry the robot into what it is braking for. */
static void test_brake_and_stop_are_immediate(void)
{
    park();
    DRIVE_FOR_TICKS(SETTLE_TICKS, motor_move_forward(255));
    reset_bus();

    CHECK(motor_brake() == ESP_OK, "brake should succeed");
    CHECK(s_write_count == 1, "brake must write at once, got %d", s_write_count);
    CHECK(last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)] == 0 &&
              last_write()[SLOT(MOTOR_RIGHT_IN1_CHANNEL)] == PCA9685_FULL_ON &&
              last_write()[SLOT(MOTOR_RIGHT_IN2_CHANNEL)] == PCA9685_FULL_ON,
          "brake from full speed must short the windings on the first write");

    DRIVE_FOR_TICKS(SETTLE_TICKS, motor_move_forward(255));
    reset_bus();

    CHECK(motor_stop() == ESP_OK, "stop should succeed");
    CHECK(s_write_count == 1, "stop must write at once, got %d", s_write_count);
    CHECK(last_write()[SLOT(MOTOR_LEFT_PWM_CHANNEL)] == 0 &&
              last_write()[SLOT(MOTOR_LEFT_IN1_CHANNEL)] == PCA9685_FULL_OFF,
          "stop from full speed must coast on the first write");
    park();
}

static void test_slowing_down_is_immediate(void)
{
    park();
    DRIVE_FOR_TICKS(SETTLE_TICKS, motor_move_forward(255));
    reset_bus();

    CHECK(motor_move_forward(60) == ESP_OK, "slower forward should succeed");
    CHECK(last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)] == speed_pwm(60),
          "lowering duty is not ramped: expected %u, got %u", speed_pwm(60),
          last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)]);
    park();
}

/* A reversal drives the motor against its own rotation, which draws more than
 * a start from rest; it must not inherit the speed of the old direction. */
static void test_a_reversal_ramps_from_zero(void)
{
    park();
    DRIVE_FOR_TICKS(SETTLE_TICKS, motor_move_forward(255));
    reset_bus();

    CHECK(motor_move_backward(255) == ESP_OK, "backward should succeed");
    const uint16_t r = last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)];
    CHECK(r > 0 && r <= RAMP_STEP_CAP, "a reversal restarts the ramp: first step %u, cap %u", r,
          RAMP_STEP_CAP);
    CHECK(last_write()[SLOT(MOTOR_RIGHT_IN1_CHANNEL)] == PCA9685_FULL_OFF &&
              last_write()[SLOT(MOTOR_RIGHT_IN2_CHANNEL)] == PCA9685_FULL_ON,
          "the new direction is applied on the first write");
    park();
}

/* The captured brownout: `rotate` from a standstill, one wheel each way. Each
 * motor ramps on its own direction, so neither may start at full rotate duty. */
static void test_rotate_from_rest_ramps_both_wheels(void)
{
    park();

    CHECK(motor_rotate_cw(153) == ESP_OK, "rotate should succeed");
    const uint16_t r = last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)];
    const uint16_t l = last_write()[SLOT(MOTOR_LEFT_PWM_CHANNEL)];
    CHECK(r > 0 && r <= RAMP_STEP_CAP && l > 0 && l <= RAMP_STEP_CAP,
          "rotate from rest must ramp both wheels: right %u left %u, cap %u", r, l, RAMP_STEP_CAP);

    DRIVE_FOR_TICKS(SETTLE_TICKS, motor_rotate_cw(153));
    CHECK(last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)] == speed_pwm(153) &&
              last_write()[SLOT(MOTOR_LEFT_PWM_CHANNEL)] == speed_pwm(153),
          "rotate must arrive at its commanded duty on both wheels");
    park();
}

/* After a failed write the chip's PWM is unknown, so the ramp must assume the
 * conservative case (stopped) rather than the value it meant to write. */
static void test_a_failed_write_restarts_the_ramp(void)
{
    park();
    DRIVE_FOR_TICKS(SETTLE_TICKS, motor_move_forward(255));

    s_next_result = ESP_FAIL;
    advance_ms(TICK_MS);
    motor_move_forward(255);
    s_next_result = ESP_OK;
    reset_bus();

    advance_ms(TICK_MS);
    CHECK(motor_move_forward(255) == ESP_OK, "the retry should succeed");
    const uint16_t r = last_write()[SLOT(MOTOR_RIGHT_PWM_CHANNEL)];
    CHECK(r <= RAMP_STEP_CAP, "after a failed write the ramp restarts from 0: got %u, cap %u", r,
          RAMP_STEP_CAP);
    park();
}

int main(void)
{
    printf("test_motor_controller\n");

    test_init_writes_the_first_stop();
    test_repeated_stop_does_not_touch_the_bus();
    test_suppression_expires();
    test_a_changed_state_always_writes();
    test_a_failed_write_is_not_remembered_as_applied();
    test_refresh_survives_the_uint32_millisecond_wrap();
    test_each_value_lands_on_its_own_channel();
    test_brake_and_coast_differ_on_the_direction_channels();
    test_a_start_from_rest_ramps_up();
    test_brake_and_stop_are_immediate();
    test_slowing_down_is_immediate();
    test_a_reversal_ramps_from_zero();
    test_rotate_from_rest_ramps_both_wheels();
    test_a_failed_write_restarts_the_ramp();

    if (s_failures != 0) {
        printf("FAILED (%d)\n", s_failures);
        return EXIT_FAILURE;
    }
    printf("PASSED\n");
    return EXIT_SUCCESS;
}
