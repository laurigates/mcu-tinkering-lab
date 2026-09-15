/**
 * @file test_servo_controller.c
 *
 * Pins servo_controller against the bus it drives.
 *
 * Init centres both servos via servo_set_angle(), and servo_set_angle()
 * refuses with ESP_ERR_INVALID_STATE until the module is initialised. The
 * flag used to be raised only AFTER centring, so init could never succeed:
 * it returned ESP_ERR_INVALID_STATE without a single bus write, and the
 * ESP_ERROR_CHECK around init_hardware() rebooted the board. A bare board
 * never reached the call (the hardware phase is skipped when I2C init
 * fails), which is why it survived until the PCA9685 was first fitted.
 *
 * The later cases pin the 2026-09 bench incident, where both servos were held
 * squealing against their end stops: a frequency change replayed counts from
 * the old period, the angle maths stretched tilt's ±45 deg across the servo's
 * full travel, and pan's +90 deg asked for 2500 us, past the SG90's 2400 us.
 *
 * servo_controller.c is compiled unmodified; the i2c_bus it writes through is
 * the recording stub at the bottom of this file.
 */

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"
#include "pin_config.h"
#include "servo_controller.h"

/* servo_move_smooth() and servo_exercise() sleep between steps. */
void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}

/* ---- recording i2c_bus stub --------------------------------------------- */

/* One ordered log of everything the module does to the chip, so a test can
 * assert not just what was written but in which order relative to a frequency
 * change. */
typedef enum { EV_WRITE, EV_FREQ } event_kind_t;

static struct {
    event_kind_t kind;
    uint8_t channel;
    uint16_t value; /* count for EV_WRITE, Hz for EV_FREQ */
} s_events[32];
static int s_event_count;
static esp_err_t s_next_result = ESP_OK;

/* The chip-wide prescaler the pulse maths reads. Settable so the tests can ask
 * what a given angle becomes at a frequency other than the shipped 200 Hz. */
static uint16_t s_pwm_hz = 200u;

static void record(event_kind_t kind, uint8_t channel, uint16_t value)
{
    if (s_event_count < (int)(sizeof(s_events) / sizeof(s_events[0]))) {
        s_events[s_event_count].kind = kind;
        s_events[s_event_count].channel = channel;
        s_events[s_event_count].value = value;
    }
    s_event_count++;
}

uint16_t i2c_bus_pca9685_frequency(void)
{
    return s_pwm_hz;
}

esp_err_t i2c_bus_pca9685_set_frequency(uint16_t hz)
{
    record(EV_FREQ, 0, hz);
    if (s_next_result == ESP_OK) {
        s_pwm_hz = hz;
    }
    return s_next_result;
}

esp_err_t i2c_bus_pca9685_set(uint8_t channel, uint16_t count)
{
    record(EV_WRITE, channel, count);
    return s_next_result;
}

static void reset_bus(void)
{
    s_event_count = 0;
    s_next_result = ESP_OK;
}

/* The expected count for a pulse, computed independently of the module. */
static uint16_t count_for_pulse(uint32_t pulse_us, uint16_t hz)
{
    const uint32_t count = (pulse_us * 4096u) / (1000000u / hz);
    return (uint16_t)(count > 4095u ? 4095u : count);
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

#define CHECK_WRITE(i, ch, count)                                                             \
    CHECK(s_events[i].kind == EV_WRITE && s_events[i].channel == (ch) &&                      \
              s_events[i].value == (count),                                                   \
          "event %d: expected write ch%u=%u, got kind=%d ch%u value=%u", (i), (unsigned)(ch), \
          (unsigned)(count), (int)s_events[i].kind, (unsigned)s_events[i].channel,            \
          (unsigned)s_events[i].value)

/* The module keeps its state in a file-static struct with no reset hook, so
 * the tests run in one process in a fixed order, each leaving the state the
 * next one documents. */

static void test_failed_centring_leaves_module_uninitialised(void)
{
    reset_bus();
    s_next_result = ESP_FAIL;

    esp_err_t err = servo_controller_init();

    CHECK(err == ESP_FAIL, "init returned %d, expected the bus error", err);
    CHECK(s_event_count == 1, "expected the first centring write, got %d", s_event_count);
    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_CENTER) == ESP_ERR_INVALID_STATE,
          "a failed init must not leave the servos reported as ready");
}

static void test_init_centres_both_servos(void)
{
    reset_bus();

    esp_err_t err = servo_controller_init();

    CHECK(err == ESP_OK, "init returned %d", err);
    CHECK(s_event_count == 2, "expected two centring writes, got %d", s_event_count);
    CHECK(s_events[0].channel == SERVO_PAN_CHANNEL, "first write to channel %u",
          s_events[0].channel);
    CHECK(s_events[1].channel == SERVO_TILT_CHANNEL, "second write to channel %u",
          s_events[1].channel);
    CHECK(s_events[0].value > 0 && s_events[1].value > 0, "centre pulse must be non-zero");

    int16_t pan = -1, tilt = -1;
    CHECK(servo_get_angle(SERVO_PAN, &pan) == ESP_OK && pan == SERVO_PAN_CENTER,
          "pan reads %d after init", pan);
    CHECK(servo_get_angle(SERVO_TILT, &tilt) == ESP_OK && tilt == SERVO_TILT_CENTER,
          "tilt reads %d after init", tilt);
}

static void test_set_angle_writes_after_init(void)
{
    reset_bus();

    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_CENTER + 10) == ESP_OK, "set_angle after init");
    CHECK(s_event_count == 1 && s_events[0].channel == SERVO_PAN_CHANNEL,
          "set_angle must write the pan channel once");
}

/* The centre pulse is 1500 us (SERVO_CENTER_PULSE_US), and a PCA9685 count is
 * that width as a fraction of the period over 4096 steps. So a centred servo's
 * count is entirely determined by the frequency:
 *
 *   200 Hz ->  5000 us period -> 1500 * 4096 /  5000 = 1228
 *    50 Hz -> 20000 us period -> 1500 * 4096 / 20000 =  307
 *
 * The maths used to divide by a hardcoded 5000, so both came out 1228 — and at
 * 50 Hz that count is a 6 ms pulse, several times longer than any servo range
 * accepts. Nothing caught it because the frequency could not be changed. */
static void test_the_count_tracks_the_pwm_frequency(void)
{
    s_pwm_hz = 200u;
    CHECK(servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER) == 1228u,
          "centre at 200 Hz should be count 1228, got %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER));

    s_pwm_hz = 50u;
    CHECK(servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER) == 307u,
          "centre at 50 Hz should be count 307, got %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER));

    /* Off-centre angles scale with it too, rather than staying put. */
    s_pwm_hz = 200u;
    const uint16_t at_200 = servo_angle_to_count(SERVO_PAN, 45);
    s_pwm_hz = 50u;
    const uint16_t at_50 = servo_angle_to_count(SERVO_PAN, 45);
    CHECK(at_50 < at_200, "a lower frequency must need a smaller count: %u vs %u", (unsigned)at_50,
          (unsigned)at_200);

    s_pwm_hz = 200u;
}

/* A pulse cannot outlast its period. At the driver's 1526 Hz ceiling even a
 * modest pulse is longer than the 655 us frame, and an unclamped count would
 * wrap in the 12-bit register and emerge as a SHORT pulse — a servo slamming to
 * the opposite endpoint rather than refusing. */
static void test_an_impossible_pulse_is_clamped(void)
{
    s_pwm_hz = 1526u;
    const uint16_t count = servo_angle_to_count(SERVO_TILT, 30);
    CHECK(count == 4095u, "an over-long pulse must clamp to 4095, got %u", (unsigned)count);
    s_pwm_hz = 200u;
}

/* An angle is a real servo angle: 1000 us per 90 deg about 1500 us, clamped to
 * the SG90's 500-2400 us. The old maths stretched each servo's configured range
 * across 500-2500 us, so tilt's "+45" was really +90 and pan's +90 was 2500 us. */
static void test_an_angle_is_a_real_servo_angle_within_the_sg90_range(void)
{
    s_pwm_hz = 200u;
    CHECK(servo_angle_to_count(SERVO_TILT, 45) == count_for_pulse(2000u, 200u),
          "tilt +45 must be a 2000 us pulse, got count %u",
          (unsigned)servo_angle_to_count(SERVO_TILT, 45));
    CHECK(servo_angle_to_count(SERVO_PAN, -90) == count_for_pulse(500u, 200u),
          "-90 must be the 500 us minimum, got count %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, -90));
    CHECK(servo_angle_to_count(SERVO_PAN, 90) == count_for_pulse(2400u, 200u),
          "+90 must clamp to the 2400 us maximum, got count %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, 90));
    CHECK(servo_angle_to_count(SERVO_PAN, -120) == count_for_pulse(500u, 200u),
          "an angle below the range must clamp to 500 us");

    int16_t hw_lo = 0, hw_hi = 0;
    servo_get_hw_range(&hw_lo, &hw_hi);
    CHECK(hw_lo == -90 && hw_hi == 81, "hardware range should be -90..+81, got %d..%d", hw_lo,
          hw_hi);
}

static void test_the_boot_limits_gate_every_move(void)
{
    reset_bus();

    int16_t lo = 0, hi = 0;
    CHECK(servo_get_limits(SERVO_PAN, &lo, &hi) == ESP_OK && lo == SERVO_PAN_LIMIT_MIN_DEFAULT &&
              hi == SERVO_PAN_LIMIT_MAX_DEFAULT,
          "pan boot limits %d..%d", lo, hi);

    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_LIMIT_MAX_DEFAULT + 1) == ESP_ERR_INVALID_ARG,
          "a pan angle past the limit must be refused");
    CHECK(servo_set_angle(SERVO_TILT, SERVO_TILT_LIMIT_MIN_DEFAULT - 1) == ESP_ERR_INVALID_ARG,
          "a tilt angle past the limit must be refused");
    CHECK(s_event_count == 0, "a refused move must not touch the bus, got %d events",
          s_event_count);

    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_LIMIT_MAX_DEFAULT) == ESP_OK,
          "the limit itself is allowed");
}

static void test_set_limits_rejects_what_the_hardware_or_centring_cannot_do(void)
{
    CHECK(servo_set_limits(SERVO_PAN, -91, 60) == ESP_ERR_INVALID_ARG, "below -90 accepted");
    CHECK(servo_set_limits(SERVO_PAN, -60, 82) == ESP_ERR_INVALID_ARG,
          "above the 2400 us angle accepted");
    CHECK(servo_set_limits(SERVO_PAN, 5, 60) == ESP_ERR_INVALID_ARG,
          "a range excluding centre accepted — init could never centre");
    CHECK(servo_set_limits(SERVO_PAN, 0, 0) == ESP_ERR_INVALID_ARG, "an empty range accepted");

    reset_bus();
    CHECK(servo_set_limits(SERVO_PAN, -90, 81) == ESP_OK, "the full hardware range is allowed");
    CHECK(s_event_count == 0, "widening must not move the servo");
    CHECK(servo_set_angle(SERVO_PAN, 81) == ESP_OK, "a widened limit must be usable");
}

static void test_narrowing_the_limits_pulls_the_servo_inside(void)
{
    reset_bus();

    CHECK(servo_set_limits(SERVO_PAN, -60, 60) == ESP_OK, "narrow pan to -60..60");
    CHECK(s_event_count == 1, "expected one corrective write, got %d", s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, servo_angle_to_count(SERVO_PAN, 60));

    int16_t pan = 0;
    CHECK(servo_get_angle(SERVO_PAN, &pan) == ESP_OK && pan == 60,
          "pan must now read the new limit, got %d", pan);
}

/* The bench incident: centred at 50 Hz (count 307), then `servo freq 200`
 * without a re-send left count 307 on a 5 ms period — a ~0.37 ms pulse that
 * held both servos against their end stops until USB was pulled. */
static void test_a_frequency_change_keeps_every_pulse_width(void)
{
    /* State from the previous test: pan=60, tilt=0, both enabled, 200 Hz. */
    reset_bus();

    CHECK(servo_set_pwm_frequency(50u) == ESP_OK, "change to 50 Hz");
    CHECK(s_event_count == 5, "expected release x2, freq, re-send x2; got %d events",
          s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, 0u);
    CHECK_WRITE(1, SERVO_TILT_CHANNEL, 0u);
    CHECK(s_events[2].kind == EV_FREQ && s_events[2].value == 50u,
          "the prescaler must change only after both outputs are released");
    CHECK_WRITE(3, SERVO_PAN_CHANNEL, count_for_pulse(1500u + 60u * 1000u / 90u, 50u));
    CHECK_WRITE(4, SERVO_TILT_CHANNEL, count_for_pulse(1500u, 50u));

    reset_bus();
    CHECK(servo_set_pwm_frequency(200u) == ESP_OK, "back to 200 Hz");
    CHECK_WRITE(4, SERVO_TILT_CHANNEL, 1228u);
    CHECK(s_event_count == 5 && s_events[4].value != 307u,
          "the 50 Hz centre count must never survive onto a 200 Hz period");
}

static void test_a_failed_release_leaves_the_frequency_alone(void)
{
    reset_bus();
    s_next_result = ESP_FAIL;

    CHECK(servo_set_pwm_frequency(50u) == ESP_FAIL, "the release error must be returned");
    CHECK(s_event_count == 1 && s_events[0].kind == EV_WRITE,
          "must stop after the failed release, got %d events", s_event_count);
    CHECK(s_pwm_hz == 200u, "frequency changed to %u despite the failed release",
          (unsigned)s_pwm_hz);

    reset_bus();
}

static void test_off_releases_the_outputs_and_on_restores_them(void)
{
    reset_bus();

    CHECK(servo_disable_all() == ESP_OK, "servo off");
    CHECK(s_event_count == 2, "off must write both channels, got %d", s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, 0u);
    CHECK_WRITE(1, SERVO_TILT_CHANNEL, 0u);
    CHECK(!servo_is_enabled(SERVO_PAN) && !servo_is_enabled(SERVO_TILT), "both must read off");

    reset_bus();
    CHECK(servo_set_angle(SERVO_TILT, 10) == ESP_ERR_INVALID_STATE, "a move while off");
    CHECK(s_event_count == 0, "a move while off must not touch the bus");

    /* A frequency change while off must not wake the outputs up. */
    CHECK(servo_set_pwm_frequency(50u) == ESP_OK, "freq while off");
    CHECK(s_event_count == 1 && s_events[0].kind == EV_FREQ,
          "only the prescaler may change while off, got %d events", s_event_count);

    reset_bus();
    CHECK(servo_enable_all() == ESP_OK, "servo on");
    CHECK(s_event_count == 2, "on must re-send both channels, got %d", s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, servo_angle_to_count(SERVO_PAN, 60));
    CHECK_WRITE(1, SERVO_TILT_CHANNEL, count_for_pulse(1500u, 50u));

    CHECK(servo_set_pwm_frequency(200u) == ESP_OK, "restore 200 Hz");
}

static void test_the_exercise_never_leaves_the_limits(void)
{
    CHECK(servo_set_limits(SERVO_TILT, -20, 25) == ESP_OK, "tilt -20..25");
    reset_bus();

    CHECK(servo_exercise() == ESP_OK, "exercise");
    CHECK(s_event_count == 10, "expected ten steps, got %d", s_event_count);

    const uint16_t pan_lo = servo_angle_to_count(SERVO_PAN, -60);
    const uint16_t pan_hi = servo_angle_to_count(SERVO_PAN, 60);
    const uint16_t tilt_lo = servo_angle_to_count(SERVO_TILT, -20);
    const uint16_t tilt_hi = servo_angle_to_count(SERVO_TILT, 25);
    bool reached_tilt_max = false;

    for (int i = 0; i < s_event_count && i < 32; i++) {
        const uint16_t v = s_events[i].value;
        if (s_events[i].channel == SERVO_PAN_CHANNEL) {
            CHECK(v >= pan_lo && v <= pan_hi, "pan step %d count %u outside %u..%u", i, (unsigned)v,
                  (unsigned)pan_lo, (unsigned)pan_hi);
        } else {
            CHECK(v >= tilt_lo && v <= tilt_hi, "tilt step %d count %u outside %u..%u", i,
                  (unsigned)v, (unsigned)tilt_lo, (unsigned)tilt_hi);
            reached_tilt_max = reached_tilt_max || v == tilt_hi;
        }
    }
    CHECK(reached_tilt_max, "the exercise must travel to the live tilt limit, not a constant");
}

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"failed_centring_leaves_module_uninitialised",
         test_failed_centring_leaves_module_uninitialised},
        {"init_centres_both_servos", test_init_centres_both_servos},
        {"set_angle_writes_after_init", test_set_angle_writes_after_init},
        {"count_tracks_the_pwm_frequency", test_the_count_tracks_the_pwm_frequency},
        {"an_impossible_pulse_is_clamped", test_an_impossible_pulse_is_clamped},
        {"an_angle_is_a_real_servo_angle_within_the_sg90_range",
         test_an_angle_is_a_real_servo_angle_within_the_sg90_range},
        {"the_boot_limits_gate_every_move", test_the_boot_limits_gate_every_move},
        {"set_limits_rejects_what_the_hardware_or_centring_cannot_do",
         test_set_limits_rejects_what_the_hardware_or_centring_cannot_do},
        {"narrowing_the_limits_pulls_the_servo_inside",
         test_narrowing_the_limits_pulls_the_servo_inside},
        {"a_frequency_change_keeps_every_pulse_width",
         test_a_frequency_change_keeps_every_pulse_width},
        {"a_failed_release_leaves_the_frequency_alone",
         test_a_failed_release_leaves_the_frequency_alone},
        {"off_releases_the_outputs_and_on_restores_them",
         test_off_releases_the_outputs_and_on_restores_them},
        {"the_exercise_never_leaves_the_limits", test_the_exercise_never_leaves_the_limits},
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = s_failures;
        tests[i].fn();
        printf("%s %s\n", s_failures == before ? "PASS" : "FAIL", tests[i].name);
    }
    printf("%d failure(s)\n", s_failures);
    return s_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
