/**
 * @file test_servo_controller.c
 *
 * Pins servo_controller_init() against the bus it initialises through.
 *
 * Init centres both servos via servo_set_angle(), and servo_set_angle()
 * refuses with ESP_ERR_INVALID_STATE until the module is initialised. The
 * flag used to be raised only AFTER centring, so init could never succeed:
 * it returned ESP_ERR_INVALID_STATE without a single bus write, and the
 * ESP_ERROR_CHECK around init_hardware() rebooted the board. A bare board
 * never reached the call (the hardware phase is skipped when I2C init
 * fails), which is why it survived until the PCA9685 was first fitted.
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

/* servo_move_smooth() sleeps between steps; nothing here calls it, but the
 * symbol has to link. */
void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}

/* ---- recording i2c_bus stub --------------------------------------------- */

static struct {
    uint8_t channel;
    uint16_t count;
} s_writes[8];
static int s_write_count;
static esp_err_t s_next_result = ESP_OK;

/* The chip-wide prescaler the pulse maths reads. Settable so the tests can ask
 * what a given angle becomes at a frequency other than the shipped 200 Hz. */
static uint16_t s_pwm_hz = 200u;

uint16_t i2c_bus_pca9685_frequency(void)
{
    return s_pwm_hz;
}

esp_err_t i2c_bus_pca9685_set(uint8_t channel, uint16_t count)
{
    if (s_write_count < (int)(sizeof(s_writes) / sizeof(s_writes[0]))) {
        s_writes[s_write_count].channel = channel;
        s_writes[s_write_count].count = count;
    }
    s_write_count++;
    return s_next_result;
}

static void reset_bus(void)
{
    s_write_count = 0;
    s_next_result = ESP_OK;
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

/* The module keeps its state in a file-static struct with no reset hook, so
 * the tests run in one process in a fixed order: the failing init first,
 * then the successful one. */

static void test_failed_centring_leaves_module_uninitialised(void)
{
    reset_bus();
    s_next_result = ESP_FAIL;

    esp_err_t err = servo_controller_init();

    CHECK(err == ESP_FAIL, "init returned %d, expected the bus error", err);
    CHECK(s_write_count == 1, "expected the first centring write, got %d", s_write_count);
    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_CENTER) == ESP_ERR_INVALID_STATE,
          "a failed init must not leave the servos reported as ready");
}

static void test_init_centres_both_servos(void)
{
    reset_bus();

    esp_err_t err = servo_controller_init();

    CHECK(err == ESP_OK, "init returned %d", err);
    CHECK(s_write_count == 2, "expected two centring writes, got %d", s_write_count);
    CHECK(s_writes[0].channel == SERVO_PAN_CHANNEL, "first write to channel %u",
          s_writes[0].channel);
    CHECK(s_writes[1].channel == SERVO_TILT_CHANNEL, "second write to channel %u",
          s_writes[1].channel);
    CHECK(s_writes[0].count > 0 && s_writes[1].count > 0, "centre pulse must be non-zero");

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
    CHECK(s_write_count == 1 && s_writes[0].channel == SERVO_PAN_CHANNEL,
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

    /* The endpoints scale with it too, rather than staying put. */
    s_pwm_hz = 200u;
    const uint16_t max_200 = servo_angle_to_count(SERVO_PAN, SERVO_PAN_MAX_ANGLE);
    s_pwm_hz = 50u;
    const uint16_t max_50 = servo_angle_to_count(SERVO_PAN, SERVO_PAN_MAX_ANGLE);
    CHECK(max_50 < max_200, "a lower frequency must need a smaller count: %u vs %u",
          (unsigned)max_50, (unsigned)max_200);

    s_pwm_hz = 200u;
}

/* A pulse cannot outlast its period. At the driver's 1526 Hz ceiling the
 * 2500 us maximum is longer than the 655 us frame, and an unclamped count would
 * wrap in the 12-bit register and emerge as a SHORT pulse — a servo slamming to
 * the opposite endpoint rather than refusing. */
static void test_an_impossible_pulse_is_clamped(void)
{
    s_pwm_hz = 1526u;
    const uint16_t count = servo_angle_to_count(SERVO_TILT, SERVO_TILT_MAX_ANGLE);
    CHECK(count == 4095u, "an over-long pulse must clamp to 4095, got %u", (unsigned)count);
    s_pwm_hz = 200u;
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
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = s_failures;
        tests[i].fn();
        printf("%s %s\n", s_failures == before ? "PASS" : "FAIL", tests[i].name);
    }
    printf("%d failure(s)\n", s_failures);
    return s_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
