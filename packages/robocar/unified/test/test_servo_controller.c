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
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = s_failures;
        tests[i].fn();
        printf("%s %s\n", s_failures == before ? "PASS" : "FAIL", tests[i].name);
    }
    printf("%d failure(s)\n", s_failures);
    return s_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
