/**
 * @file test_activity_trace.c
 *
 * Pins the LED indicator's shadow against the chip it cannot read back.
 *
 * activity_trace writes an LED only when its colour changes, because at 25 Hz
 * an unconditional write would load the shared I2C bus for no benefit. The
 * shadow that makes that possible is a claim about a PCA9685 whose contents are
 * unreadable, so it has to expire: a chip that browned out or was re-seated
 * holds its power-on defaults — all outputs off — while the shadow still says
 * the colour is lit. Without the expiry the indicator stays dark until the
 * colour happens to change, and the states worth seeing are exactly the ones
 * that do not change (a held red for a failed capture, a held blue for a
 * request that never returned).
 *
 * That is the case a bench cannot stage, which is why it is here rather than in
 * a bring-up check. activity_trace.c is compiled unmodified apart from
 * ACTIVITY_TRACE_HOST_TEST, which only changes activity_trace_tick()'s linkage
 * so the test can drive the task's loop body directly; the led_controller it
 * writes through and the clock it reads are the stubs below.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "activity_trace.h"
#include "led_controller.h"

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

/* ---- recording led_controller stub -------------------------------------- */

#define MAX_WRITES 32

static struct {
    led_position_t pos;
    rgb_color_t color;
} s_writes[MAX_WRITES];
static int s_write_count;
static int s_off_all_count;
static esp_err_t s_next_result = ESP_OK;

const rgb_color_t LED_COLOR_OFF = {0, 0, 0};
const rgb_color_t LED_COLOR_RED = {255, 0, 0};
const rgb_color_t LED_COLOR_GREEN = {0, 255, 0};
const rgb_color_t LED_COLOR_BLUE = {0, 0, 255};
const rgb_color_t LED_COLOR_WHITE = {255, 255, 255};
const rgb_color_t LED_COLOR_YELLOW = {255, 255, 0};

esp_err_t led_set_color(led_position_t position, const rgb_color_t *color)
{
    if (s_write_count < MAX_WRITES) {
        s_writes[s_write_count].pos = position;
        s_writes[s_write_count].color = *color;
    }
    s_write_count++;
    return s_next_result;
}

esp_err_t led_turn_off_all(void)
{
    s_off_all_count++;
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

static void reset_leds(void)
{
    s_write_count = 0;
    s_off_all_count = 0;
    s_next_result = ESP_OK;
}

/** Run `ms` of ticks at the indicator's real cadence. */
static void tick_for_ms(uint32_t ms)
{
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += 40u) {
        activity_trace_tick();
        advance_ms(40u);
    }
}

/* The module keeps its state in file statics with no reset hook, so the tests
 * run in one process in a fixed order. */

static void test_first_tick_writes_both_leds(void)
{
    reset_leds();
    CHECK(activity_trace_init() == ESP_OK, "init should succeed");

    activity_trace_tick();
    CHECK(s_write_count == 2, "the first tick must write both LEDs, got %d", s_write_count);
    CHECK(s_writes[0].pos == LED_LEFT && s_writes[1].pos == LED_RIGHT,
          "expected left then right, got %d then %d", s_writes[0].pos, s_writes[1].pos);
}

static void test_an_unchanged_colour_is_not_rewritten(void)
{
    reset_leds();

    /* Just under the refresh interval, so only the suppression is under test. */
    tick_for_ms(LED_REFRESH_INTERVAL_MS - 200u);
    CHECK(s_write_count == 0, "an unchanged colour must not be rewritten, got %d writes",
          s_write_count);
}

static void test_the_shadow_expires(void)
{
    reset_leds();

    /* Cross the interval. Both LEDs are due, and neither colour has changed —
     * this is the tick that recovers a chip which silently lost its state. */
    tick_for_ms(400u);
    CHECK(s_write_count == 2, "both LEDs must be re-asserted on expiry, got %d", s_write_count);

    /* And the expiry re-arms rather than firing on every tick from then on. */
    reset_leds();
    tick_for_ms(400u);
    CHECK(s_write_count == 0, "the refresh must re-arm, got %d writes", s_write_count);
}

static void test_a_changed_colour_writes_immediately(void)
{
    reset_leds();

    /* A failed capture turns the camera LED red and holds it — the state whose
     * staleness the expiry exists to correct. */
    activity_trace_camera(false, 0, 0);
    activity_trace_tick();

    CHECK(s_write_count == 1, "only the changed LED should write, got %d", s_write_count);
    CHECK(s_writes[0].pos == LED_LEFT, "the camera LED is the left one, got %d", s_writes[0].pos);
    CHECK(s_writes[0].color.red == LED_COLOR_RED.red && s_writes[0].color.green == 0,
          "a failed capture should hold red");
}

static void test_a_failed_write_is_not_remembered_as_displayed(void)
{
    reset_leds();
    s_next_result = ESP_FAIL;

    /* Change the colour back so a write is due, and fail it. */
    activity_trace_camera(true, 1000, 10);
    activity_trace_tick();
    CHECK(s_write_count == 1, "the failing attempt should be made, got %d", s_write_count);

    /* Same colour, same instant. A shadow that recorded the failed write would
     * suppress this and leave the LED showing the old colour indefinitely. */
    reset_leds();
    activity_trace_tick();
    CHECK(s_write_count == 1, "a failed write must not suppress the retry, got %d", s_write_count);
}

static void test_leds_off_stops_touching_the_bus(void)
{
    reset_leds();
    activity_trace_set_leds(false);

    tick_for_ms(3u * LED_REFRESH_INTERVAL_MS);
    CHECK(s_off_all_count == 1, "the LEDs should be released exactly once, got %d",
          s_off_all_count);
    CHECK(s_write_count == 0, "off must mean off — the refresh must not resume writes, got %d",
          s_write_count);

    /* Re-enabling writes both again, because releasing them invalidated the
     * shadow rather than leaving it claiming a colour the LEDs no longer show. */
    reset_leds();
    activity_trace_set_leds(true);
    activity_trace_tick();
    CHECK(s_write_count == 2, "re-enabling must rewrite both LEDs, got %d", s_write_count);
}

static void test_refresh_survives_the_uint32_millisecond_wrap(void)
{
    reset_leds();

    /* Park the clock just under the uint32 ms wrap and write there. */
    s_now_us = ((int64_t)0xFFFFFF00u) * 1000;
    activity_trace_tick();
    reset_leds();

    /* Cross the wrap: now_ms() is small, written_ms is huge. A signed elapsed
     * time reads as enormously negative here and either refreshes on every tick
     * or never refreshes again. */
    advance_ms(200u);
    activity_trace_tick();
    CHECK(s_write_count == 0, "200 ms after the write, across the wrap, must suppress: %d",
          s_write_count);

    advance_ms(LED_REFRESH_INTERVAL_MS);
    activity_trace_tick();
    CHECK(s_write_count == 2, "the refresh must fall due across the wrap, got %d", s_write_count);
}

int main(void)
{
    printf("test_activity_trace\n");

    test_first_tick_writes_both_leds();
    test_an_unchanged_colour_is_not_rewritten();
    test_the_shadow_expires();
    test_a_changed_colour_writes_immediately();
    test_a_failed_write_is_not_remembered_as_displayed();
    test_leds_off_stops_touching_the_bus();
    test_refresh_survives_the_uint32_millisecond_wrap();

    if (s_failures != 0) {
        printf("FAILED (%d)\n", s_failures);
        return EXIT_FAILURE;
    }
    printf("PASSED\n");
    return EXIT_SUCCESS;
}
