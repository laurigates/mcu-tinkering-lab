/**
 * @file test_status_led.c
 * @brief Host-based unit tests for the status_led component.
 *
 * Tests the LED mode state machine and blink timing logic using mocked
 * hardware drivers (led_strip, esp_timer, FreeRTOS). No hardware required.
 *
 * Build and run: make test
 */

#include "status_led.h"
#include "unity_compat.h"

/* Mock state from mocks.c */
extern uint8_t mock_led_r;
extern uint8_t mock_led_g;
extern uint8_t mock_led_b;
extern int mock_set_pixel_calls;
extern int mock_refresh_calls;
extern int64_t mock_timer_us;

/* Must match the constants in status_led.c */
#define LED_BRIGHTNESS 32
#define BLINK_SLOW_US (500 * 1000)
#define BLINK_FAST_US (200 * 1000)

/* ---------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ---------------------------------------------------------------------- */

static void reset_mocks(void)
{
    mock_led_r = 0;
    mock_led_g = 0;
    mock_led_b = 0;
    mock_set_pixel_calls = 0;
    mock_refresh_calls = 0;
    mock_timer_us = 0;
}

/**
 * Initialise the LED driver (uses mock hardware) and switch to OFF mode.
 * Call at the start of each test that exercises status_led_update().
 */
static void setup(void)
{
    reset_mocks();
    status_led_init();
    status_led_set_mode(STATUS_LED_OFF);
}

/* ---------------------------------------------------------------------- */
/* Tests: initialisation                                                   */
/* ---------------------------------------------------------------------- */

static void test_init_returns_ok(void)
{
    reset_mocks();
    esp_err_t ret = status_led_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/* ---------------------------------------------------------------------- */
/* Tests: solid-colour modes                                               */
/* ---------------------------------------------------------------------- */

static void test_mode_off_output_dark(void)
{
    setup();
    status_led_set_mode(STATUS_LED_OFF);
    mock_timer_us = 10 * 1000 * 1000LL; /* 10 s — well past any blink period */
    status_led_update();

    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(0, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);
}

static void test_mode_usb_error_output_red_solid(void)
{
    setup();
    status_led_set_mode(STATUS_LED_USB_ERROR);
    mock_timer_us = 10 * 1000 * 1000LL;
    status_led_update();

    TEST_ASSERT_EQUAL(LED_BRIGHTNESS, mock_led_r);
    TEST_ASSERT_EQUAL(0, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);
}

static void test_mode_bridging_output_green_solid(void)
{
    setup();
    status_led_set_mode(STATUS_LED_BRIDGING);
    mock_timer_us = 10 * 1000 * 1000LL;
    status_led_update();

    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(LED_BRIGHTNESS, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);
}

static void test_usb_error_stays_red_across_multiple_updates(void)
{
    setup();
    status_led_set_mode(STATUS_LED_USB_ERROR);

    for (int i = 0; i < 10; i++) {
        mock_timer_us = (int64_t)i * 100 * 1000; /* advance 100 ms each iteration */
        status_led_update();
        TEST_ASSERT_EQUAL(LED_BRIGHTNESS, mock_led_r);
        TEST_ASSERT_EQUAL(0, mock_led_g);
        TEST_ASSERT_EQUAL(0, mock_led_b);
    }
}

static void test_bridging_stays_green_across_multiple_updates(void)
{
    setup();
    status_led_set_mode(STATUS_LED_BRIDGING);

    for (int i = 0; i < 10; i++) {
        mock_timer_us = (int64_t)i * 100 * 1000;
        status_led_update();
        TEST_ASSERT_EQUAL(0, mock_led_r);
        TEST_ASSERT_EQUAL(LED_BRIGHTNESS, mock_led_g);
        TEST_ASSERT_EQUAL(0, mock_led_b);
    }
}

/* ---------------------------------------------------------------------- */
/* Tests: refresh is called on every update                                */
/* ---------------------------------------------------------------------- */

static void test_update_calls_refresh(void)
{
    setup();
    status_led_set_mode(STATUS_LED_BRIDGING);
    int before = mock_refresh_calls;
    status_led_update();
    TEST_ASSERT_TRUE(mock_refresh_calls > before);
}

/* ---------------------------------------------------------------------- */
/* Tests: mode transitions                                                 */
/* ---------------------------------------------------------------------- */

static void test_mode_transition_off_to_bridging(void)
{
    setup();
    mock_timer_us = 5 * 1000 * 1000LL;

    status_led_set_mode(STATUS_LED_OFF);
    status_led_update();
    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(0, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);

    status_led_set_mode(STATUS_LED_BRIDGING);
    status_led_update();
    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(LED_BRIGHTNESS, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);
}

static void test_mode_transition_error_to_off(void)
{
    setup();
    mock_timer_us = 5 * 1000 * 1000LL;

    status_led_set_mode(STATUS_LED_USB_ERROR);
    status_led_update();
    TEST_ASSERT_EQUAL(LED_BRIGHTNESS, mock_led_r);

    status_led_set_mode(STATUS_LED_OFF);
    status_led_update();
    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(0, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);
}

/* ---------------------------------------------------------------------- */
/* Tests: blink toggling                                                   */
/*                                                                         */
/* Blink tests advance mock_timer_us by more than one full period so that  */
/* at least one toggle is guaranteed regardless of internal blink state.   */
/* We verify that the LED value CHANGES between two consecutive samples    */
/* separated by exactly one blink period, which confirms toggling.        */
/* ---------------------------------------------------------------------- */

static void test_scanning_blink_toggles_over_slow_period(void)
{
    setup();
    status_led_set_mode(STATUS_LED_SCANNING);

    /* Sample at a large absolute time so s_last_toggle_us is stable. */
    mock_timer_us = 10 * 1000 * 1000LL; /* 10 s */
    status_led_update();
    uint8_t blue_at_T = mock_led_b;
    /* r and g must always be 0 in SCANNING mode */
    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(0, mock_led_g);

    /* Advance exactly one slow blink period + 1 µs to trigger toggle. */
    mock_timer_us += BLINK_SLOW_US + 1;
    status_led_update();
    uint8_t blue_after_period = mock_led_b;

    /* The two samples must differ (one ON, one OFF). */
    TEST_ASSERT_NOT_EQUAL(blue_at_T, blue_after_period);
    /* Both values must be valid (either 0 or LED_BRIGHTNESS). */
    TEST_ASSERT_TRUE(blue_at_T == 0 || blue_at_T == LED_BRIGHTNESS);
    TEST_ASSERT_TRUE(blue_after_period == 0 || blue_after_period == LED_BRIGHTNESS);
}

static void test_connected_no_usb_blink_toggles_slow(void)
{
    setup();
    status_led_set_mode(STATUS_LED_CONNECTED_NO_USB);

    mock_timer_us = 20 * 1000 * 1000LL;
    status_led_update();
    uint8_t r0 = mock_led_r;
    uint8_t b0 = mock_led_b;

    mock_timer_us += BLINK_SLOW_US + 1;
    status_led_update();
    uint8_t r1 = mock_led_r;
    uint8_t b1 = mock_led_b;

    /* Purple blink: r and b move together, g is always 0. */
    TEST_ASSERT_EQUAL(0, mock_led_g);
    TEST_ASSERT_NOT_EQUAL(r0, r1);
    TEST_ASSERT_NOT_EQUAL(b0, b1);
}

static void test_connected_usb_blink_toggles_fast(void)
{
    setup();
    status_led_set_mode(STATUS_LED_CONNECTED_USB);

    mock_timer_us = 30 * 1000 * 1000LL;
    status_led_update();
    uint8_t r0 = mock_led_r;

    /* Fast period is 200 ms — advance by that plus a safety margin. */
    mock_timer_us += BLINK_FAST_US + 1;
    status_led_update();
    uint8_t r1 = mock_led_r;

    TEST_ASSERT_NOT_EQUAL(r0, r1);
}

/* ---------------------------------------------------------------------- */
/* Tests: status_led_flash                                                 */
/* ---------------------------------------------------------------------- */

static void test_flash_sets_color_then_clears(void)
{
    setup();
    /* flash calls vTaskDelay (stubbed as no-op) then clears the LED */
    status_led_flash(255, 128, 64, 10);
    /* After flash completes the LED should be dark */
    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(0, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);
}

static void test_flash_does_not_affect_current_mode(void)
{
    /* After flash, the next update should honour the current mode, not the
     * flash colour. */
    setup();
    status_led_set_mode(STATUS_LED_BRIDGING);
    status_led_flash(255, 0, 0, 1);

    mock_timer_us = 5 * 1000 * 1000LL;
    status_led_update();

    TEST_ASSERT_EQUAL(0, mock_led_r);
    TEST_ASSERT_EQUAL(LED_BRIGHTNESS, mock_led_g);
    TEST_ASSERT_EQUAL(0, mock_led_b);
}

/* ---------------------------------------------------------------------- */
/* main                                                                    */
/* ---------------------------------------------------------------------- */

int main(void)
{
    UNITY_BEGIN();

    /* Initialisation */
    RUN_TEST(test_init_returns_ok);

    /* Solid-colour modes */
    RUN_TEST(test_mode_off_output_dark);
    RUN_TEST(test_mode_usb_error_output_red_solid);
    RUN_TEST(test_mode_bridging_output_green_solid);
    RUN_TEST(test_usb_error_stays_red_across_multiple_updates);
    RUN_TEST(test_bridging_stays_green_across_multiple_updates);

    /* Refresh call */
    RUN_TEST(test_update_calls_refresh);

    /* Mode transitions */
    RUN_TEST(test_mode_transition_off_to_bridging);
    RUN_TEST(test_mode_transition_error_to_off);

    /* Blink toggling */
    RUN_TEST(test_scanning_blink_toggles_over_slow_period);
    RUN_TEST(test_connected_no_usb_blink_toggles_slow);
    RUN_TEST(test_connected_usb_blink_toggles_fast);

    /* Flash */
    RUN_TEST(test_flash_sets_color_then_clears);
    RUN_TEST(test_flash_does_not_affect_current_mode);

    int failures = UNITY_END();
    return (failures > 0) ? 1 : 0;
}
