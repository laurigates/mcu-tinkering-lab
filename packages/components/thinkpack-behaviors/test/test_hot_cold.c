/**
 * @file test_hot_cold.c
 * @brief Host-based unit tests for the hot_cold RSSI indicator.
 */

#include "hot_cold.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* Reset + priming                                                     */
/* ------------------------------------------------------------------ */

static void test_reset_marks_unprimed(void)
{
    hot_cold_state_t s = {.smoothed_q8 = 999, .primed = true};
    hot_cold_reset(&s);
    TEST_ASSERT_FALSE(s.primed);
    TEST_ASSERT_EQUAL(0, s.smoothed_q8);
}

static void test_first_sample_adopted_verbatim(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, -60);
    TEST_ASSERT_TRUE(s.primed);
    TEST_ASSERT_EQUAL(-60, hot_cold_smoothed_dbm(&s));
}

static void test_unprimed_smoothed_reports_int8_min(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    TEST_ASSERT_EQUAL(-128, hot_cold_smoothed_dbm(&s));
}

/* ------------------------------------------------------------------ */
/* EMA convergence                                                     */
/* ------------------------------------------------------------------ */

static void test_ema_converges_toward_sample(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, -80);
    /* Feed -40 repeatedly; should approach -40 monotonically. */
    int8_t prev = -80;
    for (int i = 0; i < 30; i++) {
        hot_cold_update(&s, -40);
        int8_t now = hot_cold_smoothed_dbm(&s);
        /* Non-increasing distance to -40. */
        int dist_prev = prev < -40 ? (-40 - prev) : (prev + 40);
        int dist_now = now < -40 ? (-40 - now) : (now + 40);
        TEST_ASSERT_TRUE(dist_now <= dist_prev);
        prev = now;
    }
    TEST_ASSERT_EQUAL(-40, hot_cold_smoothed_dbm(&s));
}

static void test_ema_step_is_quarter_of_delta(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, -80);
    hot_cold_update(&s, -40); /* delta = +40, expected step = +10 */
    TEST_ASSERT_EQUAL(-70, hot_cold_smoothed_dbm(&s));
}

/* ------------------------------------------------------------------ */
/* Colour gradient                                                     */
/* ------------------------------------------------------------------ */

static void test_color_unprimed_is_pure_blue(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    uint8_t r = 42, g = 42, b = 42;
    hot_cold_get_color(&s, &r, &g, &b);
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(255, b);
}

static void test_color_at_rssi_min_is_blue(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, (int8_t)HOT_COLD_RSSI_MIN);
    uint8_t r, g, b;
    hot_cold_get_color(&s, &r, &g, &b);
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(255, b);
}

static void test_color_at_rssi_max_is_red(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, (int8_t)HOT_COLD_RSSI_MAX);
    uint8_t r, g, b;
    hot_cold_get_color(&s, &r, &g, &b);
    TEST_ASSERT_EQUAL(255, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(0, b);
}

static void test_color_clamps_above_max(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, -20); /* above RSSI_MAX */
    uint8_t r, g, b;
    hot_cold_get_color(&s, &r, &g, &b);
    TEST_ASSERT_EQUAL(255, r);
    TEST_ASSERT_EQUAL(0, b);
}

static void test_color_clamps_below_min(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, -110); /* below RSSI_MIN, but stored as int8 */
    uint8_t r, g, b;
    hot_cold_get_color(&s, &r, &g, &b);
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_EQUAL(255, b);
}

static void test_color_midpoint_is_purple(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    /* Midpoint between -90 and -30 is -60. */
    hot_cold_update(&s, -60);
    uint8_t r, g, b;
    hot_cold_get_color(&s, &r, &g, &b);
    TEST_ASSERT_TRUE(r > 120 && r < 136);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_TRUE(b > 120 && b < 136);
}

static void test_color_accepts_null_channels(void)
{
    hot_cold_state_t s;
    hot_cold_reset(&s);
    hot_cold_update(&s, -30);
    /* Should not crash. */
    hot_cold_get_color(&s, NULL, NULL, NULL);
    uint8_t r = 0;
    hot_cold_get_color(&s, &r, NULL, NULL);
    TEST_ASSERT_EQUAL(255, r);
}

/* ------------------------------------------------------------------ */
/* Null-safety                                                         */
/* ------------------------------------------------------------------ */

static void test_null_state_noops(void)
{
    hot_cold_reset(NULL);
    hot_cold_update(NULL, -50);
    TEST_ASSERT_EQUAL(-128, hot_cold_smoothed_dbm(NULL));
    uint8_t r = 5, g = 5, b = 5;
    hot_cold_get_color(NULL, &r, &g, &b);
    /* NULL state treated as unprimed → pure blue */
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(255, b);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_reset_marks_unprimed);
    RUN_TEST(test_first_sample_adopted_verbatim);
    RUN_TEST(test_unprimed_smoothed_reports_int8_min);
    RUN_TEST(test_ema_converges_toward_sample);
    RUN_TEST(test_ema_step_is_quarter_of_delta);
    RUN_TEST(test_color_unprimed_is_pure_blue);
    RUN_TEST(test_color_at_rssi_min_is_blue);
    RUN_TEST(test_color_at_rssi_max_is_red);
    RUN_TEST(test_color_clamps_above_max);
    RUN_TEST(test_color_clamps_below_min);
    RUN_TEST(test_color_midpoint_is_purple);
    RUN_TEST(test_color_accepts_null_channels);
    RUN_TEST(test_null_state_noops);
    return UNITY_END();
}
