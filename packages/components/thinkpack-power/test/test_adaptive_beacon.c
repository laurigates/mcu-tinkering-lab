/**
 * @file test_adaptive_beacon.c
 * @brief Tests for state -> beacon-interval mapping.
 *
 * Compiled as part of test_runner.c via #include so the Unity static
 * counters stay in a single translation unit.
 */

static void test_beacon_ok_is_500ms(void)
{
    TEST_ASSERT_EQUAL(500, thinkpack_power_adaptive_beacon_interval_ms(THINKPACK_POWER_OK));
}

static void test_beacon_low_is_2000ms(void)
{
    TEST_ASSERT_EQUAL(2000, thinkpack_power_adaptive_beacon_interval_ms(THINKPACK_POWER_LOW));
}

static void test_beacon_critical_is_5000ms(void)
{
    TEST_ASSERT_EQUAL(5000, thinkpack_power_adaptive_beacon_interval_ms(THINKPACK_POWER_CRITICAL));
}

static void test_beacon_unknown_state_fails_safe_to_5000ms(void)
{
    /* Any out-of-range state must not crash and must clamp to the lowest
     * duty cycle (safest on battery). */
    TEST_ASSERT_EQUAL(
        5000, thinkpack_power_adaptive_beacon_interval_ms((thinkpack_power_battery_state_t)99));
}
