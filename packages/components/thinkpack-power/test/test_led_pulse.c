/**
 * @file test_led_pulse.c
 * @brief Tests for thinkpack_power_low_battery_led_tick().
 *
 * Compiled as part of test_runner.c via #include so the Unity static
 * counters stay in a single translation unit.
 */

static void test_led_ok_is_black(void)
{
    uint8_t r = 99, g = 99, b = 99;
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_OK, 0, &r, &g, &b);
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(0, b);
}

static void test_led_critical_is_solid_red(void)
{
    uint8_t r = 0, g = 0, b = 0;
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_CRITICAL, 0, &r, &g, &b);
    TEST_ASSERT_EQUAL(255, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(0, b);

    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_CRITICAL, 250, &r, &g, &b);
    TEST_ASSERT_EQUAL(255, r);
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_CRITICAL, 987654, &r, &g, &b);
    TEST_ASSERT_EQUAL(255, r);
}

static void test_led_low_pulse_at_t0_is_off(void)
{
    uint8_t r = 42, g = 42, b = 42;
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_LOW, 0, &r, &g, &b);
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(0, b);
}

static void test_led_low_pulse_mid_cycle_is_full(void)
{
    uint8_t r = 0, g = 0, b = 0;
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_LOW, 500, &r, &g, &b);
    TEST_ASSERT_EQUAL(255, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(0, b);
}

static void test_led_low_pulse_at_cycle_end_is_off(void)
{
    uint8_t r = 99, g = 99, b = 99;
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_LOW, 1000, &r, &g, &b);
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_EQUAL(0, g);
    TEST_ASSERT_EQUAL(0, b);
}

static void test_led_low_pulse_monotonic_rise(void)
{
    /* Rising edge at 250ms should be ~half brightness. */
    uint8_t r = 0, g = 0, b = 0;
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_LOW, 250, &r, &g, &b);
    TEST_ASSERT_TRUE(r > 100 && r < 160);
}

static void test_led_low_pulse_null_args_safe(void)
{
    /* NULL pointers must not crash. */
    thinkpack_power_low_battery_led_tick(THINKPACK_POWER_LOW, 500, NULL, NULL, NULL);
}
