/**
 * @file test_classify.c
 * @brief Boundary tests for thinkpack_power_classify().
 *
 * Compiled as part of test_runner.c via #include so the Unity static
 * counters stay in a single translation unit.
 */

static void test_classify_well_above_ok_is_ok(void)
{
    TEST_ASSERT_EQUAL(THINKPACK_POWER_OK, thinkpack_power_classify(4200));
    TEST_ASSERT_EQUAL(THINKPACK_POWER_OK, thinkpack_power_classify(3800));
}

static void test_classify_exactly_ok_threshold_is_ok(void)
{
    TEST_ASSERT_EQUAL(THINKPACK_POWER_OK, thinkpack_power_classify(3600));
}

static void test_classify_just_below_ok_is_low(void)
{
    TEST_ASSERT_EQUAL(THINKPACK_POWER_LOW, thinkpack_power_classify(3599));
}

static void test_classify_middle_of_low_band(void)
{
    TEST_ASSERT_EQUAL(THINKPACK_POWER_LOW, thinkpack_power_classify(3450));
}

static void test_classify_exactly_low_threshold_is_low(void)
{
    TEST_ASSERT_EQUAL(THINKPACK_POWER_LOW, thinkpack_power_classify(3300));
}

static void test_classify_just_below_low_is_critical(void)
{
    TEST_ASSERT_EQUAL(THINKPACK_POWER_CRITICAL, thinkpack_power_classify(3299));
}

static void test_classify_deeply_discharged_is_critical(void)
{
    TEST_ASSERT_EQUAL(THINKPACK_POWER_CRITICAL, thinkpack_power_classify(3000));
    TEST_ASSERT_EQUAL(THINKPACK_POWER_CRITICAL, thinkpack_power_classify(0));
}
