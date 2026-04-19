/**
 * @file test_runner.c
 * @brief Unity test runner for thinkpack-power host tests.
 *
 * The per-topic test files are compiled into this TU via #include so the
 * Unity-compat static counters accumulate correctly.
 */

#include <stdint.h>

#include "thinkpack_power.h"
#include "unity_compat.h"

/* clang-format off */
#include "test_classify.c"
#include "test_adaptive_beacon.c"
#include "test_led_pulse.c"
/* clang-format on */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_classify_well_above_ok_is_ok);
    RUN_TEST(test_classify_exactly_ok_threshold_is_ok);
    RUN_TEST(test_classify_just_below_ok_is_low);
    RUN_TEST(test_classify_middle_of_low_band);
    RUN_TEST(test_classify_exactly_low_threshold_is_low);
    RUN_TEST(test_classify_just_below_low_is_critical);
    RUN_TEST(test_classify_deeply_discharged_is_critical);

    RUN_TEST(test_beacon_ok_is_500ms);
    RUN_TEST(test_beacon_low_is_2000ms);
    RUN_TEST(test_beacon_critical_is_5000ms);
    RUN_TEST(test_beacon_unknown_state_fails_safe_to_5000ms);

    RUN_TEST(test_led_ok_is_black);
    RUN_TEST(test_led_critical_is_solid_red);
    RUN_TEST(test_led_low_pulse_at_t0_is_off);
    RUN_TEST(test_led_low_pulse_mid_cycle_is_full);
    RUN_TEST(test_led_low_pulse_at_cycle_end_is_off);
    RUN_TEST(test_led_low_pulse_monotonic_rise);
    RUN_TEST(test_led_low_pulse_null_args_safe);

    int failures = UNITY_END();
    return failures == 0 ? 0 : 1;
}
