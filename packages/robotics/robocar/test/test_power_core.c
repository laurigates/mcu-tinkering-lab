/**
 * @file test_power_core.c
 * @brief Host tests for the pack cutoff and the power button (ADR-038, #1271).
 *
 * Pins what a bench cannot stage on demand: a motor-start sag that must not
 * trip the cutoff, a pack hovering at the threshold, a failing ADC, a robot on
 * USB power with no pack, a button still held from the power-on press, and the
 * uint32 millisecond wrap at day 49.
 */

#include <stdio.h>
#include <string.h>

#include "power_core.h"

static int g_failures;

#define ASSERT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            fflush(stdout);                                          \
            g_failures++;                                            \
        }                                                            \
    } while (0)

#define ASSERT_EQ(got, want)                                                                  \
    do {                                                                                      \
        const long long g_ = (long long)(got);                                                \
        const long long w_ = (long long)(want);                                               \
        if (g_ != w_) {                                                                       \
            printf("  FAIL %s:%d: %s = %lld, want %lld\n", __FILE__, __LINE__, #got, g_, w_); \
            fflush(stdout);                                                                   \
            g_failures++;                                                                     \
        }                                                                                     \
    } while (0)

/** The monitor's sample period in the firmware, for the scripted runs below. */
#define TICK_MS 100U

static power_pack_t new_pack(void)
{
    const power_pack_config_t cfg = POWER_PACK_CONFIG_DEFAULT;
    power_pack_t p;
    power_pack_init(&p, &cfg);
    return p;
}

/** Feed @p mv for @p ms starting at @p *now, one sample per tick; return the
 *  last state. */
static power_pack_state_t feed(power_pack_t *p, uint32_t *now, uint32_t mv, uint32_t ms)
{
    power_pack_state_t s = p->state;
    for (uint32_t t = 0; t < ms; t += TICK_MS) {
        *now += TICK_MS;
        s = power_pack_update(p, *now, mv);
    }
    return s;
}

/* ---- divider ---- */

static void test_divider(void)
{
    /* hardware.toml: 100k over 22k. 8.4 V reads 1.51 V, 6.0 V reads 1.08 V. */
    ASSERT_EQ(power_pack_mv_from_pin_mv(0), 0);
    ASSERT_EQ(power_pack_mv_from_pin_mv(1515), 8401); /* 1515 * 122 / 22 = 8401.4 */
    ASSERT_EQ(power_pack_mv_from_pin_mv(1082), 6000);
    /* Rounded, not truncated: 1 mV at the pin is 5.545 mV on the pack. */
    ASSERT_EQ(power_pack_mv_from_pin_mv(1), 6);
    /* No overflow at the ADC's widest reading. */
    ASSERT_EQ(power_pack_mv_from_pin_mv(3100), 17191);
}

/* ---- pack cutoff ---- */

static void test_no_sample_yet(void)
{
    power_pack_t p = new_pack();
    ASSERT_EQ(p.state, POWER_PACK_NO_SAMPLE);
    ASSERT_EQ(power_pack_filtered_mv(&p), 0);
}

static void test_healthy_pack_never_trips(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    ASSERT_EQ(feed(&p, &now, 8000, 60000), POWER_PACK_OK);
    ASSERT_EQ(power_pack_filtered_mv(&p), 8000);
}

static void test_sustained_low_trips_once_after_min_duration(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 8000, 2000);

    /* Below POWER_PACK_LOW_MV the state is LOW, not TRIPPED, until the minimum
     * duration has passed. The filter takes a few ticks to cross first. */
    ASSERT_EQ(feed(&p, &now, 6000, 1000), POWER_PACK_LOW);
    ASSERT_EQ(feed(&p, &now, 6000, 3000), POWER_PACK_LOW);
    ASSERT_EQ(feed(&p, &now, 6000, 2000), POWER_PACK_TRIPPED);

    /* Latched: later samples, even healthy ones, keep returning TRIPPED, so the
     * caller sees one transition and acts on it once. */
    ASSERT_EQ(feed(&p, &now, 8400, 5000), POWER_PACK_TRIPPED);
}

static void test_trip_time_is_measured_from_the_filtered_crossing(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 8000, 1000);
    const uint32_t start = now;
    power_pack_state_t s = POWER_PACK_OK;
    while (s != POWER_PACK_TRIPPED && now - start < 20000) {
        now += TICK_MS;
        s = power_pack_update(&p, now, 6000);
    }
    ASSERT_EQ(s, POWER_PACK_TRIPPED);
    /* At least the minimum duration, and not much more: the filter lags by a
     * handful of ticks. */
    ASSERT(now - start >= POWER_PACK_MIN_LOW_MS);
    ASSERT(now - start <= POWER_PACK_MIN_LOW_MS + 1000);
}

static void test_motor_start_sag_does_not_trip(void)
{
    /* #991: the pack drops for a moment when the wheels start. Even a deep
     * sag, repeated at every start, must leave the cutoff alone. */
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 7600, 3000);
    for (int i = 0; i < 20; i++) {
        ASSERT(feed(&p, &now, 5200, 300) != POWER_PACK_TRIPPED); /* sag */
        ASSERT(feed(&p, &now, 7600, 3000) != POWER_PACK_TRIPPED);
    }
    ASSERT_EQ(p.state, POWER_PACK_OK);
}

static void test_dip_shorter_than_min_duration_restarts_the_timer(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 8000, 2000);
    /* Low for 4 s, short of 5 s, then recovered past the clear voltage. */
    ASSERT_EQ(feed(&p, &now, 6000, 4000), POWER_PACK_LOW);
    ASSERT_EQ(feed(&p, &now, 8000, 3000), POWER_PACK_OK);
    /* The 4 s already spent must not count toward the next dip. */
    ASSERT_EQ(feed(&p, &now, 6000, 4000), POWER_PACK_LOW);
}

static void test_hysteresis_band_holds_the_low_side(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 8000, 2000);
    feed(&p, &now, 6000, 3000);
    ASSERT_EQ(p.state, POWER_PACK_LOW);

    /* Between low_mv and clear_mv the pack has not recovered: the timer keeps
     * running instead of restarting at every sample that lifts above 6.6 V. */
    const uint32_t mid = (POWER_PACK_LOW_MV + POWER_PACK_CLEAR_MV) / 2;
    ASSERT_EQ(feed(&p, &now, mid, 1500), POWER_PACK_LOW);
    ASSERT_EQ(feed(&p, &now, mid, 1500), POWER_PACK_TRIPPED);
}

static void test_hysteresis_band_does_not_start_the_timer(void)
{
    /* A pack that has been healthy and sits in the band is not "low". */
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 8000, 2000);
    const uint32_t mid = (POWER_PACK_LOW_MV + POWER_PACK_CLEAR_MV) / 2;
    ASSERT_EQ(feed(&p, &now, mid, 60000), POWER_PACK_OK);
}

static void test_first_sample_inside_the_band_is_not_low(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    const uint32_t mid = (POWER_PACK_LOW_MV + POWER_PACK_CLEAR_MV) / 2;
    ASSERT_EQ(feed(&p, &now, mid, 60000), POWER_PACK_OK);
}

static void test_booting_on_a_low_pack_trips_after_min_duration(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    ASSERT_EQ(feed(&p, &now, 6000, 1000), POWER_PACK_LOW);
    ASSERT_EQ(feed(&p, &now, 6000, 5000), POWER_PACK_TRIPPED);
}

static void test_absent_pack_never_trips(void)
{
    /* USB power only: VBAT is 0, the pin reads near 0. */
    power_pack_t p = new_pack();
    uint32_t now = 0;
    ASSERT_EQ(feed(&p, &now, 0, 60000), POWER_PACK_ABSENT);
    ASSERT_EQ(feed(&p, &now, POWER_PACK_ABSENT_MV - 1, 60000), POWER_PACK_ABSENT);
    ASSERT_EQ(power_pack_filtered_mv(&p), 0);
}

static void test_absent_clears_a_running_timer(void)
{
    /* The pack disconnects mid-dip: the dip does not carry over to the next
     * pack that is plugged in. */
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 8000, 2000);
    ASSERT_EQ(feed(&p, &now, 6000, 4000), POWER_PACK_LOW);
    ASSERT_EQ(feed(&p, &now, 0, 1000), POWER_PACK_ABSENT);
    ASSERT_EQ(feed(&p, &now, 6000, 4000), POWER_PACK_LOW);
}

static void test_plugging_in_a_pack_starts_from_its_voltage(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 0, 1000);
    ASSERT_EQ(feed(&p, &now, 8400, 100), POWER_PACK_OK);
    /* No leftover filter state from the zero readings. */
    ASSERT_EQ(power_pack_filtered_mv(&p), 8400);
}

static void test_failed_reads_do_not_add_up_to_a_trip(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 8000, 2000);
    ASSERT_EQ(feed(&p, &now, 6000, 4000), POWER_PACK_LOW);
    power_pack_read_failed(&p);
    /* The failure restarted the timer: one low sample 2 s after the failure is
     * not a trip even though the first low sample was 6 s ago. */
    now += 2000;
    ASSERT_EQ(power_pack_update(&p, now, 6000), POWER_PACK_LOW);
}

static void test_failed_read_leaves_a_trip_latched(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0;
    feed(&p, &now, 6000, 8000);
    ASSERT_EQ(p.state, POWER_PACK_TRIPPED);
    power_pack_read_failed(&p);
    ASSERT_EQ(p.state, POWER_PACK_TRIPPED);
}

static void test_millisecond_wrap(void)
{
    power_pack_t p = new_pack();
    uint32_t now = 0xFFFFFFFFu - 2500u; /* the low spell straddles the wrap */
    feed(&p, &now, 8000, 1000);
    ASSERT_EQ(feed(&p, &now, 6000, 3000), POWER_PACK_LOW);
    ASSERT(now < 3000u); /* the clock wrapped during the spell */
    ASSERT_EQ(feed(&p, &now, 6000, 3000), POWER_PACK_TRIPPED);
}

static void test_state_names(void)
{
    ASSERT(strcmp(power_pack_state_name(POWER_PACK_OK), "ok") == 0);
    ASSERT(strcmp(power_pack_state_name(POWER_PACK_TRIPPED), "tripped") == 0);
    ASSERT(strcmp(power_pack_state_name(POWER_PACK_ABSENT), "absent") == 0);
    ASSERT(strcmp(power_pack_state_name(POWER_PACK_LOW), "low") == 0);
    ASSERT(strcmp(power_pack_state_name(POWER_PACK_NO_SAMPLE), "none") == 0);
    ASSERT(strcmp(power_pack_state_name((power_pack_state_t)99), "?") == 0);
}

static void test_thresholds_are_ordered(void)
{
    /* The band and the absent floor only make sense in this order. */
    ASSERT(POWER_PACK_ABSENT_MV < POWER_PACK_LOW_MV);
    ASSERT(POWER_PACK_LOW_MV < POWER_PACK_CLEAR_MV);
    /* A full 2S pack (8.4 V) is above the clear voltage with room to spare. */
    ASSERT(POWER_PACK_CLEAR_MV < 8400);
}

/* ---- power button ---- */

static power_button_t new_button(void)
{
    power_button_t b;
    power_button_init(&b, POWER_BUTTON_LONG_PRESS_MS);
    return b;
}

/** Feed a constant sample every 50 ms for @p ms; return the first event seen. */
static power_button_event_t press_for(power_button_t *b, uint32_t *now, bool valid, bool pressed,
                                      uint32_t ms)
{
    power_button_event_t first = POWER_BUTTON_NONE;
    for (uint32_t t = 0; t < ms; t += 50) {
        *now += 50;
        const power_button_event_t e = power_button_update(b, *now, valid, pressed);
        if (e != POWER_BUTTON_NONE && first == POWER_BUTTON_NONE) {
            first = e;
        }
    }
    return first;
}

static void test_button_long_press_fires_once(void)
{
    power_button_t b = new_button();
    uint32_t now = 0;
    press_for(&b, &now, true, false, 200); /* released: armed */
    ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS - 100), POWER_BUTTON_NONE);
    ASSERT_EQ(press_for(&b, &now, true, true, 200), POWER_BUTTON_LONG_PRESS);
    /* Still held: no second event. */
    ASSERT_EQ(press_for(&b, &now, true, true, 10000), POWER_BUTTON_NONE);
}

static void test_button_short_press_does_nothing(void)
{
    power_button_t b = new_button();
    uint32_t now = 0;
    press_for(&b, &now, true, false, 200);
    for (int i = 0; i < 5; i++) {
        ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS - 200),
                  POWER_BUTTON_NONE);
        ASSERT_EQ(press_for(&b, &now, true, false, 200), POWER_BUTTON_NONE);
    }
}

static void test_button_held_from_power_on_is_not_a_press(void)
{
    /* The press that switched the robot on is still down when the monitor
     * starts, and may stay down for many seconds. */
    power_button_t b = new_button();
    uint32_t now = 0;
    ASSERT_EQ(press_for(&b, &now, true, true, 30000), POWER_BUTTON_NONE);
    /* Released, then a real long press works. */
    ASSERT_EQ(press_for(&b, &now, true, false, 100), POWER_BUTTON_NONE);
    ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS + 100),
              POWER_BUTTON_LONG_PRESS);
}

static void test_button_failed_reads_never_arm_or_fire(void)
{
    power_button_t b = new_button();
    uint32_t now = 0;
    /* No expander: every read is invalid, whatever `pressed` says. */
    ASSERT_EQ(press_for(&b, &now, false, true, 60000), POWER_BUTTON_NONE);
    ASSERT_EQ(press_for(&b, &now, false, false, 60000), POWER_BUTTON_NONE);
    ASSERT_EQ(b.state, POWER_BUTTON_ARMING);
}

static void test_button_failed_read_cancels_a_press_in_progress(void)
{
    power_button_t b = new_button();
    uint32_t now = 0;
    press_for(&b, &now, true, false, 200);
    ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS - 200), POWER_BUTTON_NONE);
    press_for(&b, &now, false, true, 50); /* one bad read */
    /* The press restarts: 300 ms more is not enough, a full period is. */
    ASSERT_EQ(press_for(&b, &now, true, true, 300), POWER_BUTTON_NONE);
    ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS), POWER_BUTTON_LONG_PRESS);
}

static void test_button_second_press_after_release_fires_again(void)
{
    power_button_t b = new_button();
    uint32_t now = 0;
    press_for(&b, &now, true, false, 200);
    ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS + 100),
              POWER_BUTTON_LONG_PRESS);
    press_for(&b, &now, true, false, 200);
    ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS + 100),
              POWER_BUTTON_LONG_PRESS);
}

static void test_button_millisecond_wrap(void)
{
    power_button_t b = new_button();
    uint32_t now = 0xFFFFFFFFu - 1000u;
    press_for(&b, &now, true, false, 200);
    ASSERT_EQ(press_for(&b, &now, true, true, POWER_BUTTON_LONG_PRESS_MS + 100),
              POWER_BUTTON_LONG_PRESS);
}

typedef void (*test_fn)(void);

int main(void)
{
    static const struct {
        const char *name;
        test_fn fn;
    } k_tests[] = {
        {"divider", test_divider},
        {"no_sample_yet", test_no_sample_yet},
        {"healthy_pack_never_trips", test_healthy_pack_never_trips},
        {"sustained_low_trips_once_after_min_duration",
         test_sustained_low_trips_once_after_min_duration},
        {"trip_time_is_measured_from_the_filtered_crossing",
         test_trip_time_is_measured_from_the_filtered_crossing},
        {"motor_start_sag_does_not_trip", test_motor_start_sag_does_not_trip},
        {"dip_shorter_than_min_duration_restarts_the_timer",
         test_dip_shorter_than_min_duration_restarts_the_timer},
        {"hysteresis_band_holds_the_low_side", test_hysteresis_band_holds_the_low_side},
        {"hysteresis_band_does_not_start_the_timer", test_hysteresis_band_does_not_start_the_timer},
        {"first_sample_inside_the_band_is_not_low", test_first_sample_inside_the_band_is_not_low},
        {"booting_on_a_low_pack_trips_after_min_duration",
         test_booting_on_a_low_pack_trips_after_min_duration},
        {"absent_pack_never_trips", test_absent_pack_never_trips},
        {"absent_clears_a_running_timer", test_absent_clears_a_running_timer},
        {"plugging_in_a_pack_starts_from_its_voltage",
         test_plugging_in_a_pack_starts_from_its_voltage},
        {"failed_reads_do_not_add_up_to_a_trip", test_failed_reads_do_not_add_up_to_a_trip},
        {"failed_read_leaves_a_trip_latched", test_failed_read_leaves_a_trip_latched},
        {"millisecond_wrap", test_millisecond_wrap},
        {"state_names", test_state_names},
        {"thresholds_are_ordered", test_thresholds_are_ordered},
        {"button_long_press_fires_once", test_button_long_press_fires_once},
        {"button_short_press_does_nothing", test_button_short_press_does_nothing},
        {"button_held_from_power_on_is_not_a_press", test_button_held_from_power_on_is_not_a_press},
        {"button_failed_reads_never_arm_or_fire", test_button_failed_reads_never_arm_or_fire},
        {"button_failed_read_cancels_a_press_in_progress",
         test_button_failed_read_cancels_a_press_in_progress},
        {"button_second_press_after_release_fires_again",
         test_button_second_press_after_release_fires_again},
        {"button_millisecond_wrap", test_button_millisecond_wrap},
    };
    const size_t n = sizeof(k_tests) / sizeof(k_tests[0]);
    for (size_t i = 0; i < n; i++) {
        const int before = g_failures;
        k_tests[i].fn();
        printf("[%zu/%zu] %s: %s\n", i + 1, n, k_tests[i].name,
               g_failures == before ? "ok" : "FAIL");
    }
    if (g_failures != 0) {
        printf("%d assertion(s) failed\n", g_failures);
        return 1;
    }
    printf("all %zu tests passed\n", n);
    return 0;
}
