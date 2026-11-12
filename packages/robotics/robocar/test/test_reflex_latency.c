/**
 * @file test_reflex_latency.c
 * @brief Worst-case time from an obstacle appearing to the reflex braking (issue #791).
 *
 * The reactive controller's own host suite uses the ultrasonic stub, which
 * returns a reading on every poll, so it cannot see how long a reading takes.
 * This suite links the real executor (reactive_controller.c) against the real
 * two-phase read (ultrasonic_core.c) instead: ultrasonic_poll() below is the
 * target driver's one-line wrapper around ultrasonic_core_step(), with a fake
 * bus in place of the I2C transactions. The clock is the executor's host tick,
 * REACTIVE_LOOP_PERIOD_MS per tick, so the sequencing is the one the firmware
 * runs at its nominal loop period.
 *
 * The fake sensor ranges at the trigger: the trigger latches the world's
 * distance and the read, ULTRASONIC_CONVERSION_MS later, returns it. That is
 * the conservative model (an echo comes back within ~25 ms of the burst, so a
 * real reading is never older than its trigger).
 *
 * Every expected value is computed from ULTRASONIC_CONVERSION_MS and
 * REACTIVE_LOOP_PERIOD_MS, so the test tracks the conversion wait when the
 * bench measurement replaces the 120 ms placeholder.
 */

#include "goal_state.h"
#include "motor_controller.h"
#include "reactive_controller.h"
#include "ultrasonic.h"
#include "ultrasonic_core.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* From motor_controller_stub.c */
void motor_stub_reset(void);
int motor_stub_get_last_call(void);
#define MOTOR_CALL_BRAKE 2
#define MOTOR_CALL_DRIVE 3

/* From reactive_controller.c (host-test build) */
void reactive_controller_tick_for_test(void);

/* From servo_controller_stub.c */
void servo_stub_reset(bool initialized);

/* =========================================================================
 * Test harness
 * ========================================================================= */

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

/* =========================================================================
 * ultrasonic.h over the real ultrasonic_core.c, with a fake bus
 * ========================================================================= */

/** What is actually in front of each sensor, in cm. */
static uint16_t s_world_cm[ULTRASONIC_COUNT];
/** The distance each sensor's last trigger ranged. */
static uint16_t s_latched_cm[ULTRASONIC_COUNT];
static ultrasonic_core_t s_core[ULTRASONIC_COUNT];
static bool s_initialized = false;

static bool fake_trigger(void *ctx, uint8_t channel)
{
    (void)ctx;
    s_latched_cm[channel] = s_world_cm[channel];
    return true;
}

static bool fake_read(void *ctx, uint8_t channel, uint8_t raw[RCWL1601_READ_LEN])
{
    (void)ctx;
    const uint32_t um = (uint32_t)s_latched_cm[channel] * 10000u;
    raw[0] = (uint8_t)(um >> 16);
    raw[1] = (uint8_t)(um >> 8);
    raw[2] = (uint8_t)um;
    return true;
}

static const ultrasonic_bus_t k_fake_bus = {
    .trigger = fake_trigger,
    .read = fake_read,
    .ctx = NULL,
};

void ultrasonic_test_set_distance(ultrasonic_id_t id, uint16_t cm)
{
    if ((unsigned)id < ULTRASONIC_COUNT) {
        s_world_cm[id] = cm;
    }
}

esp_err_t ultrasonic_init(void)
{
    for (int i = 0; i < (int)ULTRASONIC_COUNT; i++) {
        ultrasonic_core_reset(&s_core[i]);
    }
    s_initialized = true;
    return ESP_OK;
}

/* Mirrors the target ultrasonic_poll(); the channel is the sensor id here. */
ultrasonic_result_t ultrasonic_poll(ultrasonic_id_t id, uint32_t now_ms, uint16_t *distance_cm)
{
    if (distance_cm == NULL) {
        return ULTRASONIC_ERROR;
    }
    if (!s_initialized || (unsigned)id >= ULTRASONIC_COUNT) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ULTRASONIC_ERROR;
    }
    return ultrasonic_core_step(&s_core[id], (uint8_t)id, now_ms, &k_fake_bus, distance_cm);
}

uint8_t ultrasonic_channel(ultrasonic_id_t id)
{
    return (uint8_t)id;
}

const char *ultrasonic_name(ultrasonic_id_t id)
{
    return (id == ULTRASONIC_FRONT) ? "front" : "rear";
}

esp_err_t ultrasonic_deinit(void)
{
    s_initialized = false;
    return ESP_OK;
}

/* =========================================================================
 * Fixtures
 * ========================================================================= */

/** Loop ticks the read waits after its trigger: the first tick at or past it. */
#define CONVERSION_TICKS \
    ((ULTRASONIC_CONVERSION_MS + REACTIVE_LOOP_PERIOD_MS - 1u) / REACTIVE_LOOP_PERIOD_MS)

/** Ticks per reading: the read tick, then the next trigger on the tick after. */
#define READING_TICKS (CONVERSION_TICKS + 1u)

/**
 * The bound reactive_controller.h states. Worst case, the obstacle appears
 * just after a trigger ranged the clear path: that cycle's read (CONVERSION_TICKS
 * later) is still clear, the next trigger is one tick after it, and its read
 * CONVERSION_TICKS after that is the one reading that trips the reflex.
 */
#define WORST_CASE_MS ((2u * CONVERSION_TICKS + 1u) * REACTIVE_LOOP_PERIOD_MS)

#define CLEAR_CM 100u
#define NEAR_CM 5u

/* Long enough to fill the smoothing filter with clear readings several times. */
#define WARMUP_TICKS (4u * ULTRASONIC_SMOOTH_N * READING_TICKS)

static const goal_t k_drive_ahead = {
    .kind = GOAL_KIND_DRIVE,
    .params.drive = {.heading_deg = 0, .distance_cm = 200, .speed_pct = 80},
};

static void setup(void)
{
    reactive_controller_stop(); /* no-op on the first call */
    goal_state_init();
    goal_state_force_stop();
    motor_stub_reset();
    servo_stub_reset(false);
    ultrasonic_test_set_distance(ULTRASONIC_FRONT, CLEAR_CM);
    ultrasonic_test_set_distance(ULTRASONIC_REAR, CLEAR_CM);
    reactive_controller_init(); /* resets the executor clock and ultrasonic_init() */
    /* A long TTL so the goal outlives every scenario below. */
    ASSERT(goal_state_write(&k_drive_ahead, 60000) == ESP_OK);
}

static bool reflex_active(void)
{
    reactive_telemetry_t t = {0};
    ASSERT(reactive_controller_get_telemetry(&t) == ESP_OK);
    return t.reflex_active;
}

static void run_ticks(unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        reactive_controller_tick_for_test();
    }
}

/**
 * Drive clear for @p phase extra ticks, put the obstacle in front, and return
 * the time to the brake: the obstacle is taken to have appeared just after the
 * last clear tick, so the latency counts every tick from then up to and
 * including the one that brakes. Returns 0 if it never brakes.
 */
static uint32_t latency_ms_at_phase(unsigned phase)
{
    setup();
    run_ticks(WARMUP_TICKS + phase);
    ASSERT(!reflex_active());
    ASSERT(motor_stub_get_last_call() == MOTOR_CALL_DRIVE);

    ultrasonic_test_set_distance(ULTRASONIC_FRONT, NEAR_CM);
    for (unsigned n = 1; n <= 10u * ULTRASONIC_SMOOTH_N * READING_TICKS; n++) {
        reactive_controller_tick_for_test();
        if (reflex_active()) {
            ASSERT(motor_stub_get_last_call() == MOTOR_CALL_BRAKE);
            return n * REACTIVE_LOOP_PERIOD_MS;
        }
    }
    return 0;
}

/* =========================================================================
 * Tests
 * ========================================================================= */

/* The derived timing the bound rests on: one reading every READING_TICKS. */
static void test_reading_period_follows_the_conversion_wait(void)
{
    setup();
    unsigned readings = 0;
    unsigned first = 0, last = 0;
    for (unsigned tick = 0; tick < 20u * READING_TICKS; tick++) {
        /* The rear reading reaches telemetry unsmoothed, so each new value
         * there is one completed read. Alternate the world so every read
         * differs from the one before. */
        reactive_telemetry_t before = {0};
        ASSERT(reactive_controller_get_telemetry(&before) == ESP_OK);
        ultrasonic_test_set_distance(ULTRASONIC_REAR, (uint16_t)(10u + (readings % 2u)));
        reactive_controller_tick_for_test();
        reactive_telemetry_t after = {0};
        ASSERT(reactive_controller_get_telemetry(&after) == ESP_OK);
        if (after.distance_rear_cm != ULTRASONIC_DIST_ERROR &&
            after.distance_rear_cm != before.distance_rear_cm) {
            if (readings == 0) {
                first = tick;
            }
            last = tick;
            readings++;
        }
    }
    ASSERT(readings >= 3);
    ASSERT((last - first) == (readings - 1u) * READING_TICKS);
}

/* The headline number: over every phase of the obstacle against the read cycle,
 * the slowest brake is exactly the stated bound, and none is slower. */
static void test_worst_case_obstacle_to_brake(void)
{
    uint32_t worst = 0;
    for (unsigned phase = 0; phase < 2u * READING_TICKS; phase++) {
        const uint32_t ms = latency_ms_at_phase(phase);
        ASSERT(ms != 0);
        ASSERT(ms <= WORST_CASE_MS);
        if (ms > worst) {
            worst = ms;
        }
    }
    printf("     worst case %u ms (conversion %u ms, loop %u ms, bound %u ms)\n", (unsigned)worst,
           (unsigned)ULTRASONIC_CONVERSION_MS, (unsigned)REACTIVE_LOOP_PERIOD_MS,
           (unsigned)WORST_CASE_MS);
    ASSERT(worst == WORST_CASE_MS);
    ASSERT(worst == REACTIVE_REFLEX_WORST_CASE_MS);
}

/* The price of tripping on one reading: a single spurious short reading brakes
 * for one reading, and the next clear one releases. */
static void test_spurious_short_reading_releases_on_the_next(void)
{
    setup();
    run_ticks(WARMUP_TICKS);
    ultrasonic_test_set_distance(ULTRASONIC_FRONT, NEAR_CM);

    unsigned n = 0;
    while (!reflex_active()) {
        reactive_controller_tick_for_test();
        ASSERT(++n <= 2u * READING_TICKS);
    }
    /* The very tick it trips, the path clears again: the next trigger sees it. */
    ultrasonic_test_set_distance(ULTRASONIC_FRONT, CLEAR_CM);
    unsigned held = 0;
    while (reflex_active()) {
        reactive_controller_tick_for_test();
        ASSERT(++held <= READING_TICKS + CONVERSION_TICKS);
    }
    ASSERT(motor_stub_get_last_call() == MOTOR_CALL_DRIVE);
}

/* Release still goes through the running mean: a close reading that lingers in
 * the filter holds the brake even after one reading above the threshold. */
static void test_release_needs_the_mean_above_threshold(void)
{
    setup();
    run_ticks(WARMUP_TICKS);
    ultrasonic_test_set_distance(ULTRASONIC_FRONT, NEAR_CM);
    run_ticks(ULTRASONIC_SMOOTH_N * READING_TICKS + READING_TICKS); /* filter full of 5 cm */
    ASSERT(reflex_active());

    /* Just above the threshold: the latest reading is clear, the mean is not. */
    ultrasonic_test_set_distance(ULTRASONIC_FRONT, STOP_THRESHOLD_CM + 1u);
    run_ticks(2u * READING_TICKS);
    ASSERT(reflex_active());
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== reflex latency host tests ===\n\n");

    test_run("reading_period_follows_the_conversion_wait",
             test_reading_period_follows_the_conversion_wait);
    test_run("worst_case_obstacle_to_brake", test_worst_case_obstacle_to_brake);
    test_run("spurious_short_reading_releases_on_the_next",
             test_spurious_short_reading_releases_on_the_next);
    test_run("release_needs_the_mean_above_threshold", test_release_needs_the_mean_above_threshold);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
