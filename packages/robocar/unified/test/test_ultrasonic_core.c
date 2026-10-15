/**
 * @file test_ultrasonic_core.c
 * @brief Pins the RCWL-1601 two-phase read sequencing in ultrasonic_core.c.
 *
 * ultrasonic_core.c is compiled unmodified; the bus is a recording fake with
 * one programmable sensor per mux channel. What is pinned:
 *
 *  - The two phases are separate calls with the conversion wait between them,
 *    so no call both triggers and reads (the bus is never held across a
 *    conversion).
 *  - The conversion wait is honoured: no read before ULTRASONIC_CONVERSION_MS.
 *  - Triggers are paced by ULTRASONIC_CONVERSION_MS on the failure path too, so
 *    an absent sensor costs the bus no more than a working one.
 *  - Zero, out-of-range and I2C-error readings are errors, never a clamped
 *    distance.
 *  - Two sensors on two channels run independently.
 *  - The uint32 millisecond wrap at day 49 does not stall the cycle.
 */

#include "ultrasonic_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

/* ---- recording fake bus ------------------------------------------------- */

typedef enum { OP_TRIGGER, OP_READ } op_kind_t;

typedef struct {
    op_kind_t kind;
    uint8_t channel;
} op_t;

typedef struct {
    bool trigger_ok;
    bool read_ok;
    uint32_t um; /* what the next read returns, in micrometres */
} fake_sensor_t;

typedef struct {
    fake_sensor_t sensor[8];
    op_t ops[64];
    size_t n_ops;
} fake_bus_t;

static bool fake_trigger(void *ctx, uint8_t channel)
{
    fake_bus_t *b = ctx;
    b->ops[b->n_ops++] = (op_t){OP_TRIGGER, channel};
    return b->sensor[channel].trigger_ok;
}

static bool fake_read(void *ctx, uint8_t channel, uint8_t raw[RCWL1601_READ_LEN])
{
    fake_bus_t *b = ctx;
    b->ops[b->n_ops++] = (op_t){OP_READ, channel};
    const uint32_t um = b->sensor[channel].um;
    raw[0] = (uint8_t)(um >> 16);
    raw[1] = (uint8_t)(um >> 8);
    raw[2] = (uint8_t)um;
    return b->sensor[channel].read_ok;
}

static fake_bus_t s_fake;
static const ultrasonic_bus_t s_bus = {fake_trigger, fake_read, &s_fake};

static void fake_reset(void)
{
    memset(&s_fake, 0, sizeof(s_fake));
    for (int i = 0; i < 8; i++) {
        s_fake.sensor[i] = (fake_sensor_t){true, true, 500000u}; /* 50 cm */
    }
}

/* ---- decode ------------------------------------------------------------- */

static void test_decode_reads_micrometres_big_endian(void)
{
    /* 0x01 0x86 0xA0 = 100000 um = 10 cm. Byte order matters: read little-
     * endian it would be 0xA08601 um, far out of range. */
    const uint8_t raw[3] = {0x01, 0x86, 0xA0};
    uint16_t cm = 0;
    CHECK(ultrasonic_core_decode(raw, &cm) == ULTRASONIC_READY, "10 cm should decode");
    CHECK(cm == 10u, "expected 10 cm, got %u", (unsigned)cm);
}

static void test_decode_rounds_to_the_nearest_cm(void)
{
    const uint8_t up[3] = {0x00, 0xC7, 0x38};   /* 51000 um = 5.1 cm */
    const uint8_t half[3] = {0x00, 0xD6, 0xD8}; /* 55000 um = 5.5 cm */
    uint16_t cm = 0;
    CHECK(ultrasonic_core_decode(up, &cm) == ULTRASONIC_READY && cm == 5u, "5.1 cm -> 5, got %u",
          (unsigned)cm);
    CHECK(ultrasonic_core_decode(half, &cm) == ULTRASONIC_READY && cm == 6u, "5.5 cm -> 6, got %u",
          (unsigned)cm);
}

static void test_decode_rejects_zero_and_out_of_range(void)
{
    const uint8_t zero[3] = {0, 0, 0};
    uint16_t cm = 123;
    CHECK(ultrasonic_core_decode(zero, &cm) == ULTRASONIC_ERROR, "a zero reading is an error");
    CHECK(cm == ULTRASONIC_DIST_ERROR, "zero must not leave a distance, got %u", (unsigned)cm);

    /* 1 cm: below the 2 cm floor. */
    const uint8_t near[3] = {0x00, 0x27, 0x10}; /* 10000 um */
    cm = 123;
    CHECK(ultrasonic_core_decode(near, &cm) == ULTRASONIC_ERROR, "1 cm is below range");
    CHECK(cm == ULTRASONIC_DIST_ERROR, "below range must not clamp, got %u", (unsigned)cm);

    /* 401 cm: past the 400 cm ceiling. Must be an error, not 400 — a clamped
     * value reads as a real reading. */
    const uint32_t far_um = 4010000u;
    const uint8_t far[3] = {(uint8_t)(far_um >> 16), (uint8_t)(far_um >> 8), (uint8_t)far_um};
    cm = 123;
    CHECK(ultrasonic_core_decode(far, &cm) == ULTRASONIC_ERROR, "401 cm is beyond range");
    CHECK(cm == ULTRASONIC_DIST_ERROR, "beyond range must not clamp, got %u", (unsigned)cm);

    /* The bounds themselves are readings. */
    const uint32_t max_um = ULTRASONIC_MAX_RANGE_CM * 10000u;
    const uint8_t edge[3] = {(uint8_t)(max_um >> 16), (uint8_t)(max_um >> 8), (uint8_t)max_um};
    CHECK(ultrasonic_core_decode(edge, &cm) == ULTRASONIC_READY && cm == ULTRASONIC_MAX_RANGE_CM,
          "400 cm is in range, got %u", (unsigned)cm);
}

/* ---- sequencing --------------------------------------------------------- */

static void test_trigger_and_read_are_separate_calls(void)
{
    fake_reset();
    ultrasonic_core_t s;
    ultrasonic_core_reset(&s);
    uint16_t cm = 0;

    /* Phase 1: one trigger, nothing else. */
    CHECK(ultrasonic_core_step(&s, 3, 1000, &s_bus, &cm) == ULTRASONIC_PENDING,
          "phase 1 is pending");
    CHECK(s_fake.n_ops == 1 && s_fake.ops[0].kind == OP_TRIGGER && s_fake.ops[0].channel == 3,
          "phase 1 must be exactly one trigger on ch3 (ops=%zu)", s_fake.n_ops);

    /* Inside the conversion wait: no bus traffic at all. */
    for (uint32_t t = 1000; t < 1000 + ULTRASONIC_CONVERSION_MS; t += 33) {
        CHECK(ultrasonic_core_step(&s, 3, t, &s_bus, &cm) == ULTRASONIC_PENDING,
              "pending at +%u ms", (unsigned)(t - 1000));
    }
    CHECK(s_fake.n_ops == 1, "no bus op during the conversion wait, got %zu", s_fake.n_ops);

    /* Phase 2 at the end of the wait: one read, nothing else. */
    CHECK(ultrasonic_core_step(&s, 3, 1000 + ULTRASONIC_CONVERSION_MS, &s_bus, &cm) ==
              ULTRASONIC_READY,
          "phase 2 returns the reading");
    CHECK(cm == 50u, "expected 50 cm, got %u", (unsigned)cm);
    CHECK(s_fake.n_ops == 2 && s_fake.ops[1].kind == OP_READ && s_fake.ops[1].channel == 3,
          "phase 2 must be exactly one read on ch3 (ops=%zu)", s_fake.n_ops);

    /* The next cycle starts on the following call — a trigger, not a read. */
    CHECK(ultrasonic_core_step(&s, 3, 1000 + ULTRASONIC_CONVERSION_MS + 33, &s_bus, &cm) ==
              ULTRASONIC_PENDING,
          "a new cycle starts pending");
    CHECK(s_fake.n_ops == 3 && s_fake.ops[2].kind == OP_TRIGGER, "the next op is a trigger");
}

static void test_no_call_does_more_than_one_bus_op(void)
{
    fake_reset();
    ultrasonic_core_t s;
    ultrasonic_core_reset(&s);
    uint16_t cm = 0;
    for (uint32_t t = 0; t < 2000; t += 33) {
        const size_t before = s_fake.n_ops;
        (void)ultrasonic_core_step(&s, 4, t, &s_bus, &cm);
        CHECK(s_fake.n_ops - before <= 1, "at %u ms a step did %zu bus ops", (unsigned)t,
              s_fake.n_ops - before);
    }
}

static void test_a_failed_read_is_an_error_and_starts_over(void)
{
    fake_reset();
    s_fake.sensor[3].read_ok = false;
    ultrasonic_core_t s;
    ultrasonic_core_reset(&s);
    uint16_t cm = 0;

    (void)ultrasonic_core_step(&s, 3, 0, &s_bus, &cm);
    cm = 77;
    CHECK(ultrasonic_core_step(&s, 3, ULTRASONIC_CONVERSION_MS, &s_bus, &cm) == ULTRASONIC_ERROR,
          "a failed read is an error");
    CHECK(cm == ULTRASONIC_DIST_ERROR, "an error leaves no distance, got %u", (unsigned)cm);

    s_fake.sensor[3].read_ok = true;
    (void)ultrasonic_core_step(&s, 3, ULTRASONIC_CONVERSION_MS + 33, &s_bus, &cm);
    CHECK(s_fake.ops[s_fake.n_ops - 1].kind == OP_TRIGGER, "after an error the cycle re-triggers");
}

static void test_a_zero_reading_is_an_error(void)
{
    fake_reset();
    s_fake.sensor[3].um = 0;
    ultrasonic_core_t s;
    ultrasonic_core_reset(&s);
    uint16_t cm = 0;
    (void)ultrasonic_core_step(&s, 3, 0, &s_bus, &cm);
    CHECK(ultrasonic_core_step(&s, 3, ULTRASONIC_CONVERSION_MS, &s_bus, &cm) == ULTRASONIC_ERROR,
          "a zero reading is an error, not 0 cm");
}

static void test_a_failed_trigger_is_paced_like_a_good_one(void)
{
    fake_reset();
    s_fake.sensor[4].trigger_ok = false; /* sensor absent: NACK */
    ultrasonic_core_t s;
    ultrasonic_core_reset(&s);
    uint16_t cm = 0;

    CHECK(ultrasonic_core_step(&s, 4, 0, &s_bus, &cm) == ULTRASONIC_ERROR,
          "a failed trigger is reported at once");
    CHECK(cm == ULTRASONIC_DIST_ERROR, "and leaves no distance");

    /* Calling at the executor's 30 Hz must not retry before the wait is up. */
    size_t triggers = 0;
    for (uint32_t t = 33; t <= 1200; t += 33) {
        (void)ultrasonic_core_step(&s, 4, t, &s_bus, &cm);
    }
    for (size_t i = 0; i < s_fake.n_ops; i++) {
        triggers += (s_fake.ops[i].kind == OP_TRIGGER);
    }
    /* 0..1200 ms at one attempt per >= 120 ms is at most 11 attempts; retrying
     * on every 33 ms call would be 37. */
    CHECK(triggers <= 1200u / ULTRASONIC_CONVERSION_MS + 1u,
          "an absent sensor was retried %zu times in 1.2 s", triggers);
    CHECK(triggers >= 1200u / (ULTRASONIC_CONVERSION_MS + 33u),
          "but it must keep retrying, got %zu", triggers);
}

static void test_two_sensors_run_independently(void)
{
    fake_reset();
    s_fake.sensor[3].um = 300000u;  /* front: 30 cm */
    s_fake.sensor[4].um = 1200000u; /* rear: 120 cm */
    ultrasonic_core_t front, rear;
    ultrasonic_core_reset(&front);
    ultrasonic_core_reset(&rear);

    uint16_t got_front = 0, got_rear = 0;
    int ready_front = 0, ready_rear = 0;
    for (uint32_t t = 0; t < 1000; t += 33) {
        uint16_t cm = 0;
        if (ultrasonic_core_step(&front, 3, t, &s_bus, &cm) == ULTRASONIC_READY) {
            got_front = cm;
            ready_front++;
        }
        if (ultrasonic_core_step(&rear, 4, t, &s_bus, &cm) == ULTRASONIC_READY) {
            got_rear = cm;
            ready_rear++;
        }
    }
    CHECK(got_front == 30u, "front read its own channel, got %u", (unsigned)got_front);
    CHECK(got_rear == 120u, "rear read its own channel, got %u", (unsigned)got_rear);
    /* ~1 s at one reading per ~150 ms. */
    CHECK(ready_front >= 5 && ready_rear >= 5, "both must keep reading (%d, %d)", ready_front,
          ready_rear);
    for (size_t i = 0; i < s_fake.n_ops; i++) {
        CHECK(s_fake.ops[i].channel == 3 || s_fake.ops[i].channel == 4, "op on stray channel %u",
              (unsigned)s_fake.ops[i].channel);
    }
}

static void test_the_millisecond_wrap_does_not_stall_the_cycle(void)
{
    fake_reset();
    ultrasonic_core_t s;
    ultrasonic_core_reset(&s);
    uint16_t cm = 0;
    int ready = 0;
    /* Start 200 ms before the wrap and run 1 s across it. */
    uint32_t t = 0xFFFFFFFFu - 200u;
    for (int i = 0; i < 30; i++, t += 33u) {
        if (ultrasonic_core_step(&s, 3, t, &s_bus, &cm) == ULTRASONIC_READY) {
            ready++;
        }
    }
    CHECK(ready >= 5, "readings must continue across the wrap, got %d", ready);
}

static void test_null_arguments_are_errors(void)
{
    ultrasonic_core_t s;
    ultrasonic_core_reset(&s);
    uint16_t cm = 0;
    CHECK(ultrasonic_core_step(&s, 3, 0, NULL, &cm) == ULTRASONIC_ERROR, "NULL bus");
    CHECK(ultrasonic_core_step(NULL, 3, 0, &s_bus, &cm) == ULTRASONIC_ERROR, "NULL state");
    CHECK(ultrasonic_core_step(&s, 3, 0, &s_bus, NULL) == ULTRASONIC_ERROR, "NULL out");
}

int main(void)
{
    printf("test_ultrasonic_core\n");

    test_decode_reads_micrometres_big_endian();
    test_decode_rounds_to_the_nearest_cm();
    test_decode_rejects_zero_and_out_of_range();
    test_trigger_and_read_are_separate_calls();
    test_no_call_does_more_than_one_bus_op();
    test_a_failed_read_is_an_error_and_starts_over();
    test_a_zero_reading_is_an_error();
    test_a_failed_trigger_is_paced_like_a_good_one();
    test_two_sensors_run_independently();
    test_the_millisecond_wrap_does_not_stall_the_cycle();
    test_null_arguments_are_errors();

    if (s_failures != 0) {
        printf("FAILED (%d)\n", s_failures);
        return EXIT_FAILURE;
    }
    printf("PASSED\n");
    return EXIT_SUCCESS;
}
