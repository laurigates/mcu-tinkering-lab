/**
 * @file test_pca9685_phase.c
 * @brief Pins the phase-staggered LEDn encoding in main/pca9685_phase.c.
 *
 * The property that matters is that staggering changes only WHERE in the
 * period a channel's pulse sits, never HOW WIDE it is. Get the wrap arithmetic
 * wrong and a motor silently runs at a different duty cycle than it was told
 * to — which on a bench looks like a mechanical or supply problem, not an
 * arithmetic one.
 *
 * The three cases a bench cannot conveniently stage are all here: the exact
 * 12-bit wrap for a high-numbered channel, the full-on/full-off flag encodings
 * that must carry no phase at all (they are the motor DIRECTION pins), and
 * every channel-by-value combination checked for width preservation.
 */

#include "pca9685_phase.h"

#include <stdio.h>
#include <stdlib.h>

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

/** Decode the 12-bit ON and OFF counts back out of an encoded quartet. */
static void decode(const uint8_t *buf, uint16_t *on, uint16_t *off)
{
    *on = (uint16_t)(buf[0] | ((uint16_t)(buf[1] & 0x0F) << 8));
    *off = (uint16_t)(buf[2] | ((uint16_t)(buf[3] & 0x0F) << 8));
}

/* Channel 0 keeps phase 0, so nothing about the old behaviour changes for it —
 * and the spread is a flat 1/16 of the period. */
static void test_phase_is_a_flat_sixteenth_spread(void)
{
    CHECK(pca9685_phase_on(0) == 0, "ch0 must keep phase 0, got %u", pca9685_phase_on(0));
    for (uint8_t ch = 0; ch < 16; ch++) {
        const uint16_t expect = (uint16_t)(ch * 256u); /* 4096 / 16 */
        CHECK(pca9685_phase_on(ch) == expect, "ch%u phase: expected %u, got %u", ch, expect,
              pca9685_phase_on(ch));
    }
}

/* The load-bearing invariant. A pulse moved in the period must keep its width;
 * if it does not, a motor runs at a duty cycle nobody asked for. */
static void test_on_time_is_preserved_for_every_channel_and_value(void)
{
    const uint16_t values[] = {1, 100, 2055, 2457, 3195, 4094, 4095};

    for (uint8_t ch = 0; ch < 16; ch++) {
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
            uint8_t buf[4];
            pca9685_encode(ch, values[i], buf);

            uint16_t on, off;
            decode(buf, &on, &off);

            /* Modulo the period, because a pulse that starts late finishes in
             * the next period — which is exactly what the chip does. */
            const uint16_t width = (uint16_t)((off + PCA9685_PERIOD_COUNTS - on) & 0x0FFFu);
            CHECK(width == values[i], "ch%u value %u: on=%u off=%u gives width %u", ch, values[i],
                  on, off, width);
            CHECK(on == pca9685_phase_on(ch), "ch%u value %u: on=%u, expected phase %u", ch,
                  values[i], on, pca9685_phase_on(ch));
            CHECK((buf[1] & PCA9685_BIT_FULL) == 0 && (buf[3] & PCA9685_BIT_FULL) == 0,
                  "ch%u value %u: a proportional value must set neither full bit", ch, values[i]);
        }
    }
}

/* Channel 15 at a high value is where the wrap actually happens: phase 3840 +
 * 4095 exceeds the period, so OFF must land BELOW ON. A missing mask would
 * write a 13-bit value into a 12-bit field and truncate the duty cycle. */
static void test_the_twelve_bit_wrap(void)
{
    uint8_t buf[4];
    pca9685_encode(15, 4095, buf);

    uint16_t on, off;
    decode(buf, &on, &off);

    CHECK(on == 3840, "ch15 phase should be 3840, got %u", on);
    CHECK(off == 3839, "ch15 at 4095 should wrap to off=3839, got %u", off);
    CHECK(off < on, "the wrapped case must put OFF below ON");
    CHECK((buf[3] & 0xF0) == 0, "OFF_H must not spill outside the 12-bit field, got 0x%02X",
          buf[3]);
}

/* Full-on and full-off are FLAG bits, not counts — and on this board they are
 * the motor direction pins, which are digital levels rather than a duty cycle.
 * Giving either a phase offset would turn a direction pin into a ~197 Hz
 * square wave. */
static void test_full_on_and_full_off_carry_no_phase(void)
{
    for (uint8_t ch = 0; ch < 16; ch++) {
        uint8_t on_buf[4];
        pca9685_encode(ch, PCA9685_PERIOD_COUNTS, on_buf); /* 4096 = full on */
        CHECK(on_buf[1] == PCA9685_BIT_FULL, "ch%u full-on must set ON_H bit 4, got 0x%02X", ch,
              on_buf[1]);
        CHECK(on_buf[0] == 0, "ch%u full-on must carry no phase in ON_L, got %u", ch, on_buf[0]);
        CHECK((on_buf[3] & PCA9685_BIT_FULL) == 0, "ch%u full-on must not also set FULL_OFF", ch);

        uint8_t off_buf[4];
        pca9685_encode(ch, 0, off_buf);
        CHECK(off_buf[3] == PCA9685_BIT_FULL, "ch%u full-off must set OFF_H bit 4, got 0x%02X", ch,
              off_buf[3]);
        CHECK(off_buf[1] == 0, "ch%u full-off must not set FULL_ON", ch);
    }
}

/* The six motor channels in one write: the two PWM channels get distinct
 * phases, and the four direction channels stay flag-encoded. This is the exact
 * shape motor_controller.c's set_motors() produces when driving forward. */
static void test_a_motor_block_staggers_pwm_but_not_direction(void)
{
    /* ch8 = R_PWM, ch9 = R_IN2, ch10 = R_IN1, ch11 = L_IN1, ch12 = L_IN2,
     * ch13 = L_PWM — forward is IN1 high, IN2 low. */
    const uint16_t values[6] = {3195, 0, 4096, 4096, 0, 3195};
    uint8_t buf[6 * 4];
    for (uint8_t i = 0; i < 6; i++) {
        pca9685_encode((uint8_t)(8 + i), values[i], &buf[i * 4]);
    }

    uint16_t r_on, r_off, l_on, l_off;
    decode(&buf[0], &r_on, &r_off);     /* ch8  */
    decode(&buf[5 * 4], &l_on, &l_off); /* ch13 */

    CHECK(r_on != l_on, "the two motor PWM channels must not share a phase (%u vs %u)", r_on, l_on);
    CHECK(((r_off + PCA9685_PERIOD_COUNTS - r_on) & 0x0FFFu) == 3195, "right PWM width changed");
    CHECK(((l_off + PCA9685_PERIOD_COUNTS - l_on) & 0x0FFFu) == 3195, "left PWM width changed");

    /* Direction channels: 9 and 12 full-off, 10 and 11 full-on. */
    CHECK(buf[1 * 4 + 3] == PCA9685_BIT_FULL, "ch9 (R_IN2) should be full-off");
    CHECK(buf[2 * 4 + 1] == PCA9685_BIT_FULL, "ch10 (R_IN1) should be full-on");
    CHECK(buf[3 * 4 + 1] == PCA9685_BIT_FULL, "ch11 (L_IN1) should be full-on");
    CHECK(buf[4 * 4 + 3] == PCA9685_BIT_FULL, "ch12 (L_IN2) should be full-off");
}

int main(void)
{
    printf("test_pca9685_phase\n");

    test_phase_is_a_flat_sixteenth_spread();
    test_on_time_is_preserved_for_every_channel_and_value();
    test_the_twelve_bit_wrap();
    test_full_on_and_full_off_carry_no_phase();
    test_a_motor_block_staggers_pwm_but_not_direction();

    if (s_failures != 0) {
        printf("FAILED (%d)\n", s_failures);
        return EXIT_FAILURE;
    }
    printf("PASSED\n");
    return EXIT_SUCCESS;
}
