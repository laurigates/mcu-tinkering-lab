/**
 * @file pca9685_phase.h
 * @brief Phase-staggered encoding of the PCA9685's per-channel LEDn registers.
 *
 * The vendored esp-idf-lib driver hardcodes every channel's ON count to 0, so
 * all sixteen outputs rise on the same tick of the PWM period. On this board
 * that is two motor PWMs, two servos and six LED channels switching together
 * ~197 times a second — the worst case for peak current on the shared 5 V
 * rail. The PCA9685 has a per-channel ON register precisely so the edges can
 * be spread instead, and upstream esp-idf-lib exposes no API for it.
 *
 * Split out of i2c_bus.c so the encoding — the part where an arithmetic slip
 * silently changes a motor's duty cycle — is pure C with no ESP-IDF
 * dependency, and is pinned by test/test_pca9685_phase.c on the host.
 *
 * This is headroom, not a fix for an observed fault: the audio distortion that
 * prompted the investigation was resolved by adding bulk capacitance.
 */

#ifndef PCA9685_PHASE_H
#define PCA9685_PHASE_H

#include <stdint.h>

/** First of the PCA9685's per-channel registers (LED0_ON_L). */
#define PCA9685_REG_LED0 0x06
/** Bit 4 of LEDn_ON_H / LEDn_OFF_H — full-on and full-off respectively. */
#define PCA9685_BIT_FULL 0x10
/** Counts per PWM period. */
#define PCA9685_PERIOD_COUNTS 4096u
/** Channels on the device. */
#define PCA9685_CHANNELS 16u

/**
 * @brief Where channel @p ch starts its ON pulse within the period.
 *
 * A flat 1/16-of-a-period spread. Deliberately a pure function of the channel
 * number and nothing else, so a channel's phase never depends on the order
 * writes happen to arrive in — two callers updating overlapping blocks must
 * not be able to move each other's edges.
 */
uint16_t pca9685_phase_on(uint8_t ch);

/**
 * @brief Compose the four LEDn registers for one channel into @p buf.
 *
 * @param ch     Channel number, 0..15 — decides the phase offset.
 * @param value  Driver convention: 0 = full off, >= 4096 = full on, otherwise
 *               an on-time in counts.
 * @param buf    Receives ON_L, ON_H, OFF_L, OFF_H.
 *
 * Full-on and full-off are flag bits rather than counts, so they carry no
 * phase — which is what the motor direction pins want, since they are digital
 * levels and not a duty cycle at all.
 */
void pca9685_encode(uint8_t ch, uint16_t value, uint8_t *buf);

#endif  // PCA9685_PHASE_H
