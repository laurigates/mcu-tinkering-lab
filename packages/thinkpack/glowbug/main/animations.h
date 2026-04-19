/**
 * @file animations.h
 * @brief LED animation engine for the glowbug box.
 *
 * All animations write through led_ring, which enforces the ≤60% brightness
 * cap (PRD FR-T21).  The minimum frame period is 100 ms; animations_render()
 * enforces this internally and ignores calls that arrive too early.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#ifndef ANIMATIONS_H
#define ANIMATIONS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Available animation modes.
 */
typedef enum {
    ANIM_BREATHE,       /**< Sinusoidal brightness modulation, hue from tilt */
    ANIM_RAINBOW_CHASE, /**< Moving rainbow around the ring */
    ANIM_SPARKLE_BURST, /**< White sparkles that decay */
    ANIM_NIGHTLIGHT,    /**< Steady amber (R=255, G=100, B=20) */
    ANIM_SYNC_PULSE,    /**< Single flash synchronised with mesh leader pulse */
    ANIM_MOOD_MIRROR,   /**< Hue follows aggregate group light-level */
} anim_mode_t;

/**
 * @brief Animation state block (opaque to callers — managed by animations.c).
 */
typedef struct {
    anim_mode_t mode;  /**< Current animation mode */
    uint8_t hue;       /**< Base hue (0-255) where mode uses it */
    uint32_t start_ms; /**< Timestamp when the current mode was entered */
    uint8_t param;     /**< Mode-specific parameter (sparkle decay, phase…) */
} anim_state_t;

/** Minimum frame period enforced in animations_render() (PRD FR-T21). */
#define ANIM_MIN_FRAME_MS 100u

/**
 * @brief Initialise the animation engine and reset state to ANIM_BREATHE.
 */
void animations_init(void);

/**
 * @brief Switch to a new animation mode.
 *
 * Resets start_ms and param; the next animations_render() call begins the
 * new animation from frame 0.
 *
 * @param mode  Animation to activate.
 * @param hue   Base hue (used by BREATHE, MOOD_MIRROR, SYNC_PULSE).
 */
void animations_set_mode(anim_mode_t mode, uint8_t hue);

/**
 * @brief Retrieve the current animation state (read-only snapshot).
 *
 * @return Pointer to the internal state struct.  Valid until the next
 *         animations_set_mode() call.
 */
const anim_state_t *animations_get_state(void);

/**
 * @brief Render one animation frame into the LED ring.
 *
 * Enforces ANIM_MIN_FRAME_MS: if less than 100 ms have elapsed since the
 * last render the call is a no-op.  Call at ~30 Hz from the animation task.
 *
 * @param now_ms  Current monotonic time in milliseconds.
 */
void animations_render(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* ANIMATIONS_H */
