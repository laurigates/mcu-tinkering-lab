/**
 * @file animations.c
 * @brief LED animation engine — breathe, rainbow-chase, sparkle-burst,
 *        nightlight, sync-pulse, and mood-mirror.
 *
 * All pixel writes go through led_ring which applies the ≤60% brightness cap
 * (PRD FR-T21).  animations_render() enforces a minimum 100 ms inter-frame
 * period to prevent strobe effects.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "animations.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "led_ring.h"

static const char *TAG = "animations";

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static anim_state_t s_state;
static uint32_t s_last_render_ms;

/* ------------------------------------------------------------------ */
/* Internal render helpers                                             */
/* ------------------------------------------------------------------ */

/** Breathe: sinusoidal V channel, fixed hue and full saturation. */
static void render_breathe(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - s_state.start_ms;
    /* 4-second cycle */
    float phase = (float)(elapsed % 4000u) / 4000.0f * 2.0f * (float)M_PI;
    uint8_t v = (uint8_t)((sinf(phase) * 0.5f + 0.5f) * 255.0f);

    for (uint32_t i = 0; i < LED_RING_COUNT; i++) {
        led_ring_set_pixel_hsv(i, s_state.hue, 255, v);
    }
    led_ring_show();
}

/** Rainbow-chase: each pixel offset by its index, wheel advances over time. */
static void render_rainbow_chase(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - s_state.start_ms;
    /* Full rotation every 2 seconds */
    uint8_t offset = (uint8_t)((elapsed % 2000u) * 255u / 2000u);

    for (uint32_t i = 0; i < LED_RING_COUNT; i++) {
        uint8_t hue = (uint8_t)(offset + (i * 255u / LED_RING_COUNT));
        led_ring_set_pixel_hsv(i, hue, 255, 200);
    }
    led_ring_show();
}

/**
 * Sparkle-burst: random-looking bright white pixels that decay over 500 ms.
 * Uses a deterministic pseudo-random pattern keyed on elapsed time to avoid
 * pulling in rand() state concerns from ISR context.
 */
static void render_sparkle_burst(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - s_state.start_ms;
    /* Decay from full white to off over 500 ms */
    uint8_t v = (elapsed >= 500u) ? 0u : (uint8_t)(255u - (elapsed * 255u / 500u));

    led_ring_clear();

    /* Light ~half the pixels deterministically */
    uint32_t seed = (now_ms / ANIM_MIN_FRAME_MS) ^ 0xA5A5A5A5u;
    for (uint32_t i = 0; i < LED_RING_COUNT; i++) {
        uint32_t h = seed ^ (i * 2654435761u); /* Knuth multiplicative hash */
        if ((h & 1u) != 0u) {
            led_ring_set_pixel(i, v, v, v);
        }
    }
    led_ring_show();

    /* After decay period, snap back to BREATHE */
    if (elapsed >= 500u) {
        s_state.mode = ANIM_BREATHE;
        s_state.start_ms = now_ms;
    }
}

/** Nightlight: steady warm amber. */
static void render_nightlight(void)
{
    led_ring_fill(255, 100, 20);
    led_ring_show();
}

/**
 * Sync-pulse: single bright white flash for 200 ms, then returns to BREATHE.
 * param carries the phase byte from the leader's SYNC_PULSE packet.
 */
static void render_sync_pulse(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - s_state.start_ms;
    if (elapsed < 200u) {
        led_ring_fill(255, 255, 255);
    } else {
        led_ring_clear();
        s_state.mode = ANIM_BREATHE;
        s_state.start_ms = now_ms;
    }
    led_ring_show();
}

/** Mood-mirror: hue tracks aggregate light level (set externally via hue field). */
static void render_mood_mirror(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - s_state.start_ms;
    /* Gentle breathe at full saturation with the aggregate hue */
    float phase = (float)(elapsed % 3000u) / 3000.0f * 2.0f * (float)M_PI;
    uint8_t v = (uint8_t)((sinf(phase) * 0.4f + 0.6f) * 200.0f);

    for (uint32_t i = 0; i < LED_RING_COUNT; i++) {
        led_ring_set_pixel_hsv(i, s_state.hue, 220, v);
    }
    led_ring_show();
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void animations_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.mode = ANIM_BREATHE;
    s_state.hue = 160; /* Start with cool blue */
    s_last_render_ms = 0;
    ESP_LOGI(TAG, "Animation engine initialized");
}

void animations_set_mode(anim_mode_t mode, uint8_t hue)
{
    if (s_state.mode == mode && s_state.hue == hue) {
        return;
    }
    s_state.mode = mode;
    s_state.hue = hue;
    s_state.start_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    s_state.param = 0;
    ESP_LOGD(TAG, "Mode -> %d hue=%d", (int)mode, (int)hue);
}

const anim_state_t *animations_get_state(void)
{
    return &s_state;
}

void animations_render(uint32_t now_ms)
{
    /* Enforce minimum frame period — no strobing (PRD FR-T21) */
    if ((now_ms - s_last_render_ms) < ANIM_MIN_FRAME_MS) {
        return;
    }
    s_last_render_ms = now_ms;

    switch (s_state.mode) {
        case ANIM_BREATHE:
            render_breathe(now_ms);
            break;
        case ANIM_RAINBOW_CHASE:
            render_rainbow_chase(now_ms);
            break;
        case ANIM_SPARKLE_BURST:
            render_sparkle_burst(now_ms);
            break;
        case ANIM_NIGHTLIGHT:
            render_nightlight();
            break;
        case ANIM_SYNC_PULSE:
            render_sync_pulse(now_ms);
            break;
        case ANIM_MOOD_MIRROR:
            render_mood_mirror(now_ms);
            break;
        default:
            render_breathe(now_ms);
            break;
    }
}
