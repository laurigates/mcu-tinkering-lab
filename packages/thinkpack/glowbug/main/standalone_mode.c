/**
 * @file standalone_mode.c
 * @brief Sensor-driven animation selector — LDR + IMU → animation mode.
 *
 * State machine (per 33 ms tick):
 *  1. Poll LDR:
 *     - < 800  → NIGHTLIGHT
 *     - > 3200 → RAINBOW_CHASE
 *     - else   → keep previous
 *  2. Read accel. If vector delta > 1.5 g → SPARKLE_BURST (hold 500 ms).
 *  3. Otherwise: tilt angle → hue for BREATHE.
 *  4. Button press cycles a forced-mode override that wins over auto.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "standalone_mode.h"

#include "animations.h"
#include "esp_log.h"
#include "imu.h"
#include "light_sensor.h"

static const char *TAG = "standalone";

/* LDR thresholds (12-bit ADC, 0=dark 4095=bright) */
#define LDR_NIGHTLIGHT_THRESHOLD 800
#define LDR_RAINBOW_THRESHOLD 3200

/* How long to hold SPARKLE_BURST before returning to sensor-driven mode */
#define SPARKLE_HOLD_MS 500u

/* Override cycle steps */
typedef enum {
    OVERRIDE_AUTO = 0,
    OVERRIDE_BREATHE,
    OVERRIDE_RAINBOW,
    OVERRIDE_SPARKLE,
    OVERRIDE_COUNT,
} override_t;

static override_t s_override = OVERRIDE_AUTO;
static anim_mode_t s_auto_mode = ANIM_BREATHE;
static uint32_t s_sparkle_start_ms = 0;
static bool s_in_sparkle = false;

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void standalone_mode_init(void)
{
    s_override = OVERRIDE_AUTO;
    s_auto_mode = ANIM_BREATHE;
    s_in_sparkle = false;
    ESP_LOGI(TAG, "Standalone mode initialized");
}

void standalone_mode_tick(uint32_t now_ms)
{
    /* If a forced override is active, just apply it and return */
    if (s_override != OVERRIDE_AUTO) {
        switch (s_override) {
            case OVERRIDE_BREATHE:
                animations_set_mode(ANIM_BREATHE, imu_tilt_to_hue());
                break;
            case OVERRIDE_RAINBOW:
                animations_set_mode(ANIM_RAINBOW_CHASE, 0);
                break;
            case OVERRIDE_SPARKLE:
                animations_set_mode(ANIM_SPARKLE_BURST, 0);
                break;
            default:
                break;
        }
        return;
    }

    /* Clear sparkle hold if it has expired */
    if (s_in_sparkle && (now_ms - s_sparkle_start_ms) >= SPARKLE_HOLD_MS) {
        s_in_sparkle = false;
    }

    /* Step 2: shake check (takes priority over LDR auto-select) */
    if (!s_in_sparkle && imu_detect_shake()) {
        s_in_sparkle = true;
        s_sparkle_start_ms = now_ms;
        animations_set_mode(ANIM_SPARKLE_BURST, 0);
        ESP_LOGD(TAG, "Shake detected — SPARKLE_BURST");
        return;
    }

    if (s_in_sparkle) {
        /* Hold sparkle for the configured duration */
        animations_set_mode(ANIM_SPARKLE_BURST, 0);
        return;
    }

    /* Step 1: LDR-driven mode */
    int lux = 0;
    if (light_sensor_read(&lux) == ESP_OK) {
        if (lux < LDR_NIGHTLIGHT_THRESHOLD) {
            s_auto_mode = ANIM_NIGHTLIGHT;
        } else if (lux > LDR_RAINBOW_THRESHOLD) {
            s_auto_mode = ANIM_RAINBOW_CHASE;
        }
        /* else: keep s_auto_mode unchanged */
    }

    /* Step 3: tilt → hue for BREATHE; nightlight/rainbow don't use hue */
    if (s_auto_mode == ANIM_BREATHE || s_auto_mode == ANIM_MOOD_MIRROR) {
        uint8_t hue = imu_tilt_to_hue();
        animations_set_mode(s_auto_mode, hue);
    } else {
        animations_set_mode(s_auto_mode, 0);
    }
}

void standalone_mode_cycle_override(void)
{
    s_override = (override_t)((int)s_override + 1);
    if (s_override >= OVERRIDE_COUNT) {
        s_override = OVERRIDE_AUTO;
    }

    const char *names[] = {"AUTO", "BREATHE", "RAINBOW_CHASE", "SPARKLE_BURST"};
    ESP_LOGI(TAG, "Override -> %s", names[s_override]);
}
