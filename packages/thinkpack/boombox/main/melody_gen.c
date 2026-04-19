/**
 * @file melody_gen.c
 * @brief Procedural melody patterns: MARCH, WALTZ, PENTATONIC, SILENCE.
 */

#include "melody_gen.h"

#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"

static const char *TAG = "melody_gen";

static volatile melody_pattern_t s_pattern = MELODY_MARCH;

/* ------------------------------------------------------------------ */
/* Pattern tables                                                      */
/* ------------------------------------------------------------------ */

/* 4/4 March in C major — 8 steps, downbeat on index % 4 == 0 */
static const note_t s_march_notes[] = {
    NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_G4, NOTE_E4, NOTE_C4, NOTE_REST,
};
#define MARCH_LEN 8

/* 3/4 Waltz in G major — 6 steps, downbeat on index % 3 == 0 */
static const note_t s_waltz_notes[] = {
    NOTE_G4, NOTE_B4, NOTE_D5, NOTE_G4, NOTE_B4, NOTE_D5,
};
#define WALTZ_LEN 6

/* Pentatonic pool: C4 D4 E4 G4 A4 */
static const note_t s_pentatonic_pool[] = {
    NOTE_C4, NOTE_D4, NOTE_E4, NOTE_G4, NOTE_A4,
};
#define PENTATONIC_POOL_LEN 5

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void melody_gen_set_pattern(melody_pattern_t pattern)
{
    if (pattern >= MELODY_COUNT) {
        ESP_LOGW(TAG, "Invalid pattern %d, ignoring", (int)pattern);
        return;
    }
    s_pattern = pattern;
    ESP_LOGI(TAG, "Pattern -> %d", (int)pattern);
}

melody_pattern_t melody_gen_get_pattern(void)
{
    return s_pattern;
}

melody_step_t melody_gen_next(uint32_t beat_index)
{
    melody_step_t step = {.note = NOTE_REST, .hold_fraction = 1, .is_downbeat = false};

    switch (s_pattern) {
        case MELODY_SILENCE:
            /* Always silent */
            break;

        case MELODY_MARCH: {
            uint32_t idx = beat_index % MARCH_LEN;
            step.note = s_march_notes[idx];
            step.hold_fraction = (step.note == NOTE_REST) ? 1 : 2; /* half-beat gap */
            step.is_downbeat = ((beat_index % 4) == 0);
            break;
        }

        case MELODY_WALTZ: {
            uint32_t idx = beat_index % WALTZ_LEN;
            step.note = s_waltz_notes[idx];
            step.hold_fraction = (beat_index % 3 == 0) ? 1 : 2;
            step.is_downbeat = ((beat_index % 3) == 0);
            break;
        }

        case MELODY_PENTATONIC: {
            uint32_t idx = (uint32_t)rand() % PENTATONIC_POOL_LEN;
            step.note = s_pentatonic_pool[idx];
            step.hold_fraction = 2;
            step.is_downbeat = true; /* every beat flashes for pentatonic */
            break;
        }

        default:
            break;
    }

    return step;
}
