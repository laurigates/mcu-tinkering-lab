/**
 * @file standalone_mode.c
 * @brief Beat scheduler: tempo/pitch from pots, melody_gen → tone_engine + led.
 *
 * Button short-press cycles through patterns: MARCH → WALTZ → PENTATONIC →
 * SILENCE → MARCH. Button 2-second hold resets to MARCH at beat 0.
 */

#include "standalone_mode.h"

#include <stdint.h>

#include "esp_log.h"
#include "led_single.h"
#include "melody_gen.h"
#include "pot_reader.h"
#include "tone_engine.h"

static const char *TAG = "standalone";

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static uint32_t s_beat_index;
static uint32_t s_last_beat_ms;
static uint32_t s_tone_stop_ms;
static bool s_tone_playing;

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void standalone_mode_init(void)
{
    s_beat_index = 0;
    s_last_beat_ms = 0;
    s_tone_stop_ms = 0;
    s_tone_playing = false;
    melody_gen_set_pattern(MELODY_MARCH);
    ESP_LOGI(TAG, "Standalone mode initialized");
}

void standalone_mode_tick(uint32_t now_ms)
{
    uint16_t bpm = pot_tempo_to_bpm(pot_reader_get_raw(POT_TEMPO));
    int8_t shift = pot_pitch_to_semitones(pot_reader_get_raw(POT_PITCH));
    uint32_t period_ms = 60000u / bpm;

    /* Fire next beat */
    if ((now_ms - s_last_beat_ms) >= period_ms) {
        /* Stop any prior note before starting the next */
        if (s_tone_playing) {
            tone_engine_stop();
            s_tone_playing = false;
        }

        melody_step_t step = melody_gen_next(s_beat_index);

        if (step.note != NOTE_REST) {
            tone_engine_play(step.note, shift);
            s_tone_playing = true;
            uint32_t hold_ms = period_ms / (uint32_t)step.hold_fraction;
            /* Clamp hold to at least 20 ms so the note is audible */
            if (hold_ms < 20u)
                hold_ms = 20u;
            s_tone_stop_ms = now_ms + hold_ms;
        }

        if (step.is_downbeat) {
            led_single_flash(255, 255, 255, 80);
        }

        s_beat_index++;
        s_last_beat_ms = now_ms;
    }

    /* Schedule note-off */
    if (s_tone_playing && now_ms >= s_tone_stop_ms) {
        tone_engine_stop();
        s_tone_playing = false;
    }
}

uint32_t standalone_mode_get_beat_index(void)
{
    return s_beat_index;
}

void standalone_mode_set_beat_index(uint32_t beat_index)
{
    s_beat_index = beat_index;
    /* Reset the beat timer so the next beat fires after a full period */
    s_last_beat_ms = (uint32_t)(esp_log_timestamp()); /* coarse, replaced in tick */
    ESP_LOGD(TAG, "Beat index aligned to %lu", (unsigned long)beat_index);
}

void standalone_mode_reset(void)
{
    s_beat_index = 0;
    s_last_beat_ms = 0;
    s_tone_playing = false;
    tone_engine_stop();
    melody_gen_set_pattern(MELODY_MARCH);
    led_single_clear();
    ESP_LOGI(TAG, "Standalone mode reset to MARCH beat 0");
}
