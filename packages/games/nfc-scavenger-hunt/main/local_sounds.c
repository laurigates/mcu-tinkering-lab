#include "local_sounds.h"

#include <math.h>
#include <string.h>

/*
 * All tones are generated at 24kHz/16-bit mono to match Gemini TTS output.
 * Stored in static buffers (flash/DRAM). Sizes are small enough for DRAM.
 */

#define SAMPLE_RATE 24000
#define PI 3.14159265358979323846f

/* Buffer sizes in samples */
#define STARTUP_SAMPLES (SAMPLE_RATE * 1)     // 1.0s
#define CHIME_SAMPLES (SAMPLE_RATE / 2)       // 0.5s
#define BEEP_SAMPLES (SAMPLE_RATE * 2 / 5)    // 0.4s
#define BUZZ_SAMPLES (SAMPLE_RATE * 3 / 10)   // 0.3s
#define ERROR_SAMPLES (SAMPLE_RATE * 3 / 10)  // 0.3s

static int16_t startup_buf[STARTUP_SAMPLES];
static int16_t chime_buf[CHIME_SAMPLES];
static int16_t beep_buf[BEEP_SAMPLES];
static int16_t buzz_buf[BUZZ_SAMPLES];
static int16_t error_buf[ERROR_SAMPLES];

static void generate_tone(int16_t *buf, int samples, float freq, float amplitude,
                          float fade_in_frac, float fade_out_frac)
{
    int fade_in_samples = (int)(samples * fade_in_frac);
    int fade_out_start = samples - (int)(samples * fade_out_frac);

    for (int i = 0; i < samples; i++) {
        float t = (float)i / SAMPLE_RATE;
        float sample = sinf(2.0f * PI * freq * t) * amplitude;

        /* Apply fade envelope */
        if (i < fade_in_samples) {
            sample *= (float)i / fade_in_samples;
        } else if (i > fade_out_start) {
            sample *= (float)(samples - i) / (samples - fade_out_start);
        }

        /* Clamp to int16 range */
        if (sample > 32767.0f) {
            sample = 32767.0f;
        }
        if (sample < -32768.0f) {
            sample = -32768.0f;
        }
        buf[i] = (int16_t)sample;
    }
}

static void generate_startup(void)
{
    /* Ascending three-note jingle: C5 → E5 → G5 */
    int third = STARTUP_SAMPLES / 3;
    generate_tone(startup_buf, third, 523.25f, 20000.0f, 0.05f, 0.1f);
    generate_tone(startup_buf + third, third, 659.25f, 20000.0f, 0.05f, 0.1f);
    generate_tone(startup_buf + 2 * third, STARTUP_SAMPLES - 2 * third, 783.99f, 20000.0f, 0.05f,
                  0.3f);
}

static void generate_chime(void)
{
    /* Quick ascending two-note: E5 → C6 */
    int half = CHIME_SAMPLES / 2;
    generate_tone(chime_buf, half, 659.25f, 22000.0f, 0.02f, 0.1f);
    generate_tone(chime_buf + half, CHIME_SAMPLES - half, 1046.50f, 22000.0f, 0.02f, 0.2f);
}

static void generate_beep(void)
{
    /* Double beep at A5 with gap */
    int beep_len = BEEP_SAMPLES * 3 / 8;
    int gap_start = beep_len;
    int gap_end = BEEP_SAMPLES - beep_len;

    generate_tone(beep_buf, beep_len, 880.0f, 18000.0f, 0.02f, 0.1f);
    memset(beep_buf + gap_start, 0, (gap_end - gap_start) * sizeof(int16_t));
    generate_tone(beep_buf + gap_end, BEEP_SAMPLES - gap_end, 880.0f, 18000.0f, 0.02f, 0.15f);
}

static void generate_buzz(void)
{
    /* Low gentle buzz at ~200Hz */
    generate_tone(buzz_buf, BUZZ_SAMPLES, 200.0f, 15000.0f, 0.05f, 0.2f);
}

static void generate_error(void)
{
    /* Descending two-note: B4 → F4 */
    int half = ERROR_SAMPLES / 2;
    generate_tone(error_buf, half, 493.88f, 18000.0f, 0.02f, 0.1f);
    generate_tone(error_buf + half, ERROR_SAMPLES - half, 349.23f, 18000.0f, 0.02f, 0.2f);
}

void local_sounds_init(void)
{
    generate_startup();
    generate_chime();
    generate_beep();
    generate_buzz();
    generate_error();
}

void local_sounds_get(sound_id_t id, const int16_t **pcm_data, size_t *pcm_len)
{
    switch (id) {
        case SOUND_STARTUP:
            *pcm_data = startup_buf;
            *pcm_len = sizeof(startup_buf);
            break;
        case SOUND_HAPPY_CHIME:
            *pcm_data = chime_buf;
            *pcm_len = sizeof(chime_buf);
            break;
        case SOUND_ALREADY_FOUND:
            *pcm_data = beep_buf;
            *pcm_len = sizeof(beep_buf);
            break;
        case SOUND_UNKNOWN_TAG:
            *pcm_data = buzz_buf;
            *pcm_len = sizeof(buzz_buf);
            break;
        case SOUND_ERROR:
            *pcm_data = error_buf;
            *pcm_len = sizeof(error_buf);
            break;
        default:
            *pcm_data = NULL;
            *pcm_len = 0;
            break;
    }
}
