/**
 * @file main.c
 * @brief ESP32-S3 Gamepad Synth — Bluetooth controller drives I2S audio
 *
 * Three top-level voicings cycled via the Share/View/- button:
 *   1. Continuous — monotron-style instrument (LY=pitch, sticks=tone)
 *   2. Discrete   — face buttons pick notes / chord degrees
 *   3. One-shot   — face buttons trigger retro SFX envelopes
 *
 * Orthogonal toggles layer on top (RB-held + face button):
 *   RB+A = DUAL_OSC   (Continuous: face picks interval)
 *   RB+B = DRONE_HOLD (Continuous: LY+RY integrate pitch, piezos sing)
 *   RB+X = DELAY      (Continuous + Discrete: LB+RY/RX fine-tunes)
 *   RB+Y = WAVEFORM   (global: cycles sq→saw→tri→sine)
 *   RB+LB = ARP       (Discrete: arpeggiate chord at global tempo)
 *
 * Global controls:
 *   D-pad ↑/↓     = master volume         (4 Hz auto-repeat)
 *   D-pad ←/→     = drum tempo / arp step (4 Hz auto-repeat)
 *   Share/View/-  = cycle voicing (3 choices)
 *   Home/PS/Xbox  = tap toggles drum engine, hold+face selects pattern 1-4
 *   Menu/Options  = enter settings-edit overlay (d-pad navigates fields)
 *   LT/RT         = ±7-semitone pitch bend in pitched voicings
 *   LS click      = reset per-voicing tweak parameters to defaults
 *
 * Sticks use *rate control* for tweak parameters (RY/RX): holding
 * off-center changes the value over time; releasing holds. Primary pitch
 * sticks (LY in Continuous/Discrete) stay absolute unless DRONE_HOLD.
 *
 * Bluepad32 runs on Core 0 (BTstack event loop, blocks forever).
 * Core 1 runs two tasks:
 *   - Control task at 50 Hz: reads gamepad, integrates stick state,
 *     drives synth parameters
 *   - Audio render task: generates samples, writes to I2S DMA (MAX98357A)
 *
 * Tested with: Xbox Series X/S controller (BLE, firmware v5.15+)
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Bluepad32 headers */
#include "uni_bt.h"
#include "uni_controller.h"
#include "uni_esp32.h"
#include "uni_gamepad.h"
#include "uni_hid_device.h"
#include "uni_platform.h"

#include "drums.h"
#include "piezo_voice.h"
#include "tts_player.h"

/* Voicing-announcement PCM blobs (raw 16-bit LE, 24 kHz mono) embedded via
 * EMBED_FILES in main/CMakeLists.txt. Regenerate with `just tts-generate`. */
extern const uint8_t tts_continuous_pcm_start[] asm("_binary_tts_continuous_pcm_start");
extern const uint8_t tts_continuous_pcm_end[] asm("_binary_tts_continuous_pcm_end");
extern const uint8_t tts_discrete_pcm_start[] asm("_binary_tts_discrete_pcm_start");
extern const uint8_t tts_discrete_pcm_end[] asm("_binary_tts_discrete_pcm_end");
extern const uint8_t tts_one_shot_pcm_start[] asm("_binary_tts_one_shot_pcm_start");
extern const uint8_t tts_one_shot_pcm_end[] asm("_binary_tts_one_shot_pcm_end");

#define TTS_SRC_RATE 24000
#define TTS_TAIL_MS 80

static const char *TAG = "gamepad_synth";

/* Gated diagnostic logging for axis / dispatch debugging. Enable via
 * `idf.py menuconfig` → "Gamepad Synth" → "Enable verbose axis..." to
 * trace input-mapping bugs (e.g. right-stick cardinal-axis cutoff). */
#if CONFIG_GAMEPAD_SYNTH_DEBUG_AXES
#define DBG_AXES(fmt, ...) ESP_LOGI(TAG, "AXES: " fmt, ##__VA_ARGS__)
#else
#define DBG_AXES(fmt, ...) \
    do {                   \
    } while (0)
#endif

/* ── Pin Definitions ─────────────────────────────────────── */

#define I2S_BCLK_PIN GPIO_NUM_5
#define I2S_WS_PIN GPIO_NUM_6
#define I2S_DOUT_PIN GPIO_NUM_7
#define LED_PIN GPIO_NUM_2

#define PIEZO_A_PIN GPIO_NUM_8
#define PIEZO_B_PIN GPIO_NUM_9

/* Detune between the two piezos in Drone mode. Fixed ratio so beating rate
 * scales with pitch: ~4 Hz beat at 200 Hz → ~40 Hz warble at 2 kHz. */
#define PIEZO_DETUNE_RATIO 1.02f

/* ── I2S / Audio Configuration ───────────────────────────── */

#define SAMPLE_RATE 44100
#define BLOCK_SIZE 256
#define DMA_DESC_NUM 4
#define AMPLITUDE 8000
#define SINE_TABLE_SIZE 256

static i2s_chan_handle_t s_tx_chan;

/* ── Waveform Types ──────────────────────────────────────── */

typedef enum {
    WAVE_SQUARE = 0,
    WAVE_SAWTOOTH,
    WAVE_TRIANGLE,
    WAVE_SINE,
    WAVE_NOISE,
    WAVE_COUNT,
} waveform_t;

/* Pre-computed sine table (256 entries, -AMPLITUDE to +AMPLITUDE) */
static int16_t s_sine_table[SINE_TABLE_SIZE];

static void init_sine_table(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        s_sine_table[i] =
            (int16_t)(AMPLITUDE * sinf(2.0f * 3.14159265f * (float)i / (float)SINE_TABLE_SIZE));
    }
}

static inline int16_t osc_sample(float phase, waveform_t wave)
{
    switch (wave) {
        case WAVE_SQUARE:
            return (phase < 0.5f) ? AMPLITUDE : -AMPLITUDE;
        case WAVE_SAWTOOTH:
            return (int16_t)(AMPLITUDE * (2.0f * phase - 1.0f));
        case WAVE_TRIANGLE: {
            float tri = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
            return (int16_t)(AMPLITUDE * tri);
        }
        case WAVE_SINE: {
            int idx = (int)(phase * SINE_TABLE_SIZE) & (SINE_TABLE_SIZE - 1);
            return s_sine_table[idx];
        }
        case WAVE_NOISE:
            return (int16_t)((esp_random() >> 17) - AMPLITUDE);
        default:
            return 0;
    }
}

/* ── Bluepad32 Button Constants ──────────────────────────── */

#define BTN_A (1 << 0)          /* A (Xbox) / Cross (PS) */
#define BTN_B (1 << 1)          /* B (Xbox) / Circle (PS) */
#define BTN_X (1 << 2)          /* X (Xbox) / Square (PS) */
#define BTN_Y (1 << 3)          /* Y (Xbox) / Triangle (PS) */
#define BTN_SHOULDER_L (1 << 4) /* LB (Xbox) / L1 (PS) */
#define BTN_SHOULDER_R (1 << 5) /* RB (Xbox) / R1 (PS) */
#define BTN_THUMB_L (1 << 8)    /* Left-stick click (LSB/L3) */
#define BTN_THUMB_R (1 << 9)    /* Right-stick click (RSB/R3) */

#define BTN_FACE_MASK (BTN_A | BTN_B | BTN_X | BTN_Y)

#define DPAD_UP (1 << 0)
#define DPAD_DOWN (1 << 1)
#define DPAD_RIGHT (1 << 2)
#define DPAD_LEFT (1 << 3)

#define MISC_BACK (1 << 1)   /* View (Xbox) / Share (PS) */
#define MISC_HOME (1 << 2)   /* Xbox button / PS button */
#define MISC_SELECT (1 << 3) /* Menu (Xbox) / Options (PS) */

/* ── Sound Parameters ────────────────────────────────────── */

#define MIN_FREQ 100
#define MAX_FREQ 2000
#define STICK_DEADZONE 50 /* ignore stick values below this */
#define STICK_MAX 512
#define TRIGGER_MAX 1023

#define CONTROL_TASK_HZ 50
#define CONTROL_TASK_PERIOD_MS (1000 / CONTROL_TASK_HZ)

/* ── Musical Note Table (C4–B6, equal temperament) ───────── */

static const uint16_t NOTE_FREQ[] = {
    262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466, 494, /* C4–B4 */
    523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932, 988, /* C5–B5 */
    1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760,           /* C6–A6 */
    1865, 1976,                                                           /* A#6–B6 */
};
#define NOTE_COUNT (sizeof(NOTE_FREQ) / sizeof(NOTE_FREQ[0]))

/* C major scale: indices into NOTE_FREQ within one octave */
static const uint8_t SCALE_MAJOR[] = {0, 2, 4, 5, 7, 9, 11, 12};
#define SCALE_LEN 8

/* Chord patterns (semitone offsets from root) */
static const uint8_t CHORD_MAJOR[] = {0, 4, 7, 12};
static const uint8_t CHORD_MINOR[] = {0, 3, 7, 12};
static const uint8_t CHORD_7TH[] = {0, 4, 7, 10};
static const uint8_t CHORD_DIM[] = {0, 3, 6, 9};
#define CHORD_LEN 4

/* ── Shared Gamepad State ────────────────────────────────── */

typedef struct {
    uint16_t buttons;
    uint8_t dpad;
    uint8_t misc_buttons;
    int16_t axis_x;   /* left stick X: -511..512 */
    int16_t axis_y;   /* left stick Y: -511..512 */
    int16_t axis_rx;  /* right stick X */
    int16_t axis_ry;  /* right stick Y */
    int16_t brake;    /* L2 trigger: 0..1023 */
    int16_t throttle; /* R2 trigger: 0..1023 */
    bool connected;
} gamepad_state_t;

static volatile gamepad_state_t s_gp;

/* ── Synth State (written by control task, read by audio task) ── */

typedef enum {
    LFO_TARGET_NONE = 0,
    LFO_TARGET_PITCH,
    LFO_TARGET_CUTOFF,
    LFO_TARGET_BOTH,
} lfo_target_t;

typedef struct {
    float target_freq;
    bool active;
    waveform_t waveform;
    /* Oscillator B for dual-osc and drone modes */
    bool osc_b_enabled;
    float target_freq_b;
    waveform_t waveform_b;
    bool filter_enabled;
    float filter_cutoff;    /* Hz, [40 .. 18000] */
    float filter_resonance; /* Q-like, [0.5 .. 6.0] */
    lfo_target_t lfo_target;
    float lfo_rate;  /* Hz, [0.1 .. 20] */
    float lfo_depth; /* 0.0 .. 1.0 */
    bool delay_enabled;
    int delay_samples;    /* [1 .. DELAY_MAX_SAMPLES] */
    float delay_feedback; /* 0.0 .. 0.95 */
    float delay_mix;      /* 0.0 dry .. 1.0 wet */
} synth_state_t;

static volatile synth_state_t s_synth = {0};

/* ── Settings Page ────────────────────────────────────────
 *
 * Settings-edit mode is toggled by the Menu button (MISC_SELECT). While
 * active, the normal mode-dispatch is suppressed and the d-pad is
 * repurposed:
 *   left/right — move cursor across fields
 *   up/down    — increment/decrement the currently-cursored value
 *
 * Audible feedback uses the existing synth_blip() infrastructure:
 *   • cursor move → neutral blip, then a "value ladder" encoding the value
 *   • increment   → BLIP_FREQ_UP (+ clamp)
 *   • decrement   → BLIP_FREQ_DOWN (+ clamp)
 *
 * master_volume is consumed once, in audio_render_task right before the
 * blip mix, by multiplying every sample in stereo_buf. The other fields
 * are exposed here for sibling agents (drum engine, LFO-wiring pass);
 * this module does not consume them.
 */

/* Master volume and tempo moved to the d-pad (global) in phase 1, so
 * they are no longer fields here. Pattern stays because it's a discrete
 * four-way selector and Home+face is the other entry point. */
typedef enum {
    SETTING_DRUM_PATTERN = 0,
    SETTING_DRUM_VOLUME,
    SETTING_LFO_RATE,
    SETTING_LFO_DEPTH,
    SETTING_LFO_TARGET,
    SETTING_VOICE_ANNOUNCE,
    SETTING_COUNT,
} setting_id_t;

typedef struct {
    float master_volume; /* 0.0 .. 1.0 */
    int drum_pattern;    /* 0 = off, 1..4 */
    int drum_tempo_bpm;  /* 60..200 */
    float drum_volume;   /* 0.0 .. 1.0 */
    float lfo_rate_hz;   /* 0.1 .. 20.0 */
    float lfo_depth;     /* 0.0 .. 1.0 */
    int lfo_target;      /* lfo_target_t value */
    bool voice_announce; /* true → speak voicing name on switch */
} settings_t;

volatile settings_t s_settings = {
    .master_volume = 0.7f,
    .drum_pattern = 0,
    .drum_tempo_bpm = 110,
    .drum_volume = 0.5f,
    .lfo_rate_hz = 5.0f,
    .lfo_depth = 0.3f,
    .lfo_target = LFO_TARGET_CUTOFF,
    .voice_announce = true,
};

volatile bool s_settings_edit_active = false;
volatile int s_settings_cursor = 0;

/* Settings edit input debounce tracking (rising-edge and d-pad repeat) */
#define SETTINGS_DPAD_REPEAT_MS 200

/* Global d-pad auto-repeat: 4 Hz while held. Applies to the volume
 * (↑/↓) and tempo (←/→) nudges outside the settings-edit overlay. */
#define GLOBAL_DPAD_REPEAT_MS 250

/* ── State-Variable Filter (TPT form, unconditionally stable) ── */

#define FILTER_CUTOFF_MIN 40.0f
#define FILTER_CUTOFF_MAX 18000.0f
#define FILTER_Q_MIN 0.5f
#define FILTER_Q_MAX 6.0f

/* Topology-Preserving Transform SVF (Zavalishin/Simper). Unconditionally
 * stable for any cutoff/resonance — the previous Chamberlin form diverged
 * at high cutoff (fc above ~fs/6) because f² + 2·f·q exceeded 4, sending
 * the state to NaN and silencing the audio until the active-flag reset.
 * Per-block coefficients: g = tan(pi·fc/fs), k = 1/Q,
 * a1 = 1 / (1 + g·(g + k)). State variables s1, s2 are integrator equivs;
 * v2 is the low-pass output.
 */
static inline int16_t svf_process(int16_t sample, float g, float k, float a1, float *s1, float *s2)
{
    float v0 = (float)sample;
    float v3 = v0 - *s2;
    float v1 = a1 * (*s1 + g * v3);
    float v2 = *s2 + g * v1;
    *s1 = 2.0f * v1 - *s1;
    *s2 = 2.0f * v2 - *s2;
    float out = v2;
    if (out > 32767.0f)
        out = 32767.0f;
    if (out < -32768.0f)
        out = -32768.0f;
    return (int16_t)out;
}

/* ── LFO (block-rate, triangle wave) ─────────────────────── */

#define LFO_RATE_MIN 0.1f
#define LFO_RATE_MAX 20.0f

/* Triangle wave in [-1, 1] given phase in [0, 1) */
static inline float lfo_triangle(float phase)
{
    return (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
}

/* ── Delay Line (circular buffer, ~44 KB in DRAM) ────────── */

#define DELAY_MAX_SAMPLES 22050 /* 0.5 seconds @ 44.1 kHz */
#define DELAY_FEEDBACK_MAX 0.95f

static int16_t s_delay_buf[DELAY_MAX_SAMPLES];
static int s_delay_write_idx = 0;

/* ── Confirmation Blip ───────────────────────────────────── */

/* Short sine ping mixed over the main voice to confirm silent state changes
 * (octave shifts, chord/interval picks, mode toggles). Control task writes;
 * audio task consumes. Ducking keeps the blip audible when a voice is active.
 */
#define BLIP_DURATION_MS 70
#define BLIP_FREQ_NEUTRAL 880.0f /* A5 — generic state toggle */
#define BLIP_FREQ_UP 1320.0f     /* E6 — "up" direction */
#define BLIP_FREQ_DOWN 660.0f    /* E5 — "down" direction */
#define BLIP_MAIN_DUCK_SHIFT 2   /* >> 2 = -12 dB on the underlying voice */

static volatile int s_blip_samples_remaining = 0;
static volatile float s_blip_freq_hz = BLIP_FREQ_NEUTRAL;

static void synth_blip(float freq_hz)
{
    s_blip_freq_hz = freq_hz;
    s_blip_samples_remaining = (SAMPLE_RATE * BLIP_DURATION_MS) / 1000;
}

static inline int16_t clip_i16(float x)
{
    if (x > 32767.0f)
        return 32767;
    if (x < -32768.0f)
        return -32768;
    return (int16_t)x;
}

static inline int16_t delay_process(int16_t in, int delay_samples, float feedback, float mix)
{
    int read_idx = s_delay_write_idx - delay_samples;
    if (read_idx < 0)
        read_idx += DELAY_MAX_SAMPLES;
    int16_t delayed = s_delay_buf[read_idx];

    /* Write input + feedback into the buffer */
    s_delay_buf[s_delay_write_idx] = clip_i16((float)in + feedback * (float)delayed);
    s_delay_write_idx++;
    if (s_delay_write_idx >= DELAY_MAX_SAMPLES)
        s_delay_write_idx = 0;

    /* Output is dry + wet mix */
    return clip_i16((1.0f - mix) * (float)in + mix * (float)delayed);
}

/* ── Trigger-driven Pitch Bend ───────────────────────────── */

/* LT/RT act as a pitch-bend pair across pitched modes. Range is ±7 semitones,
 * where LT full = −7 st and RT full = +7 st. Both pressed equally cancels out
 * to a 1.0 multiplier; asymmetric presses give a fine detune. */
static inline float compute_bend_mult(const gamepad_state_t *gp)
{
    float bend_semitones = ((float)(gp->throttle - gp->brake) / (float)TRIGGER_MAX) * 7.0f;
    if (bend_semitones > 7.0f)
        bend_semitones = 7.0f;
    if (bend_semitones < -7.0f)
        bend_semitones = -7.0f;
    return powf(2.0f, bend_semitones / 12.0f);
}

/* ── Voicings and Voicing Config ─────────────────────────
 *
 * Three top-level voicings cycled by Share/View/-. Orthogonal toggles
 * layer on top (RB-held + face button). Per-voicing config persists
 * across voicing switches — finding a sweet setup in Continuous and
 * switching to Discrete for a chord bridge preserves the Continuous
 * setup.
 */

typedef enum {
    VOICING_CONTINUOUS = 0, /* LY=pitch, sticks shape the tone */
    VOICING_DISCRETE,       /* Face buttons play notes / chord degrees */
    VOICING_ONE_SHOT,       /* Face buttons trigger SFX envelopes */
    VOICING_COUNT,
} voicing_t;

typedef struct {
    bool dual_osc;       /* Continuous only: enable osc B */
    bool drone_hold;     /* Continuous only: LY+RY integrate pitch, piezos sing; forces dual_osc */
    bool delay;          /* Continuous + Discrete: delay tail */
    bool arp;            /* Discrete only: arp overlay */
    waveform_t waveform; /* Per-voicing timbre */
    int interval_semitones; /* Dual-osc interval (Continuous) */
} voicing_cfg_t;

static voicing_t s_voicing = VOICING_CONTINUOUS;
static voicing_cfg_t s_cfg[VOICING_COUNT] = {
    [VOICING_CONTINUOUS] = {.waveform = WAVE_SAWTOOTH, .interval_semitones = 0},
    [VOICING_DISCRETE] = {.waveform = WAVE_SINE, .interval_semitones = 0},
    [VOICING_ONE_SHOT] = {.waveform = WAVE_SQUARE, .interval_semitones = 0},
};

/* Arpeggiator state (used by VOICING_DISCRETE when cfg.arp is true) */
static const uint8_t *s_arp_chord = CHORD_MAJOR;
static int s_arp_root = 0;      /* semitone offset from C4 */
static int s_arp_index = 0;     /* current note in chord */
static int s_arp_direction = 1; /* 1=up, -1=down */
static bool s_arp_running = false;
static uint32_t s_arp_last_ms = 0;

/* SFX state (used by VOICING_ONE_SHOT) */
typedef struct {
    float freq;
    float freq_end;
    float step;
    int ticks_left;
} sfx_state_t;
static sfx_state_t s_sfx = {0};

/* Scale octave for arp: 0=C4, 1=C5, 2=C6 */
static int s_octave = 1;

/* Mode switch debounce */
static uint8_t s_prev_misc = 0;

/* Drum engine: on/off toggled by Home button. The pattern index itself
 * lives in s_settings.drum_pattern and is read on toggle. Tempo/volume
 * are pushed to the engine every control-task tick. */
static bool s_drum_on = false;

/* ── Tone Helpers (now set synth state instead of LEDC) ──── */

static void tone_play(uint32_t freq_hz)
{
    if (freq_hz < MIN_FREQ)
        freq_hz = MIN_FREQ;
    if (freq_hz > MAX_FREQ)
        freq_hz = MAX_FREQ;
    s_synth.target_freq = (float)freq_hz;
    s_synth.active = true;
}

static void synth_set_waveform(waveform_t wave)
{
    s_synth.waveform = wave;
}

static void synth_set_osc_b(bool enabled, float freq_hz, waveform_t wave)
{
    if (freq_hz < (float)MIN_FREQ)
        freq_hz = (float)MIN_FREQ;
    if (freq_hz > (float)MAX_FREQ)
        freq_hz = (float)MAX_FREQ;
    s_synth.osc_b_enabled = enabled;
    s_synth.target_freq_b = freq_hz;
    s_synth.waveform_b = wave;
}

static void synth_set_filter(bool enabled, float cutoff, float resonance)
{
    if (cutoff < FILTER_CUTOFF_MIN)
        cutoff = FILTER_CUTOFF_MIN;
    if (cutoff > FILTER_CUTOFF_MAX)
        cutoff = FILTER_CUTOFF_MAX;
    if (resonance < FILTER_Q_MIN)
        resonance = FILTER_Q_MIN;
    if (resonance > FILTER_Q_MAX)
        resonance = FILTER_Q_MAX;
    s_synth.filter_enabled = enabled;
    s_synth.filter_cutoff = cutoff;
    s_synth.filter_resonance = resonance;
}

static void synth_set_lfo(lfo_target_t target, float rate, float depth)
{
    if (rate < LFO_RATE_MIN)
        rate = LFO_RATE_MIN;
    if (rate > LFO_RATE_MAX)
        rate = LFO_RATE_MAX;
    if (depth < 0.0f)
        depth = 0.0f;
    if (depth > 1.0f)
        depth = 1.0f;
    s_synth.lfo_target = target;
    s_synth.lfo_rate = rate;
    s_synth.lfo_depth = depth;
}

static void synth_set_delay(bool enabled, int samples, float feedback, float mix)
{
    if (samples < 1)
        samples = 1;
    if (samples > DELAY_MAX_SAMPLES)
        samples = DELAY_MAX_SAMPLES;
    if (feedback < 0.0f)
        feedback = 0.0f;
    if (feedback > DELAY_FEEDBACK_MAX)
        feedback = DELAY_FEEDBACK_MAX;
    if (mix < 0.0f)
        mix = 0.0f;
    if (mix > 1.0f)
        mix = 1.0f;
    s_synth.delay_enabled = enabled;
    s_synth.delay_samples = samples;
    s_synth.delay_feedback = feedback;
    s_synth.delay_mix = mix;
}

static void tone_stop(void)
{
    s_synth.active = false;
}

static uint16_t note_at(int semitone)
{
    if (semitone < 0)
        semitone = 0;
    if (semitone >= (int)NOTE_COUNT)
        semitone = NOTE_COUNT - 1;
    return NOTE_FREQ[semitone];
}

/* Map a value from [in_min..in_max] to [out_min..out_max] */
static float map_range(float val, float in_min, float in_max, float out_min, float out_max)
{
    if (val < in_min)
        val = in_min;
    if (val > in_max)
        val = in_max;
    return out_min + (out_max - out_min) * ((val - in_min) / (in_max - in_min));
}

/* ── Integrating-stick helpers ─────────────────────────────
 *
 * Sticks now drive *rate of change* for parameters rather than absolute
 * position (except for the primary pitch stick in pitched modes, where
 * absolute mapping preserves theremin-style muscle memory).
 *
 * stick_rate()        : raw stick value → normalized rate in [-1, +1]
 *                       after deadzone. Linear beyond the deadzone.
 *
 * integrate_exp()     : multiply a value by 2^(rate·octaves_per_sec·dt)
 *                       and clamp. Use for log-perceptual params like
 *                       cutoff, pitch, delay-time.
 *
 * integrate_lin()     : value += rate·units_per_sec·dt, clamped.
 *                       Use for linear params (resonance, feedback,
 *                       detune cents, mix).
 *
 * Both integrate_* return true when the value is clamped at a limit
 * *this tick*, so the caller can emit a "bump" blip at the edge.
 */
#define INTEGRATION_DT (1.0f / (float)CONTROL_TASK_HZ)
#define STICK_RATE_RANGE ((float)(STICK_MAX - STICK_DEADZONE))

static inline float stick_rate(int16_t stick_val)
{
    if (stick_val > STICK_DEADZONE)
        return (float)(stick_val - STICK_DEADZONE) / STICK_RATE_RANGE;
    if (stick_val < -STICK_DEADZONE)
        return (float)(stick_val + STICK_DEADZONE) / STICK_RATE_RANGE;
    return 0.0f;
}

static inline bool integrate_exp(float *value, float rate, float octaves_per_sec, float min_v,
                                 float max_v, float dt)
{
    if (rate == 0.0f)
        return false;
    float prev = *value;
    *value *= powf(2.0f, rate * octaves_per_sec * dt);
    if (*value < min_v) {
        *value = min_v;
        return prev > min_v;
    }
    if (*value > max_v) {
        *value = max_v;
        return prev < max_v;
    }
    return false;
}

static inline bool integrate_lin(float *value, float rate, float units_per_sec, float min_v,
                                 float max_v, float dt)
{
    if (rate == 0.0f)
        return false;
    float prev = *value;
    *value += rate * units_per_sec * dt;
    if (*value < min_v) {
        *value = min_v;
        return prev > min_v;
    }
    if (*value > max_v) {
        *value = max_v;
        return prev < max_v;
    }
    return false;
}

/* ── Shared Tweak Parameter State ─────────────────────────
 *
 * These persist across ticks within a mode (so finding a sweet spot and
 * releasing the stick keeps the value). They reset on mode switch and on
 * LS-click. Every pitched mode pulls cutoff/resonance from here so the
 * right stick always means the same thing.
 */
#define TWEAK_CUTOFF_DEFAULT 6000.0f
#define TWEAK_RESONANCE_DEFAULT 1.5f
#define TWEAK_DELAY_MS_DEFAULT 200.0f
#define TWEAK_FEEDBACK_DEFAULT 0.4f
#define TWEAK_DETUNE_CENTS_DEFAULT 0.0f
#define TWEAK_PITCH_OFFSET_ST_DEFAULT 0.0f

static float s_tweak_cutoff_hz = TWEAK_CUTOFF_DEFAULT;
static float s_tweak_resonance = TWEAK_RESONANCE_DEFAULT;
static float s_tweak_delay_ms = TWEAK_DELAY_MS_DEFAULT;
static float s_tweak_feedback = TWEAK_FEEDBACK_DEFAULT;
static float s_tweak_detune_cents = TWEAK_DETUNE_CENTS_DEFAULT;
static float s_tweak_pitch_offset_st = TWEAK_PITCH_OFFSET_ST_DEFAULT; /* Scale root slide */
/* Drone mode: both oscillator freqs drift via integrating sticks. */
static float s_drone_freq_a = 220.0f;
static float s_drone_freq_b = 330.0f;

static void reset_tweaks(void)
{
    s_tweak_cutoff_hz = TWEAK_CUTOFF_DEFAULT;
    s_tweak_resonance = TWEAK_RESONANCE_DEFAULT;
    s_tweak_delay_ms = TWEAK_DELAY_MS_DEFAULT;
    s_tweak_feedback = TWEAK_FEEDBACK_DEFAULT;
    s_tweak_detune_cents = TWEAK_DETUNE_CENTS_DEFAULT;
    s_tweak_pitch_offset_st = TWEAK_PITCH_OFFSET_ST_DEFAULT;
    s_drone_freq_a = 220.0f;
    s_drone_freq_b = 330.0f;
}

/* Integrate cutoff (RY) and resonance (RX) from the right stick. Applies
 * the result via synth_set_filter(). Emits a short bump-blip at the
 * edges so the user can tell they're maxed out. */
static void integrate_right_stick_filter(int16_t ry, int16_t rx)
{
    bool bumped = false;
    bumped |= integrate_exp(&s_tweak_cutoff_hz, stick_rate(ry), 4.0f, FILTER_CUTOFF_MIN,
                            FILTER_CUTOFF_MAX, INTEGRATION_DT);
    bumped |= integrate_lin(&s_tweak_resonance, stick_rate(rx), 3.0f, FILTER_Q_MIN, FILTER_Q_MAX,
                            INTEGRATION_DT);
    synth_set_filter(true, s_tweak_cutoff_hz, s_tweak_resonance);

    /* Rate-limit the bump blip so it doesn't spam while pinned at a limit. */
    static uint32_t last_bump_ms = 0;
    if (bumped) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_bump_ms > 400) {
            synth_blip(BLIP_FREQ_NEUTRAL * 0.5f); /* low "thud" */
            last_bump_ms = now;
        }
    }
}

/* ── LED Helpers ─────────────────────────────────────────── */

static void led_on(void)
{
    gpio_set_level(LED_PIN, 1);
}
static void led_off(void)
{
    gpio_set_level(LED_PIN, 0);
}

/* ── SFX helpers (used by VOICING_ONE_SHOT) ──────────────── */

static void sfx_start(float start, float end, int ticks)
{
    s_sfx.freq = start;
    s_sfx.freq_end = end;
    s_sfx.step = (end - start) / (float)ticks;
    s_sfx.ticks_left = ticks;
}

static void sfx_tick(void)
{
    if (s_sfx.ticks_left <= 0) {
        tone_stop();
        led_off();
        return;
    }
    tone_play((uint32_t)s_sfx.freq);
    led_on();
    s_sfx.freq += s_sfx.step;
    s_sfx.ticks_left--;
}

/* Arpeggiator core — called from VOICING_DISCRETE when cfg.arp is true.
 * Handles chord selection on face-button rising edge and steps through
 * the chord at global-tempo 16ths. */
static void arp_tick(const gamepad_state_t *gp, uint16_t face_pressed)
{
    if (face_pressed & BTN_A) {
        s_arp_chord = CHORD_MAJOR;
        s_arp_running = true;
        s_arp_index = 0;
        s_arp_direction = 1;
        synth_blip(BLIP_FREQ_UP);
    } else if (face_pressed & BTN_B) {
        s_arp_chord = CHORD_MINOR;
        s_arp_running = true;
        s_arp_index = 0;
        s_arp_direction = 1;
        synth_blip(BLIP_FREQ_DOWN);
    } else if (face_pressed & BTN_X) {
        s_arp_chord = CHORD_7TH;
        s_arp_running = true;
        s_arp_index = 0;
        s_arp_direction = 1;
        synth_blip(BLIP_FREQ_NEUTRAL);
    } else if (face_pressed & BTN_Y) {
        s_arp_chord = CHORD_DIM;
        s_arp_running = true;
        s_arp_index = 0;
        s_arp_direction = 1;
        synth_blip(BLIP_FREQ_DOWN * 0.75f);
    }

    /* LY-integrated pitch offset drives the arp root. */
    s_arp_root = (int)s_tweak_pitch_offset_st;

    /* RT rising edge still toggles arp running (off-switch without
     * disabling the whole cfg.arp toggle). */
    static bool prev_r2 = false;
    bool r2 = gp->throttle > 100;
    if (r2 && !prev_r2) {
        s_arp_running = !s_arp_running;
        s_arp_index = 0;
        s_arp_direction = 1;
        synth_blip(s_arp_running ? BLIP_FREQ_UP : BLIP_FREQ_DOWN);
        if (!s_arp_running)
            tone_stop();
    }
    prev_r2 = r2;

    if (!s_arp_running) {
        led_off();
        DBG_AXES("voicing=discrete arp=on running=false stop_reason=arp_not_running");
        return;
    }

    /* Step rate is derived from the global tempo (16th notes). */
    float speed_ms = 60000.0f / (float)s_settings.drum_tempo_bpm / 4.0f;

    /* Pattern from left stick X quadrant: left=down, right=up, center=up-down */
    int pattern;
    if (gp->axis_x < -STICK_DEADZONE)
        pattern = 1;
    else if (gp->axis_x > STICK_DEADZONE)
        pattern = 0;
    else
        pattern = 2;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now_ms - s_arp_last_ms >= (uint32_t)speed_ms) {
        s_arp_last_ms = now_ms;

        int semitone = (s_octave * 12) + s_arp_root + s_arp_chord[s_arp_index];
        tone_play(note_at(semitone));
        led_on();

        if (pattern == 0) {
            s_arp_index = (s_arp_index + 1) % CHORD_LEN;
        } else if (pattern == 1) {
            s_arp_index--;
            if (s_arp_index < 0)
                s_arp_index = CHORD_LEN - 1;
        } else {
            s_arp_index += s_arp_direction;
            if (s_arp_index >= CHORD_LEN) {
                s_arp_index = CHORD_LEN - 2;
                s_arp_direction = -1;
            } else if (s_arp_index < 0) {
                s_arp_index = 1;
                s_arp_direction = 1;
            }
        }
    }
}

/* ── Voicing 1: Continuous (Mono + Dual Osc + Delay + Drone) ── */

static void voicing_continuous(const gamepad_state_t *gp, uint16_t face_pressed)
{
    const voicing_cfg_t cfg = s_cfg[VOICING_CONTINUOUS];
    bool lb_held = (gp->buttons & BTN_SHOULDER_L) != 0;

    /* LFO from settings (Mono+Drone both wired this up). */
    synth_set_lfo((lfo_target_t)s_settings.lfo_target, s_settings.lfo_rate_hz,
                  s_settings.lfo_depth);

    /* Face-button interval pick (dual-osc, not drone-hold). Drone-hold
     * doesn't use intervals because osc B is piezo-only and follows
     * RY-integrating pitch instead. */
    if (cfg.dual_osc && !cfg.drone_hold) {
        if (face_pressed & BTN_A) {
            s_cfg[VOICING_CONTINUOUS].interval_semitones = 0;
            synth_blip(BLIP_FREQ_DOWN);
        } else if (face_pressed & BTN_B) {
            s_cfg[VOICING_CONTINUOUS].interval_semitones = 7;
            synth_blip(BLIP_FREQ_NEUTRAL);
        } else if (face_pressed & BTN_X) {
            s_cfg[VOICING_CONTINUOUS].interval_semitones = 12;
            synth_blip(BLIP_FREQ_UP);
        } else if (face_pressed & BTN_Y) {
            s_cfg[VOICING_CONTINUOUS].interval_semitones = 24;
            synth_blip(BLIP_FREQ_UP * 1.5f);
        }
    }

    /* Right-stick tweaks — delay params if LB+delay, else filter. */
    if (cfg.delay && lb_held) {
        bool bumped = false;
        bumped |= integrate_exp(&s_tweak_delay_ms, stick_rate(gp->axis_ry), 2.0f, 20.0f, 500.0f,
                                INTEGRATION_DT);
        bumped |= integrate_lin(&s_tweak_feedback, stick_rate(gp->axis_rx), 0.5f, 0.0f, 0.9f,
                                INTEGRATION_DT);
        static uint32_t last_bump_ms = 0;
        if (bumped) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_bump_ms > 400) {
                synth_blip(BLIP_FREQ_NEUTRAL * 0.5f);
                last_bump_ms = now;
            }
        }
    } else if (cfg.drone_hold) {
        /* Drone-hold: LX integrates filter cutoff, RX integrates resonance.
         * (LY=pitch A, RY=pitch B are handled below in the render path.) */
        integrate_exp(&s_tweak_cutoff_hz, stick_rate(gp->axis_x), 4.0f, FILTER_CUTOFF_MIN,
                      FILTER_CUTOFF_MAX, INTEGRATION_DT);
        integrate_lin(&s_tweak_resonance, stick_rate(gp->axis_rx), 3.0f, FILTER_Q_MIN, FILTER_Q_MAX,
                      INTEGRATION_DT);
        synth_set_filter(true, s_tweak_cutoff_hz, s_tweak_resonance);
    } else {
        integrate_right_stick_filter(gp->axis_ry, gp->axis_rx);
    }

    /* Apply delay from persistent tweak state (when delay is enabled). */
    if (cfg.delay) {
        int delay_samples = (int)(SAMPLE_RATE * s_tweak_delay_ms / 1000.0f);
        synth_set_delay(true, delay_samples, s_tweak_feedback, 0.5f);
    } else {
        synth_set_delay(false, 1, 0.0f, 0.0f);
    }

    float bend_mult = compute_bend_mult(gp);

    if (cfg.drone_hold) {
        /* Both DAC osc A pitch (LY) and piezo osc B pitch (RY) integrate. */
        integrate_exp(&s_drone_freq_a, stick_rate(gp->axis_y), 2.0f, (float)MIN_FREQ,
                      (float)MAX_FREQ, INTEGRATION_DT);
        integrate_exp(&s_drone_freq_b, stick_rate(gp->axis_ry), 2.0f, (float)MIN_FREQ,
                      (float)MAX_FREQ, INTEGRATION_DT);
        float pitch_a = s_drone_freq_a * bend_mult;
        float pitch_b = s_drone_freq_b * bend_mult;
        tone_play((uint32_t)pitch_a);
        synth_set_osc_b(false, (float)MIN_FREQ, cfg.waveform); /* Osc B is piezo-only in drone */
        piezo_voice_note_on(PIEZO_A, pitch_b);
        piezo_voice_note_on(PIEZO_B, pitch_b * PIEZO_DETUNE_RATIO);
        led_on();
        return;
    }

    /* Standard continuous: LY=absolute pitch, LX=vibrato depth. */
    int16_t ly = gp->axis_y;
    int16_t lx = gp->axis_x;
    if (abs(ly) < STICK_DEADZONE && abs(lx) < STICK_DEADZONE) {
        tone_stop();
        synth_set_osc_b(false, (float)MIN_FREQ, cfg.waveform);
        piezo_voice_note_off(PIEZO_A);
        piezo_voice_note_off(PIEZO_B);
        led_off();
        DBG_AXES("voicing=continuous ly=%d lx=%d stop_reason=deadzone", ly, lx);
        return;
    }

    float pitch = map_range((float)ly, -STICK_MAX, STICK_MAX, MIN_FREQ, MAX_FREQ);
    float vib_depth = map_range((float)abs(lx), 0, STICK_MAX, 0, 100);
    static uint32_t tick = 0;
    tick++;
    float vib = vib_depth * sinf(2.0f * 3.14159f * 5.0f * (float)tick / CONTROL_TASK_HZ);
    pitch += vib;
    pitch *= bend_mult;

    tone_play((uint32_t)pitch);

    if (cfg.dual_osc) {
        int interval = s_cfg[VOICING_CONTINUOUS].interval_semitones;
        float detune_mult = powf(2.0f, s_tweak_detune_cents / 1200.0f);
        float pitch_b = pitch * powf(2.0f, (float)interval / 12.0f) * detune_mult;
        synth_set_osc_b(true, pitch_b, cfg.waveform);
    } else {
        synth_set_osc_b(false, (float)MIN_FREQ, cfg.waveform);
    }
    led_on();
}

/* ── Voicing 2: Discrete (Scale + Arpeggio) ───────────────── */

static void voicing_discrete(const gamepad_state_t *gp, uint16_t face_pressed)
{
    const voicing_cfg_t cfg = s_cfg[VOICING_DISCRETE];
    bool lb_held = (gp->buttons & BTN_SHOULDER_L) != 0;

    /* LY integrates pitch offset (±12 st) — root transpose or scale slide. */
    integrate_lin(&s_tweak_pitch_offset_st, stick_rate(gp->axis_y), 6.0f, -12.0f, 12.0f,
                  INTEGRATION_DT);

    /* RX integrates filter cutoff (resonance stays at current tweak state
     * unless the user flips over to Continuous and adjusts it). */
    if (cfg.delay && lb_held) {
        bool bumped = false;
        bumped |= integrate_exp(&s_tweak_delay_ms, stick_rate(gp->axis_ry), 2.0f, 20.0f, 500.0f,
                                INTEGRATION_DT);
        bumped |= integrate_lin(&s_tweak_feedback, stick_rate(gp->axis_rx), 0.5f, 0.0f, 0.9f,
                                INTEGRATION_DT);
        static uint32_t last_bump_ms = 0;
        if (bumped) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_bump_ms > 400) {
                synth_blip(BLIP_FREQ_NEUTRAL * 0.5f);
                last_bump_ms = now;
            }
        }
    } else {
        integrate_exp(&s_tweak_cutoff_hz, stick_rate(gp->axis_rx), 4.0f, FILTER_CUTOFF_MIN,
                      FILTER_CUTOFF_MAX, INTEGRATION_DT);
    }
    synth_set_filter(true, s_tweak_cutoff_hz, s_tweak_resonance);

    if (cfg.delay) {
        int delay_samples = (int)(SAMPLE_RATE * s_tweak_delay_ms / 1000.0f);
        synth_set_delay(true, delay_samples, s_tweak_feedback, 0.5f);
    } else {
        synth_set_delay(false, 1, 0.0f, 0.0f);
    }
    synth_set_osc_b(false, (float)MIN_FREQ, cfg.waveform);

    if (cfg.arp) {
        arp_tick(gp, face_pressed);
        return;
    }

    /* Scale mode: face buttons play scalar notes.
     *   LB-not-held + A/B/X/Y → Do  Re  Mi  Fa
     *   LB-held     + A/B/X/Y → Sol La  Ti  Do  */
    int note_idx = -1;
    if (gp->buttons & BTN_A)
        note_idx = lb_held ? 4 : 0;
    if (gp->buttons & BTN_B)
        note_idx = lb_held ? 5 : 1;
    if (gp->buttons & BTN_X)
        note_idx = lb_held ? 6 : 2;
    if (gp->buttons & BTN_Y)
        note_idx = lb_held ? 7 : 3;

    if (note_idx < 0) {
        tone_stop();
        led_off();
        DBG_AXES("voicing=discrete arp=off stop_reason=no_face_button");
        return;
    }

    int semitone = (12 /* base C5 */) + SCALE_MAJOR[note_idx] + (int)s_tweak_pitch_offset_st;
    float freq = (float)note_at(semitone);
    /* RY = fine bend (absolute). */
    freq += map_range((float)gp->axis_ry, -STICK_MAX, STICK_MAX, -50, 50);
    freq *= compute_bend_mult(gp);
    tone_play((uint32_t)freq);
    led_on();
}

/* ── Voicing 3: One-Shot (Retro SFX) ──────────────────────── */

static void voicing_one_shot(const gamepad_state_t *gp, uint16_t face_pressed)
{
    /* RT = speed multiplier on the sweep tick count. */
    float speed = map_range((float)gp->throttle, 0, TRIGGER_MAX, 1.0f, 0.3f);
    int base_ticks = 25;
    int ticks = (int)((float)base_ticks * speed);
    if (ticks < 5)
        ticks = 5;

    bool lb_held = (gp->buttons & BTN_SHOULDER_L) != 0;

    if (!lb_held) {
        if (face_pressed & BTN_A)
            sfx_start(1800, 200, ticks); /* Laser */
        if (face_pressed & BTN_B)
            sfx_start(800, 100, ticks * 2); /* Explosion */
        if (face_pressed & BTN_X)
            sfx_start(200, 1600, ticks); /* Power-up */
        if (face_pressed & BTN_Y)
            sfx_start(1200, 1800, ticks / 2); /* Coin */
    } else {
        if (face_pressed & BTN_A)
            sfx_start(400, 1200, ticks * 3); /* Siren */
        if (face_pressed & BTN_B)
            sfx_start(100, 250, ticks * 3); /* Engine */
        if (face_pressed & BTN_X)
            sfx_start(300, 1400, ticks / 2); /* Jump */
        if (face_pressed & BTN_Y)
            sfx_start(400, 1800, ticks); /* Warp */
    }

    sfx_tick();
}

/* ── TTS voicing announcement ────────────────────────────
 *
 * Plays the spoken voicing name (embedded PCM, 24 kHz mono) before the
 * musical signature gesture. Blocks the control task for the clip's
 * duration + 80 ms tail so the buffer drains cleanly before the signature
 * starts. Drum engine volume is ducked to 0 during the clip and restored
 * afterward so two-syllable words stay intelligible over a running beat.
 *
 * Synth is already silent at this point (tone_stop() fires earlier in
 * enter_voicing), so we don't need to mute the synth path.
 */
static void tts_play_voicing(voicing_t v)
{
    const uint8_t *start = NULL;
    const uint8_t *end = NULL;
    switch (v) {
        case VOICING_CONTINUOUS:
            start = tts_continuous_pcm_start;
            end = tts_continuous_pcm_end;
            break;
        case VOICING_DISCRETE:
            start = tts_discrete_pcm_start;
            end = tts_discrete_pcm_end;
            break;
        case VOICING_ONE_SHOT:
            start = tts_one_shot_pcm_start;
            end = tts_one_shot_pcm_end;
            break;
        default:
            return;
    }

    int num_bytes = (int)(end - start);
    int num_samples = num_bytes / (int)sizeof(int16_t);
    if (num_samples <= 1)
        return;
    int duration_ms = (num_samples * 1000) / TTS_SRC_RATE + TTS_TAIL_MS;

    float saved_vol = s_settings.drum_volume;
    drums_set_volume(0.0f);
    tts_player_start((const int16_t *)start, num_samples);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    drums_set_volume(saved_vol);
}

/* ── Voicing Signature Gestures ──────────────────────────
 *
 * On voicing switch, play a short phrase *in that voicing's own voice*
 * so the user hears which voicing they're in without having to count
 * LED blinks. Blocks the control task for ~300-500 ms; audio render
 * keeps running, so the user hears the gesture immediately.
 */
static void play_voicing_signature(voicing_t voicing)
{
    switch (voicing) {
        case VOICING_CONTINUOUS: {
            /* Pitch-bend chirp (sawtooth, 400→1200 Hz) — instrument feel */
            synth_set_waveform(WAVE_SAWTOOTH);
            synth_set_filter(true, 6000.0f, 1.5f);
            synth_set_osc_b(false, (float)MIN_FREQ, WAVE_SAWTOOTH);
            synth_set_lfo(LFO_TARGET_NONE, 5.0f, 0.0f);
            synth_set_delay(false, 1, 0.0f, 0.0f);
            for (int i = 0; i < 5; i++) {
                tone_play(400 + i * 200);
                vTaskDelay(pdMS_TO_TICKS(60));
            }
            break;
        }
        case VOICING_DISCRETE: {
            /* Do-Mi-Sol ascending on sine — melodic feel */
            synth_set_waveform(WAVE_SINE);
            synth_set_filter(true, 5000.0f, 0.8f);
            synth_set_osc_b(false, (float)MIN_FREQ, WAVE_SAWTOOTH);
            synth_set_lfo(LFO_TARGET_NONE, 5.0f, 0.0f);
            synth_set_delay(true, SAMPLE_RATE * 120 / 1000, 0.25f, 0.3f);
            tone_play(523);
            vTaskDelay(pdMS_TO_TICKS(110));
            tone_play(659);
            vTaskDelay(pdMS_TO_TICKS(110));
            tone_play(784);
            vTaskDelay(pdMS_TO_TICKS(160));
            break;
        }
        case VOICING_ONE_SHOT: {
            /* Laser zap — descending square sweep */
            synth_set_waveform(WAVE_SQUARE);
            synth_set_filter(false, FILTER_CUTOFF_MAX, FILTER_Q_MIN);
            synth_set_osc_b(false, (float)MIN_FREQ, WAVE_SAWTOOTH);
            synth_set_lfo(LFO_TARGET_NONE, 5.0f, 0.0f);
            synth_set_delay(false, 1, 0.0f, 0.0f);
            for (int i = 0; i < 10; i++) {
                tone_play(1800 - i * 160);
                vTaskDelay(pdMS_TO_TICKS(22));
            }
            break;
        }
        default:
            break;
    }
    tone_stop();
    vTaskDelay(pdMS_TO_TICKS(60));
}

/* ── Toggle cues ─────────────────────────────────────────── */

/* Two-blip rising cue for toggle-on, single descending blip for toggle-off. */
static void toggle_cue(bool enabled)
{
    if (enabled) {
        synth_blip(BLIP_FREQ_UP);
        vTaskDelay(pdMS_TO_TICKS(110));
        synth_blip(BLIP_FREQ_UP * 1.25f);
    } else {
        synth_blip(BLIP_FREQ_DOWN);
    }
}

/* Waveform cycle cue: signature blip frequency per waveform so the user
 * can identify the new timbre without hearing it yet. */
static void waveform_cue(waveform_t w)
{
    static const float freqs[WAVE_COUNT] = {
        [WAVE_SQUARE] = BLIP_FREQ_NEUTRAL,      [WAVE_SAWTOOTH] = BLIP_FREQ_UP,
        [WAVE_TRIANGLE] = BLIP_FREQ_UP * 1.15f, [WAVE_SINE] = BLIP_FREQ_UP * 1.3f,
        [WAVE_NOISE] = BLIP_FREQ_DOWN * 0.75f,
    };
    synth_blip(freqs[w]);
}

/* ── Bluepad32 Custom Platform ───────────────────────────── */

static void plat_init(int argc, const char **argv)
{
    (void)argc;
    (void)argv;
    ESP_LOGI(TAG, "Bluepad32 platform initialized");
}

static void plat_on_init_complete(void)
{
    ESP_LOGI(TAG, "Scanning for BLE controllers...");
    uni_bt_enable_new_connections_safe(true);
}

static void plat_on_device_connected(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Device connected: %p", d);
}

static void plat_on_device_disconnected(uni_hid_device_t *d)
{
    ESP_LOGW(TAG, "Device disconnected: %p", d);
    gamepad_state_t empty = {0};
    memcpy((void *)&s_gp, &empty, sizeof(s_gp));
}

static uni_error_t plat_on_device_ready(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Device ready: VID=0x%04X PID=0x%04X", uni_hid_device_get_vendor_id(d),
             uni_hid_device_get_product_id(d));
    s_gp.connected = true;
    return UNI_ERROR_SUCCESS;
}

static void plat_on_controller_data(uni_hid_device_t *d, uni_controller_t *ctl)
{
    (void)d;
    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD)
        return;
    const uni_gamepad_t *gp = &ctl->gamepad;
    s_gp.buttons = gp->buttons;
    s_gp.dpad = gp->dpad;
    s_gp.misc_buttons = gp->misc_buttons;
    s_gp.axis_x = gp->axis_x;
    s_gp.axis_y = gp->axis_y;
    s_gp.axis_rx = gp->axis_rx;
    s_gp.axis_ry = gp->axis_ry;
    s_gp.brake = gp->brake;
    s_gp.throttle = gp->throttle;
    s_gp.connected = true;
}

static int32_t plat_get_property(uni_platform_property_t key)
{
    (void)key;
    return -1;
}

static void plat_on_oob_event(uni_platform_oob_event_t event, void *data)
{
    (void)data;
    ESP_LOGD(TAG, "OOB event: %d", event);
}

static struct uni_platform s_platform = {
    .name = "gamepad_synth",
    .init = plat_init,
    .on_init_complete = plat_on_init_complete,
    .on_device_connected = plat_on_device_connected,
    .on_device_disconnected = plat_on_device_disconnected,
    .on_device_ready = plat_on_device_ready,
    .on_gamepad_data = NULL,
    .on_controller_data = plat_on_controller_data,
    .get_property = plat_get_property,
    .on_oob_event = plat_on_oob_event,
    .device_dump = NULL,
    .register_console_cmds = NULL,
};

struct uni_platform *uni_platform_custom_create(void)
{
    return &s_platform;
}

/* ── Hardware Init ───────────────────────────────────────── */

static void init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = BLOCK_SIZE;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = I2S_BCLK_PIN,
                .ws = I2S_WS_PIN,
                .dout = I2S_DOUT_PIN,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {0},
            },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_LOGI(TAG, "I2S initialized: BCLK=GPIO%d WS=GPIO%d DOUT=GPIO%d @ %d Hz", I2S_BCLK_PIN,
             I2S_WS_PIN, I2S_DOUT_PIN, SAMPLE_RATE);
}

static void init_led(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    led_off();
}

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

/* ── Startup Jingle (blocking, runs before tasks) ────────── */

static void play_startup_jingle(void)
{
    /* C5 E5 G5 C6 — major arpeggio rendered directly via I2S */
    const uint16_t notes[] = {523, 659, 784, 1047};
    const int note_samples = SAMPLE_RATE * 120 / 1000; /* 120 ms per note */
    const int gap_samples = SAMPLE_RATE * 60 / 1000;   /* 60 ms gap */
    int16_t stereo_buf[BLOCK_SIZE * 2];

    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    for (int n = 0; n < 4; n++) {
        float phase = 0.0f;
        float phase_inc = (float)notes[n] / (float)SAMPLE_RATE;
        int remaining = note_samples;

        led_on();
        while (remaining > 0) {
            int chunk = (remaining < BLOCK_SIZE) ? remaining : BLOCK_SIZE;
            for (int i = 0; i < chunk; i++) {
                int16_t sample = (phase < 0.5f) ? AMPLITUDE : -AMPLITUDE;
                stereo_buf[i * 2] = sample;
                stereo_buf[i * 2 + 1] = sample;
                phase += phase_inc;
                if (phase >= 1.0f)
                    phase -= 1.0f;
            }
            size_t written;
            i2s_channel_write(s_tx_chan, stereo_buf, chunk * 2 * sizeof(int16_t), &written,
                              portMAX_DELAY);
            remaining -= chunk;
        }

        /* Silence gap */
        led_off();
        remaining = gap_samples;
        memset(stereo_buf, 0, sizeof(stereo_buf));
        while (remaining > 0) {
            int chunk = (remaining < BLOCK_SIZE) ? remaining : BLOCK_SIZE;
            size_t written;
            i2s_channel_write(s_tx_chan, stereo_buf, chunk * 2 * sizeof(int16_t), &written,
                              portMAX_DELAY);
            remaining -= chunk;
        }
    }

    ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
}

/* ── Audio Render Task (Core 1, high priority) ───────────── */

static void audio_render_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Audio render task started on core %d", xPortGetCoreID());

    float phase = 0.0f;
    float phase_b = 0.0f;
    float svf_low = 0.0f;
    float svf_band = 0.0f;
    float lfo_phase = 0.0f;
    float blip_phase = 0.0f;
    int16_t stereo_buf[BLOCK_SIZE * 2];

    const float block_period = (float)BLOCK_SIZE / (float)SAMPLE_RATE;

    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    bool prev_active = false;
    while (1) {
        float freq = s_synth.target_freq;
        bool active = s_synth.active;

        if (active != prev_active) {
            DBG_AXES("audio: active %d -> %d (freq=%.1f reason=%s)", prev_active, active, freq,
                     !active ? (freq < (float)MIN_FREQ ? "freq<MIN" : "!active") : "on");
            prev_active = active;
        }

        if (!active || freq < (float)MIN_FREQ) {
            memset(stereo_buf, 0, sizeof(stereo_buf));
            phase = 0.0f;
            phase_b = 0.0f;
            svf_low = 0.0f;
            svf_band = 0.0f;
        } else {
            waveform_t wave = s_synth.waveform;
            bool filter_on = s_synth.filter_enabled;
            float cutoff = s_synth.filter_cutoff;
            bool osc_b_on = s_synth.osc_b_enabled;
            float freq_b = s_synth.target_freq_b;
            waveform_t wave_b = s_synth.waveform_b;

            /* Update LFO (block-rate triangle) and apply to pitch/cutoff */
            lfo_target_t lfo_tgt = s_synth.lfo_target;
            if (lfo_tgt != LFO_TARGET_NONE) {
                lfo_phase += s_synth.lfo_rate * block_period;
                if (lfo_phase >= 1.0f)
                    lfo_phase -= (float)((int)lfo_phase);
                float lfo_val = lfo_triangle(lfo_phase); /* [-1, 1] */
                float depth = s_synth.lfo_depth;
                if (lfo_tgt == LFO_TARGET_PITCH || lfo_tgt == LFO_TARGET_BOTH) {
                    /* Up to one octave pitch mod at full depth */
                    freq *= powf(2.0f, depth * lfo_val);
                }
                if (lfo_tgt == LFO_TARGET_CUTOFF || lfo_tgt == LFO_TARGET_BOTH) {
                    /* Up to two octaves cutoff mod at full depth */
                    cutoff *= powf(2.0f, 2.0f * depth * lfo_val);
                    if (cutoff < FILTER_CUTOFF_MIN)
                        cutoff = FILTER_CUTOFF_MIN;
                    if (cutoff > FILTER_CUTOFF_MAX)
                        cutoff = FILTER_CUTOFF_MAX;
                }
            }

            float phase_inc = freq / (float)SAMPLE_RATE;
            float phase_inc_b = freq_b / (float)SAMPLE_RATE;

            /* Recompute filter coefficients once per block (TPT SVF) */
            float g_coef = tanf(3.14159265f * cutoff / (float)SAMPLE_RATE);
            float k_coef = 1.0f / s_synth.filter_resonance;
            float a1_coef = 1.0f / (1.0f + g_coef * (g_coef + k_coef));

            bool delay_on = s_synth.delay_enabled;
            int delay_samples = s_synth.delay_samples;
            float delay_fb = s_synth.delay_feedback;
            float delay_mix = s_synth.delay_mix;

            for (int i = 0; i < BLOCK_SIZE; i++) {
                int16_t sample = osc_sample(phase, wave);
                if (osc_b_on) {
                    int16_t sample_b = osc_sample(phase_b, wave_b);
                    /* Average A and B to stay within int16 range */
                    sample = (int16_t)(((int32_t)sample + (int32_t)sample_b) / 2);
                    phase_b += phase_inc_b;
                    if (phase_b >= 1.0f)
                        phase_b -= 1.0f;
                }
                if (filter_on) {
                    sample = svf_process(sample, g_coef, k_coef, a1_coef, &svf_low, &svf_band);
                }
                if (delay_on) {
                    sample = delay_process(sample, delay_samples, delay_fb, delay_mix);
                }
                stereo_buf[i * 2] = sample;
                stereo_buf[i * 2 + 1] = sample;
                phase += phase_inc;
                if (phase >= 1.0f)
                    phase -= 1.0f;
            }

            /* Defensive NaN guard — TPT SVF should be unconditionally
             * stable, but a stale config or coefficient corner could
             * still propagate NaN and silence the audio until the
             * active-flag reset. Clear to zero and log once per event. */
            if (isnan(svf_low) || isnan(svf_band)) {
                ESP_LOGW(TAG, "SVF state NaN recovered (cutoff=%.1f Q=%.2f)", cutoff,
                         s_synth.filter_resonance);
                svf_low = 0.0f;
                svf_band = 0.0f;
            }
        }

        /* Mix background drum engine into the synth output. Drums are
         * program material, so they pass through the master-volume stage
         * below. drums_render_block() does a saturating add into L/R. */
        drums_render_block(stereo_buf, BLOCK_SIZE);

        /* Voicing-announcement TTS clip (24 kHz → 44.1 kHz linear upsample,
         * saturating overlay mix). Subject to master volume like drums. */
        tts_player_render_block(stereo_buf, BLOCK_SIZE);

        /* Master volume attenuation — applied to (synth + drums) before the
         * blip mix so confirmation blips stay audible at low volumes. Read
         * once per block to avoid torn reads. */
        float master_vol = s_settings.master_volume;
        if (master_vol < 0.999f) {
            for (int i = 0; i < BLOCK_SIZE; i++) {
                int16_t left = stereo_buf[i * 2];
                int16_t right = stereo_buf[i * 2 + 1];
                stereo_buf[i * 2] = clip_i16((float)left * master_vol);
                stereo_buf[i * 2 + 1] = clip_i16((float)right * master_vol);
            }
        }

        /* Mix confirmation blip over the top (post-main synthesis). The main
         * voice is ducked so the blip is audible even at full volume. */
        int blip_rem = s_blip_samples_remaining;
        if (blip_rem > 0) {
            float blip_inc = s_blip_freq_hz / (float)SAMPLE_RATE;
            int total_samples = (SAMPLE_RATE * BLIP_DURATION_MS) / 1000;
            int count = (blip_rem < BLOCK_SIZE) ? blip_rem : BLOCK_SIZE;
            for (int i = 0; i < count; i++) {
                /* Linear fade-out over the last 20% of the blip to avoid a click. */
                int pos = total_samples - blip_rem + i;
                float env = 1.0f;
                int fade_start = (total_samples * 4) / 5;
                if (pos > fade_start) {
                    env = (float)(total_samples - pos) / (float)(total_samples - fade_start);
                    if (env < 0.0f)
                        env = 0.0f;
                }
                int idx = (int)(blip_phase * SINE_TABLE_SIZE) & (SINE_TABLE_SIZE - 1);
                int16_t blip_sample = (int16_t)((float)s_sine_table[idx] * env);
                int16_t ducked = stereo_buf[i * 2] >> BLIP_MAIN_DUCK_SHIFT;
                int16_t mixed = clip_i16((float)ducked + (float)blip_sample);
                stereo_buf[i * 2] = mixed;
                stereo_buf[i * 2 + 1] = mixed;
                blip_phase += blip_inc;
                if (blip_phase >= 1.0f)
                    blip_phase -= 1.0f;
            }
            s_blip_samples_remaining = blip_rem - count;
            if (s_blip_samples_remaining <= 0)
                blip_phase = 0.0f;
        }

        size_t written;
        i2s_channel_write(s_tx_chan, stereo_buf, sizeof(stereo_buf), &written, portMAX_DELAY);
    }
}

/* ── Settings Page Helpers ───────────────────────────────── */

#define SETTINGS_LADDER_GAP_MS 100

/* Play a sequence of `count` blips at the given frequency, 100 ms apart.
 * The blip engine is single-slot so we space the emissions out with a
 * short vTaskDelay — acceptable here because this runs on the control
 * task which is idle while editing settings. */
static void settings_play_ladder(int count, float freq_hz)
{
    if (count < 1)
        count = 1;
    if (count > 12)
        count = 12;
    for (int i = 0; i < count; i++) {
        synth_blip(freq_hz);
        vTaskDelay(pdMS_TO_TICKS(SETTINGS_LADDER_GAP_MS));
    }
}

/* Encode the current value of s_settings_cursor as an audible ladder. */
static void settings_play_value_ladder(int cursor)
{
    switch (cursor) {
        case SETTING_DRUM_PATTERN: {
            int n = s_settings.drum_pattern + 1;
            settings_play_ladder(n, BLIP_FREQ_NEUTRAL);
            break;
        }
        case SETTING_DRUM_VOLUME: {
            int n = (int)(s_settings.drum_volume * 10.0f + 0.5f) + 1;
            settings_play_ladder(n, BLIP_FREQ_NEUTRAL);
            break;
        }
        case SETTING_LFO_RATE:
            settings_play_ladder(3, BLIP_FREQ_NEUTRAL);
            break;
        case SETTING_LFO_DEPTH: {
            int n = (int)(s_settings.lfo_depth * 10.0f + 0.5f) + 1;
            settings_play_ladder(n, BLIP_FREQ_NEUTRAL);
            break;
        }
        case SETTING_LFO_TARGET: {
            int n = s_settings.lfo_target + 1;
            settings_play_ladder(n, BLIP_FREQ_NEUTRAL);
            break;
        }
        case SETTING_VOICE_ANNOUNCE: {
            /* 1 blip = off, 2 blips = on — so "more blips = more feature". */
            int n = s_settings.voice_announce ? 2 : 1;
            settings_play_ladder(n, BLIP_FREQ_NEUTRAL);
            break;
        }
        default:
            break;
    }
}

/* Clamp helpers — keep each setting within documented bounds. */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}
static inline int clampi(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* Apply a step (±1) to the currently-cursored setting. */
static void settings_adjust(int cursor, int direction)
{
    switch (cursor) {
        case SETTING_DRUM_PATTERN:
            s_settings.drum_pattern = clampi(s_settings.drum_pattern + direction, 0, 4);
            break;
        case SETTING_DRUM_VOLUME:
            s_settings.drum_volume = clampf(s_settings.drum_volume + 0.1f * direction, 0.0f, 1.0f);
            break;
        case SETTING_LFO_RATE:
            s_settings.lfo_rate_hz =
                clampf(s_settings.lfo_rate_hz + 0.5f * direction, LFO_RATE_MIN, LFO_RATE_MAX);
            break;
        case SETTING_LFO_DEPTH:
            s_settings.lfo_depth = clampf(s_settings.lfo_depth + 0.1f * direction, 0.0f, 1.0f);
            break;
        case SETTING_LFO_TARGET: {
            int t = s_settings.lfo_target + direction;
            /* Wrap within [LFO_TARGET_NONE .. LFO_TARGET_BOTH] */
            int count = LFO_TARGET_BOTH + 1;
            t = ((t % count) + count) % count;
            s_settings.lfo_target = t;
            break;
        }
        case SETTING_VOICE_ANNOUNCE:
            /* Bool field — any direction toggles it. */
            s_settings.voice_announce = !s_settings.voice_announce;
            break;
        default:
            break;
    }
}

/* Settings-edit d-pad handler. Debounces with SETTINGS_DPAD_REPEAT_MS
 * between retriggers while held. */
static void settings_edit_handle(const gamepad_state_t *gp)
{
    static uint32_t last_dpad_ms = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_dpad_ms < SETTINGS_DPAD_REPEAT_MS)
        return;

    if (gp->dpad & DPAD_LEFT) {
        s_settings_cursor = (s_settings_cursor + SETTING_COUNT - 1) % SETTING_COUNT;
        last_dpad_ms = now;
        synth_blip(BLIP_FREQ_NEUTRAL);
        vTaskDelay(pdMS_TO_TICKS(SETTINGS_LADDER_GAP_MS));
        settings_play_value_ladder(s_settings_cursor);
    } else if (gp->dpad & DPAD_RIGHT) {
        s_settings_cursor = (s_settings_cursor + 1) % SETTING_COUNT;
        last_dpad_ms = now;
        synth_blip(BLIP_FREQ_NEUTRAL);
        vTaskDelay(pdMS_TO_TICKS(SETTINGS_LADDER_GAP_MS));
        settings_play_value_ladder(s_settings_cursor);
    } else if (gp->dpad & DPAD_UP) {
        settings_adjust(s_settings_cursor, +1);
        last_dpad_ms = now;
        synth_blip(BLIP_FREQ_UP);
    } else if (gp->dpad & DPAD_DOWN) {
        settings_adjust(s_settings_cursor, -1);
        last_dpad_ms = now;
        synth_blip(BLIP_FREQ_DOWN);
    }
}

/* ── Control Task (Core 1, reads gamepad, updates synth) ── */

/* Apply per-voicing synth defaults on entry. Waveform comes from the
 * persistent voicing cfg; osc B, filter, LFO, and delay reset so the
 * user starts from a known state. Per-voicing toggles (dual_osc, arp,
 * delay) are re-applied on the first tick of the voicing function. */
static void apply_voicing_defaults(voicing_t voicing)
{
    const voicing_cfg_t cfg = s_cfg[voicing];
    synth_set_waveform(cfg.waveform);
    synth_set_osc_b(false, (float)MIN_FREQ, cfg.waveform);

    switch (voicing) {
        case VOICING_CONTINUOUS:
            synth_set_filter(true, TWEAK_CUTOFF_DEFAULT, TWEAK_RESONANCE_DEFAULT);
            break;
        case VOICING_DISCRETE:
            synth_set_filter(true, 5000.0f, 0.8f);
            break;
        case VOICING_ONE_SHOT:
        default:
            synth_set_filter(false, FILTER_CUTOFF_MAX, FILTER_Q_MIN);
            break;
    }

    /* LFO is driven per-tick by Continuous (from settings). Discrete
     * and One-shot silence it on entry. */
    if (voicing != VOICING_CONTINUOUS)
        synth_set_lfo(LFO_TARGET_NONE, 5.0f, 0.0f);

    /* Delay tail default; if cfg.delay is true, the voicing function
     * will re-enable on the next tick via synth_set_delay(). */
    if (cfg.delay) {
        int delay_samples = (int)(SAMPLE_RATE * s_tweak_delay_ms / 1000.0f);
        synth_set_delay(true, delay_samples, s_tweak_feedback, 0.5f);
    } else {
        synth_set_delay(false, 1, 0.0f, 0.0f);
    }
}

/* Handle a voicing switch: silence voices, reset transient state, play
 * the signature gesture, and restore the voicing's defaults. Per-voicing
 * cfg (dual_osc, delay, waveform, ...) is preserved across switches. */
static void enter_voicing(voicing_t new_voicing)
{
    tone_stop();
    piezo_voice_note_off(PIEZO_A);
    piezo_voice_note_off(PIEZO_B);
    s_voicing = new_voicing;
    s_arp_running = false;
    s_sfx.ticks_left = 0;
    reset_tweaks();

    led_on();
    vTaskDelay(pdMS_TO_TICKS(40));
    led_off();

    apply_voicing_defaults(new_voicing);
    if (s_settings.voice_announce) {
        tts_play_voicing(new_voicing);
    }
    play_voicing_signature(new_voicing);
    apply_voicing_defaults(new_voicing);

    ESP_LOGI(TAG, "Voicing: %d cfg={dual=%d drone=%d delay=%d arp=%d wave=%d}", s_voicing,
             s_cfg[s_voicing].dual_osc, s_cfg[s_voicing].drone_hold, s_cfg[s_voicing].delay,
             s_cfg[s_voicing].arp, s_cfg[s_voicing].waveform);
}

/* Handle RB-held + face-button toggle combos. Returns the bitmask of
 * face buttons "consumed" (suppressed from voicing dispatch) when RB is
 * held, so the normal face-button-as-note behavior doesn't fire. */
static uint16_t handle_toggle_combo(uint16_t face_pressed, bool rb_held, bool lb_pressed)
{
    if (!rb_held)
        return 0;

    uint16_t consumed = 0;

    if (face_pressed & BTN_A) {
        if (s_voicing == VOICING_CONTINUOUS) {
            s_cfg[VOICING_CONTINUOUS].dual_osc = !s_cfg[VOICING_CONTINUOUS].dual_osc;
            if (!s_cfg[VOICING_CONTINUOUS].dual_osc)
                s_cfg[VOICING_CONTINUOUS].drone_hold = false;
            toggle_cue(s_cfg[VOICING_CONTINUOUS].dual_osc);
            ESP_LOGI(TAG, "Toggle DUAL_OSC: %d", s_cfg[VOICING_CONTINUOUS].dual_osc);
        }
        consumed |= BTN_A;
    }
    if (face_pressed & BTN_B) {
        if (s_voicing == VOICING_CONTINUOUS) {
            s_cfg[VOICING_CONTINUOUS].drone_hold = !s_cfg[VOICING_CONTINUOUS].drone_hold;
            if (s_cfg[VOICING_CONTINUOUS].drone_hold)
                s_cfg[VOICING_CONTINUOUS].dual_osc = true;
            toggle_cue(s_cfg[VOICING_CONTINUOUS].drone_hold);
            ESP_LOGI(TAG, "Toggle DRONE_HOLD: %d", s_cfg[VOICING_CONTINUOUS].drone_hold);
        }
        consumed |= BTN_B;
    }
    if (face_pressed & BTN_X) {
        if (s_voicing == VOICING_CONTINUOUS || s_voicing == VOICING_DISCRETE) {
            s_cfg[s_voicing].delay = !s_cfg[s_voicing].delay;
            toggle_cue(s_cfg[s_voicing].delay);
            ESP_LOGI(TAG, "Toggle DELAY[%d]: %d", s_voicing, s_cfg[s_voicing].delay);
        }
        consumed |= BTN_X;
    }
    if (face_pressed & BTN_Y) {
        waveform_t next =
            (waveform_t)((s_cfg[s_voicing].waveform + 1) % (WAVE_NOISE)); /* skip noise */
        s_cfg[s_voicing].waveform = next;
        synth_set_waveform(next);
        waveform_cue(next);
        ESP_LOGI(TAG, "Cycle WAVEFORM[%d]: %d", s_voicing, next);
        consumed |= BTN_Y;
    }
    if (lb_pressed && s_voicing == VOICING_DISCRETE) {
        s_cfg[VOICING_DISCRETE].arp = !s_cfg[VOICING_DISCRETE].arp;
        if (!s_cfg[VOICING_DISCRETE].arp) {
            s_arp_running = false;
            tone_stop();
        }
        toggle_cue(s_cfg[VOICING_DISCRETE].arp);
        ESP_LOGI(TAG, "Toggle ARP: %d", s_cfg[VOICING_DISCRETE].arp);
    }

    return consumed;
}

/* Global d-pad: ↑/↓ nudges master_volume by ±0.05, ←/→ nudges tempo by
 * ±5 BPM. Auto-repeats at GLOBAL_DPAD_REPEAT_MS while held. */
static void handle_global_dpad(uint8_t dpad)
{
    static uint32_t last_repeat_ms = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_repeat_ms < GLOBAL_DPAD_REPEAT_MS)
        return;

    bool acted = false;
    if (dpad & DPAD_UP) {
        s_settings.master_volume = clampf(s_settings.master_volume + 0.05f, 0.0f, 1.0f);
        synth_blip(BLIP_FREQ_UP);
        acted = true;
    } else if (dpad & DPAD_DOWN) {
        s_settings.master_volume = clampf(s_settings.master_volume - 0.05f, 0.0f, 1.0f);
        synth_blip(BLIP_FREQ_DOWN);
        acted = true;
    } else if (dpad & DPAD_RIGHT) {
        s_settings.drum_tempo_bpm = clampi(s_settings.drum_tempo_bpm + 5, 60, 200);
        synth_blip(BLIP_FREQ_UP);
        acted = true;
    } else if (dpad & DPAD_LEFT) {
        s_settings.drum_tempo_bpm = clampi(s_settings.drum_tempo_bpm - 5, 60, 200);
        synth_blip(BLIP_FREQ_DOWN);
        acted = true;
    }
    if (acted)
        last_repeat_ms = now;
}

static void control_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Control task started on core %d", xPortGetCoreID());

    TickType_t last_wake = xTaskGetTickCount();
    static uint16_t s_prev_buttons_global = 0;
    static uint16_t s_home_prev_face = 0;
    static bool s_home_used_as_modifier = false;

    while (1) {
        gamepad_state_t gp;
        memcpy(&gp, (const void *)&s_gp, sizeof(gp));

        if (!gp.connected) {
            tone_stop();
            piezo_voice_note_off(PIEZO_A);
            piezo_voice_note_off(PIEZO_B);
            led_off();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        uint8_t misc_pressed = gp.misc_buttons & ~s_prev_misc;
        uint8_t misc_released = s_prev_misc & ~gp.misc_buttons;
        s_prev_misc = gp.misc_buttons;

        uint16_t btn_pressed_global = gp.buttons & ~s_prev_buttons_global;
        s_prev_buttons_global = gp.buttons;

        /* Voicing switch on Share button (rising edge) */
        if (misc_pressed & MISC_BACK) {
            voicing_t next = (voicing_t)((s_voicing + 1) % VOICING_COUNT);
            enter_voicing(next);
        }

        /* Home held → modifier for drum-pattern select. Face-button tap
         * while Home held assigns pattern 1..4 and marks Home as "used as
         * modifier" so the tap-release doesn't toggle drums off. */
        bool home_held = (gp.misc_buttons & MISC_HOME) != 0;
        if (home_held) {
            uint16_t face_pressed = gp.buttons & ~s_home_prev_face;
            s_home_prev_face = gp.buttons;
            int pat = 0;
            if (face_pressed & BTN_A)
                pat = 1;
            else if (face_pressed & BTN_B)
                pat = 2;
            else if (face_pressed & BTN_X)
                pat = 3;
            else if (face_pressed & BTN_Y)
                pat = 4;
            if (pat > 0) {
                s_settings.drum_pattern = pat;
                s_drum_on = true;
                drums_set_pattern(pat);
                synth_blip(BLIP_FREQ_UP);
                s_home_used_as_modifier = true;
                ESP_LOGI(TAG, "Drum pattern: %d (via Home+face)", pat);
            }
        } else {
            s_home_prev_face = gp.buttons;
        }

        /* Home released: only toggle drums if Home wasn't used as modifier
         * during this press. */
        if (misc_released & MISC_HOME) {
            if (s_home_used_as_modifier) {
                s_home_used_as_modifier = false;
            } else {
                s_drum_on = !s_drum_on;
                if (s_drum_on) {
                    int pat = s_settings.drum_pattern;
                    if (pat < 1 || pat > 4)
                        pat = 1;
                    drums_set_pattern(pat);
                    synth_blip(BLIP_FREQ_UP);
                } else {
                    drums_set_pattern(0);
                    synth_blip(BLIP_FREQ_DOWN);
                }
                ESP_LOGI(TAG, "Drums: %s", s_drum_on ? "on" : "off");
            }
        }

        /* LS-click (left thumb) resets per-mode tweak parameters to
         * defaults. Confirmation blip descends like "reset". */
        if (btn_pressed_global & BTN_THUMB_L) {
            reset_tweaks();
            synth_blip(BLIP_FREQ_DOWN);
            ESP_LOGI(TAG, "Tweaks reset to defaults");
        }

        /* Push live settings-page values into the drum engine so
         * d-pad-tempo and settings-volume changes take effect immediately. */
        drums_set_tempo(s_settings.drum_tempo_bpm);
        drums_set_volume(s_settings.drum_volume);

        /* Menu button toggles the settings-edit overlay (rising edge) */
        if (misc_pressed & MISC_SELECT) {
            s_settings_edit_active = !s_settings_edit_active;
            if (s_settings_edit_active) {
                tone_stop();
                piezo_voice_note_off(PIEZO_A);
                piezo_voice_note_off(PIEZO_B);
                synth_blip(BLIP_FREQ_UP);
                ESP_LOGI(TAG, "Settings edit: ENTER (cursor=%d)", s_settings_cursor);
                vTaskDelay(pdMS_TO_TICKS(SETTINGS_LADDER_GAP_MS));
                settings_play_value_ladder(s_settings_cursor);
            } else {
                synth_blip(BLIP_FREQ_DOWN);
                ESP_LOGI(TAG, "Settings edit: EXIT");
            }
        }

        /* Settings overlay: d-pad is navigation, mode dispatch suppressed. */
        if (s_settings_edit_active) {
            settings_edit_handle(&gp);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        /* Home held without a face-button hit also suppresses mode dispatch
         * (otherwise holding Home to try patterns would accidentally play
         * notes/scales). */
        if (home_held) {
            tone_stop();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        /* Global d-pad: volume (↑/↓) and tempo (←/→). */
        handle_global_dpad(gp.dpad);

        /* RB-held + face / LB toggles combos. Face buttons consumed by
         * toggle combos are masked out before reaching voicing dispatch
         * so the normal note/interval behavior doesn't double-fire. */
        bool rb_held = (gp.buttons & BTN_SHOULDER_R) != 0;
        uint16_t face_pressed = btn_pressed_global & BTN_FACE_MASK;
        bool lb_pressed_edge = (btn_pressed_global & BTN_SHOULDER_L) != 0;
        uint16_t consumed = handle_toggle_combo(face_pressed, rb_held, lb_pressed_edge);
        uint16_t face_for_voicing = face_pressed & ~consumed;
        /* While RB is held, suppress note/interval face-button semantics
         * entirely so the user can think of RB as a "modifier" key. */
        if (rb_held)
            face_for_voicing = 0;

        /* Periodic axis log (every ~10 ticks, 5 Hz) for diagnostics. */
        static int s_dbg_tick = 0;
        if (++s_dbg_tick >= 10) {
            s_dbg_tick = 0;
            DBG_AXES("lx=%d ly=%d rx=%d ry=%d btns=0x%04x dpad=0x%02x misc=0x%02x "
                     "voicing=%d active=%d freq=%.1f cutoff=%.1f Q=%.2f",
                     gp.axis_x, gp.axis_y, gp.axis_rx, gp.axis_ry, gp.buttons, gp.dpad,
                     gp.misc_buttons, s_voicing, s_synth.active, s_synth.target_freq,
                     s_synth.filter_cutoff, s_synth.filter_resonance);
        }

        /* Dispatch to current voicing */
        switch (s_voicing) {
            case VOICING_CONTINUOUS:
                voicing_continuous(&gp, face_for_voicing);
                break;
            case VOICING_DISCRETE:
                voicing_discrete(&gp, face_for_voicing);
                break;
            case VOICING_ONE_SHOT:
                voicing_one_shot(&gp, face_for_voicing);
                break;
            default:
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

/* ── Entry Point ─────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Gamepad Synth starting!");

    init_nvs();
    init_sine_table();
    drums_init();
    init_i2s();
    init_led();
    piezo_voice_init(PIEZO_A, PIEZO_A_PIN);
    piezo_voice_init(PIEZO_B, PIEZO_B_PIN);
    synth_set_waveform(WAVE_SAWTOOTH);
    synth_set_filter(true, 6000.0f, 1.5f);
    synth_set_lfo(LFO_TARGET_NONE, 5.0f, 0.0f);
    synth_set_delay(false, 1, 0.0f, 0.0f);

    /* Settings-page defaults (also set via static initializer; reassigned
     * here so any future reset-to-defaults path has a single code location
     * to mirror). */
    s_settings.master_volume = 0.7f;
    s_settings.drum_pattern = 0;
    s_settings.drum_tempo_bpm = 110;
    s_settings.drum_volume = 0.5f;
    s_settings.lfo_rate_hz = 5.0f;
    s_settings.lfo_depth = 0.3f;
    s_settings.lfo_target = LFO_TARGET_CUTOFF;
    s_settings.voice_announce = true;
    s_settings_edit_active = false;
    s_settings_cursor = 0;
    ESP_LOGI(TAG,
             "Settings: master_vol=%.2f drum_ptn=%d drum_bpm=%d drum_vol=%.2f "
             "lfo_rate=%.1f lfo_depth=%.2f lfo_tgt=%d voice=%d",
             s_settings.master_volume, s_settings.drum_pattern, s_settings.drum_tempo_bpm,
             s_settings.drum_volume, s_settings.lfo_rate_hz, s_settings.lfo_depth,
             s_settings.lfo_target, s_settings.voice_announce);

    play_startup_jingle();

    ESP_LOGI(TAG, "Starting audio render task on Core 1 (priority 10)...");
    xTaskCreatePinnedToCore(audio_render_task, "audio", 4096, NULL, 10, NULL, 1);

    ESP_LOGI(TAG, "Starting control task on Core 1 (priority 5)...");
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Starting Bluepad32 on Core 0 (will not return)...");
    ESP_LOGI(TAG, "Put controller in pairing mode (Xbox: hold pair button on top)");
    uni_esp32_main();
}
