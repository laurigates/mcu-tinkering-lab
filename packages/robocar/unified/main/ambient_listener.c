/**
 * @file ambient_listener.c
 * @brief Continuous PDM capture feeding the ambient speech gate.
 *
 * See ambient_listener.h for why this is a task rather than an inline call in the
 * planner, and why the loop deliberately contains no delay.
 */

#include "ambient_listener.h"

#include <string.h>

#include "ambient_audio.h"
#include "audio_player.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mic_dump.h"
#include "mic_pdm.h"
#include "pin_config.h"
#include "speech_trigger.h"
#include "voice_preroll.h"
#include "voice_turn.h"

static const char *TAG = "ambient_listener";

/** VAD auto-trigger cooldowns: between triggers inside an active conversation,
 *  and when idle. The first used to double as the fixed VAD recording window;
 *  VAD turns now end on silence (issue #616), so it is only a cooldown. */
#define VAD_CONV_COOLDOWN_MS 3500
#define VAD_IDLE_COOLDOWN_MS 10000

/** Bound on one frame's DMA wait. Deliberately not portMAX_DELAY: mic_pdm_read()
 *  takes MILLISECONDS, and portMAX_DELAY in a millisecond parameter is ~72 minutes
 *  at CONFIG_FREERTOS_HZ=1000, not "forever" — a wedged channel would then look
 *  like a hang rather than an error. Generous relative to the 64 ms frame so a
 *  timeout means something is genuinely wrong. */
#define LISTENER_READ_TIMEOUT_MS 1000

/** Bound on acquiring the mic. A voice turn holds the lock for its whole recording
 *  window (seconds), so this must comfortably exceed the longest such window;
 *  failing to get the lock is normal, not an error, and simply skips a frame. */
#define LISTENER_LOCK_TIMEOUT_MS 12000

/** Frame buffer at file scope, not on the task stack: 1024 int16 is 2 KB, which is
 *  half of AMBIENT_LISTENER_TASK_STACK_SIZE. Single-reader by construction (only
 *  this task touches it), so it needs no lock of its own. */
static int16_t s_frame[AMBIENT_FRAME_SAMPLES];

/** The pre-roll for hands-free voice turns (issue #616). Storage is PSRAM —
 *  1.5 s is 48 kB, far too much for internal RAM — allocated at start. Guarded
 *  by the microphone lock: offered here and taken by a voice turn only while
 *  holding it. */
static voice_preroll_t s_preroll;

static volatile bool s_running;
static volatile int16_t s_level_db;
static volatile uint32_t s_last_accept_ms;
static volatile uint32_t s_frames_accepted;
static volatile uint32_t s_frames_muted;
static uint32_t s_last_vad_trigger_ms;

/** When playback last went inactive. Seeded to 0 meaning "long ago", so the very
 *  first frames after boot are accepted rather than quarantined. */
static uint32_t s_last_playback_end_ms;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Track the falling edge of playback so the hangover has an anchor.
 *
 * ambient_capture_allowed() needs to know WHEN playback stopped, not merely that
 * it has. Nothing else in the firmware records that instant, and audio_player has
 * no reason to grow a timestamp for one consumer's benefit — so the edge is
 * detected here, by the only task that cares.
 */
static bool playback_active_edge(void)
{
    static bool was_active;
    const bool active = audio_player_is_active();
    if (was_active && !active) {
        s_last_playback_end_ms = now_ms();
    }
    was_active = active;
    return active;
}

static void ambient_listener_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Ambient listener running (%d Hz, %d-sample frames, core %d)", MIC_SAMPLE_RATE_HZ,
             AMBIENT_FRAME_SAMPLES, AMBIENT_LISTENER_TASK_CORE);

    for (;;) {
        /* Skipping a frame because a voice turn owns the mic is expected traffic,
         * not a fault — hence LOGD and no counter of its own. */
        if (mic_pdm_lock(LISTENER_LOCK_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGD(TAG, "mic busy, skipping frame");
            continue;
        }

        /* The start cue is sampled on both sides of the read: it lasts 320 ms
         * against a 64 ms frame, so a cue overlapping this frame is active at one
         * end of it or the other. */
        const bool cue_before = voice_turn_cue_active();
        size_t got = 0;
        const esp_err_t err =
            mic_pdm_read(s_frame, AMBIENT_FRAME_SAMPLES, &got, LISTENER_READ_TIMEOUT_MS);
        const bool cue = cue_before || voice_turn_cue_active();

        const uint32_t t = now_ms();
        const bool playing = playback_active_edge();
        const bool allowed = ambient_capture_allowed(playing, t, s_last_playback_end_ms,
                                                     AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT);

        /* Offered while the lock is still held, so a voice turn that takes the
         * lock next finds the ring complete and the DMA's next samples directly
         * after its last one. A failed read is a gap in time, so it empties the
         * ring the same way the playback quarantine does. */
        const bool read_ok = (err == ESP_OK && got != 0);
        voice_preroll_offer(&s_preroll, s_frame, got, read_ok && allowed, cue);
        mic_pdm_unlock();

        if (!read_ok) {
            /* A read failure must NOT reach the gate. An all-zero or partial frame
             * fingerprints as flat silence, which is a legitimate reading, so the
             * gate could not tell "the mic broke" from "the room went quiet" — and
             * a run of broken reads would eventually read as a brand-new soundscape
             * and license speech about nothing. Drop it here instead. */
            ESP_LOGW(TAG, "mic read failed: %s (%u samples)", esp_err_to_name(err), (unsigned)got);
            vTaskDelay(pdMS_TO_TICKS(100)); /* only place a delay belongs: a broken
                                             * read returns instantly, so without
                                             * this the loop would spin at full tilt
                                             * on a dead microphone. */
            speech_trigger_reset_run();
            continue;
        }

        if (!allowed) {
            s_frames_muted++;
            speech_trigger_reset_run();
            continue;
        }

        ambient_fingerprint_t fp;
        ambient_fingerprint_from_pcm(s_frame, got, &fp);
        ambient_audio_note(&fp, t);

        /* THIS FRAME's level, not the floor. Reporting the floor here (as an
         * earlier version did) made the `mic` status line print the same number
         * twice under two different labels, and destroyed the one comparison the
         * line exists for: level sitting hard on the floor with no spread is a
         * dead or muted microphone, while level moving above a settled floor is
         * a live one. Two equal numbers can never show that. */
        s_level_db = fp.level_db;
        s_last_accept_ms = t;
        s_frames_accepted++;

        /* Per-frame logging is DEBUG only: this loop runs at ~15 Hz and an INFO
         * line per frame would bury every other message in the monitor. The
         * tunable values ride the planner's 15 s line instead. */
        /* The speech trigger runs whether or not VAD is on, so `mic` can show its
         * scores while someone tunes it. The start cue is a 1 kHz beep — in band
         * and 200 ms long, i.e. speech-shaped by this rule — so it breaks the run
         * instead of being scored. */
        bool speech = false;
        if (cue) {
            speech_trigger_reset_run();
        } else {
            speech = speech_trigger_note(s_frame, got, fp.level_db, ambient_audio_floor_db(), t);
        }

        ESP_LOGD(TAG, "frame: %u samples, loud %u, shape %u, voice band %u%%, run %u ms",
                 (unsigned)got, ambient_audio_loud_score(t), ambient_audio_shape_score(t),
                 (unsigned)speech_trigger_last_share_pct(), (unsigned)speech_trigger_run_ms());

        mic_dump_maybe(s_frame, got);

        /* Hands-free voice engagement auto-trigger (VAD).
         * Fires when VAD is enabled, the robot is not busy or playing audio, and
         * speech-shaped audio has been sustained (speech_trigger.h, issue #617 —
         * it used to be a broadband loudness excursion, which a door slam passes
         * as easily as a voice). Rate-limited to 10 s between triggers when idle;
         * during an active conversation window (7 s after robot replied),
         * triggers as soon as playback and hangover have finished. */
        if (speech && voice_turn_get_vad() && !voice_turn_is_busy() && !audio_player_is_active()) {
            const bool in_conv = voice_turn_in_conversation();
            const uint32_t elapsed = t - s_last_vad_trigger_ms;
            const bool cooldown_ok =
                (s_last_vad_trigger_ms == 0) ||
                (in_conv ? (elapsed >= VAD_CONV_COOLDOWN_MS) : (elapsed >= VAD_IDLE_COOLDOWN_MS));

            if (cooldown_ok && voice_turn_request_vad() == ESP_OK) {
                s_last_vad_trigger_ms = t;
                ESP_LOGI(
                    TAG, "VAD auto-trigger (speech %u ms, voice band %u%%, +%d dB, in_conv=%d)",
                    (unsigned)speech_trigger_run_ms(), (unsigned)speech_trigger_last_share_pct(),
                    (int)(fp.level_db - ambient_audio_floor_db()), (int)in_conv);
                /* The next turn must be earned by fresh speech, not by the tail of
                 * this run once the listener gets the microphone back. */
                speech_trigger_reset_run();
            }
        }
    }
}

esp_err_t ambient_listener_start(void)
{
    if (s_running) {
        return ESP_OK;
    }
    if (!mic_pdm_is_ready()) {
        ESP_LOGW(TAG, "PDM microphone not ready — ambient gate will never report novelty");
        return ESP_ERR_INVALID_STATE;
    }

    speech_trigger_init();

    /* Non-fatal like everything else on this path: without the ring a
     * hands-free turn simply starts at the cue, as it did before issue #616. */
    int16_t *preroll = heap_caps_malloc(VOICE_PREROLL_SAMPLES * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!preroll) {
        ESP_LOGW(TAG, "no PSRAM for the %u-sample pre-roll — VAD turns start at the cue",
                 (unsigned)VOICE_PREROLL_SAMPLES);
    }
    voice_preroll_init(&s_preroll, preroll, preroll ? VOICE_PREROLL_SAMPLES : 0);

    const BaseType_t ok = xTaskCreatePinnedToCore(
        ambient_listener_task, "ambient_listener", AMBIENT_LISTENER_TASK_STACK_SIZE, NULL,
        AMBIENT_LISTENER_TASK_PRIORITY, NULL, AMBIENT_LISTENER_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create ambient listener task");
        return ESP_ERR_NO_MEM;
    }
    s_running = true;
    return ESP_OK;
}

bool ambient_listener_is_running(void)
{
    return s_running;
}

int16_t ambient_listener_level_db(void)
{
    return s_level_db;
}

uint32_t ambient_listener_last_accept_ms(void)
{
    return s_last_accept_ms;
}

uint32_t ambient_listener_frames_accepted(void)
{
    return s_frames_accepted;
}

uint32_t ambient_listener_frames_muted(void)
{
    return s_frames_muted;
}

size_t ambient_listener_take_preroll(int16_t *dst, size_t max)
{
    return voice_preroll_take(&s_preroll, dst, max);
}
