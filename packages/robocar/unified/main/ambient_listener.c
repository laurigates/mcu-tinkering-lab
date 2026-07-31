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
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mic_dump.h"
#include "mic_pdm.h"
#include "pin_config.h"

static const char *TAG = "ambient_listener";

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

static volatile bool s_running;
static volatile int16_t s_level_db;
static volatile uint32_t s_last_accept_ms;
static volatile uint32_t s_frames_accepted;
static volatile uint32_t s_frames_muted;

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

        size_t got = 0;
        const esp_err_t err =
            mic_pdm_read(s_frame, AMBIENT_FRAME_SAMPLES, &got, LISTENER_READ_TIMEOUT_MS);
        mic_pdm_unlock();

        if (err != ESP_OK || got == 0) {
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
            continue;
        }

        const uint32_t t = now_ms();
        const bool playing = playback_active_edge();

        if (!ambient_capture_allowed(playing, t, s_last_playback_end_ms,
                                     AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT)) {
            s_frames_muted++;
            continue;
        }

        ambient_fingerprint_t fp;
        ambient_fingerprint_from_pcm(s_frame, got, &fp);
        ambient_audio_note(&fp, t);

        s_level_db = ambient_audio_floor_db();
        s_last_accept_ms = t;
        s_frames_accepted++;

        /* Per-frame logging is DEBUG only: this loop runs at ~15 Hz and an INFO
         * line per frame would bury every other message in the monitor. The
         * tunable values ride the planner's 15 s line instead. */
        ESP_LOGD(TAG, "frame: %u samples, loud %u, shape %u", (unsigned)got,
                 ambient_audio_loud_score(), ambient_audio_shape_score());

        mic_dump_maybe(s_frame, got);
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
