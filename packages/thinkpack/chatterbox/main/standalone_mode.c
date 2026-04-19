/**
 * @file standalone_mode.c
 * @brief Touch-to-record / touch-to-playback loop (no peers).
 */

#include "standalone_mode.h"

#include "audio_engine.h"
#include "esp_log.h"
#include "thinkpack_audio.h"

static const char *TAG = "standalone";

static thinkpack_audio_sm_t s_sm;

void standalone_mode_init(void)
{
    thinkpack_audio_sm_init(&s_sm);
}

static void handle_post_touch_state(thinkpack_audio_state_t st)
{
    switch (st) {
        case THINKPACK_AUDIO_STATE_RECORDING:
            ESP_LOGI(TAG, "recording...");
            audio_engine_record_start();
            break;
        case THINKPACK_AUDIO_STATE_PLAYING: {
            size_t n = 0;
            audio_engine_record_stop(&n);
            ESP_LOGI(TAG, "playback %u samples", (unsigned)n);
            audio_engine_playback();
            thinkpack_audio_sm_handle(&s_sm, THINKPACK_AUDIO_EVENT_CLIP_DONE);
            break;
        }
        case THINKPACK_AUDIO_STATE_IDLE:
            /* Nothing to do. */
            break;
    }
}

void standalone_mode_on_touch(bool pressed)
{
    thinkpack_audio_event_t ev =
        pressed ? THINKPACK_AUDIO_EVENT_TOUCH_START : THINKPACK_AUDIO_EVENT_TOUCH_END;
    thinkpack_audio_state_t st = thinkpack_audio_sm_handle(&s_sm, ev);
    handle_post_touch_state(st);
}

void standalone_mode_tick(void)
{
    if (s_sm.state != THINKPACK_AUDIO_STATE_RECORDING) {
        return;
    }
    bool limit = false;
    audio_engine_record_tick(&limit);
    if (limit) {
        thinkpack_audio_state_t st =
            thinkpack_audio_sm_handle(&s_sm, THINKPACK_AUDIO_EVENT_LIMIT_REACHED);
        handle_post_touch_state(st);
    }
}
