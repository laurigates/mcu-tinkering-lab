/**
 * @file live_session_core.c
 * @brief The decisions of a Gemini Live session. See live_session_core.h.
 */

#include "live_session_core.h"

#include <stdlib.h>
#include <string.h>

const char *live_end_name(live_end_t end)
{
    switch (end) {
        case LIVE_END_NONE:
            return "none";
        case LIVE_END_IDLE:
            return "idle";
        case LIVE_END_CAP:
            return "cap";
        case LIVE_END_GOAWAY:
            return "goaway";
        case LIVE_END_ERROR:
            return "error";
        case LIVE_END_STOPPED:
            return "stopped";
    }
    return "?";
}

void live_session_begin(live_session_t *s, const live_session_cfg_t *cfg, uint32_t now_ms)
{
    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
    s->opened_ms = now_ms;
    s->last_activity_ms = now_ms;
}

/* Playback is activity, and its falling edge anchors both the hangover and the
 * idle window. Shared by the two callers so they agree on the edge. */
static void track_playback(live_session_t *s, bool playing, uint32_t now_ms)
{
    if (playing) {
        s->last_activity_ms = now_ms;
    } else if (s->was_playing) {
        s->playback_end_ms = now_ms;
        s->played_once = true;
        s->last_activity_ms = now_ms;
    }
    s->was_playing = playing;
}

bool live_session_upstream_allowed(live_session_t *s, bool playing, uint32_t now_ms)
{
    track_playback(s, playing, now_ms);
    const bool in_hangover =
        s->played_once && (uint32_t)(now_ms - s->playback_end_ms) < s->cfg.hangover_ms;
    const bool allowed = !playing && !s->model_speaking && !in_hangover;
    if (allowed) {
        s->chunks_sent++;
    } else {
        s->chunks_muted++;
    }
    return allowed;
}

void live_session_note_speech(live_session_t *s, uint32_t now_ms)
{
    s->last_activity_ms = now_ms;
}

/* Append at a UTF-8 boundary; the tail of a long reply is dropped. */
static void append_text(char *dst, size_t cap, const char *src)
{
    size_t len = strlen(dst);
    size_t n = strlen(src);
    if (len + n >= cap) {
        n = cap - 1 - len;
        while (n > 0 && ((unsigned char)src[n] & 0xC0) == 0x80) {
            n--;
        }
    }
    memcpy(dst + len, src, n);
    dst[len + n] = '\0';
}

unsigned live_session_note_server(live_session_t *s, const live_server_msg_t *m, uint32_t now_ms)
{
    unsigned ev = 0;
    if (m->audio_parts > 0 || m->input_text[0] != '\0' || m->output_text[0] != '\0' ||
        m->turn_complete || m->tool_call) {
        s->last_activity_ms = now_ms;
    }

    if (m->audio_parts > 0) {
        if (!s->model_speaking) {
            s->model_speaking = true;
            s->turn_text[0] = '\0';
            ev |= LIVE_EV_TURN_START;
        }
        s->audio_bytes += m->audio_bytes;
    }
    if (m->output_text[0] != '\0') {
        append_text(s->turn_text, sizeof(s->turn_text), m->output_text);
    }

    if (m->interrupted && s->model_speaking) {
        s->model_speaking = false;
        s->interrupts++;
        ev |= LIVE_EV_INTERRUPTED;
    } else if (m->turn_complete && s->model_speaking) {
        s->model_speaking = false;
        s->turns++;
        ev |= LIVE_EV_TURN_END;
    }

    if (m->tool_call) {
        s->tool_pending = true;
        memcpy(s->tool_id, m->tool_id, sizeof(s->tool_id));
        memcpy(s->tool_name, m->tool_name, sizeof(s->tool_name));
        ev |= LIVE_EV_TOOL;
        if (strcmp(m->tool_name, LIVE_PROTO_WAIT_TOOL) == 0) {
            s->waits++;
            ev |= LIVE_EV_WAIT;
        }
    }

    if (m->go_away) {
        s->go_away = true;
    }
    if (m->has_usage) {
        s->total_tokens = m->total_tokens;
    }
    return ev;
}

void live_session_tool_sent(live_session_t *s)
{
    s->tool_pending = false;
}

void live_session_fail(live_session_t *s)
{
    s->error = true;
}

void live_session_stop(live_session_t *s)
{
    s->stopped = true;
}

live_end_t live_session_should_end(live_session_t *s, bool playing, uint32_t now_ms)
{
    track_playback(s, playing, now_ms);
    if (s->stopped) {
        return LIVE_END_STOPPED;
    }
    if (s->error) {
        return LIVE_END_ERROR;
    }
    if (s->go_away) {
        return LIVE_END_GOAWAY;
    }
    if ((uint32_t)(now_ms - s->opened_ms) >= s->cfg.max_ms) {
        return LIVE_END_CAP;
    }
    if (!playing && (uint32_t)(now_ms - s->last_activity_ms) >= s->cfg.idle_ms) {
        return LIVE_END_IDLE;
    }
    return LIVE_END_NONE;
}

/* ------------------------------------------------------------- reassembly */

void live_rx_init(live_rx_t *rx, size_t max_bytes)
{
    memset(rx, 0, sizeof(*rx));
    rx->max = max_bytes;
}

void live_rx_free(live_rx_t *rx)
{
    free(rx->buf);
    live_rx_init(rx, rx->max);
}

int live_rx_feed(live_rx_t *rx, const char *data, size_t len, size_t offset, size_t frame_len,
                 bool fin)
{
    if (rx->complete) {
        rx->len = 0;
        rx->overflow = false;
        rx->complete = false;
    }
    if (!rx->overflow && len > 0) {
        if (rx->len + len > rx->max) {
            rx->overflow = true;
        } else {
            if (rx->len + len + 1 > rx->cap) {
                size_t want = rx->cap ? rx->cap * 2 : 4096;
                if (want < rx->len + len + 1) {
                    want = rx->len + len + 1;
                }
                if (want > rx->max + 1) {
                    want = rx->max + 1;
                }
                char *grown = realloc(rx->buf, want);
                if (!grown) {
                    rx->overflow = true; /* no memory: drop this message whole */
                } else {
                    rx->buf = grown;
                    rx->cap = want;
                }
            }
            if (!rx->overflow) {
                memcpy(rx->buf + rx->len, data, len);
                rx->len += len;
                rx->buf[rx->len] = '\0';
            }
        }
    }
    if (!fin || offset + len < frame_len) {
        return 0;
    }
    rx->complete = true;
    if (rx->overflow || !rx->buf) {
        return rx->overflow ? -1 : 0;
    }
    return 1;
}

uint32_t live_gain_apply(int16_t *pcm, size_t samples, int32_t dc, uint32_t gain_q8)
{
    uint32_t clipped = 0;
    for (size_t i = 0; i < samples; i++) {
        int32_t v = (((int32_t)pcm[i] - dc) * (int32_t)gain_q8) / 256;
        if (v > 32767) {
            v = 32767;
            clipped++;
        } else if (v < -32768) {
            v = -32768;
            clipped++;
        }
        pcm[i] = (int16_t)v;
    }
    return clipped;
}
