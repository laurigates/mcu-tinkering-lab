/**
 * @file live_session_core.h
 * @brief The decisions of a Gemini Live session, free of ESP-IDF (ADR-032).
 *
 * live_session.c owns the WebSocket, the microphone and the player; this
 * module owns every decision it makes about them, on an injected millisecond
 * clock, so the cases a bench cannot stage are host tests:
 *
 *   - **Half-duplex** (ADR-032 §3, phase 1). Nothing goes upstream from the
 *     first reply audio of a model turn until the amplifier has been quiet
 *     for `hangover_ms`. The speaker sits centimetres from the PDM mic; sent
 *     upstream, the robot's own voice would trip the server's VAD, interrupt
 *     the reply and answer itself. The mute starts at the first audio part,
 *     not at the amplifier: the player banks a preroll before it plays, and
 *     what the mic hears in that gap is the room, not a person.
 *   - **Close** (ADR-032 §4) on idle (no speech either way for `idle_ms`,
 *     counted from the end of playback), on the session cap, on `goAway`, on
 *     a transport error, or on request. Nothing reconnects; the next speech
 *     trigger opens a new session.
 *   - **Turns**: when to open and close an utterance on the player, the
 *     reply's transcript for the post-turn bookkeeping, and the `wait` calls
 *     that answer them.
 *   - **Reassembly** of a server message from WebSocket frame pieces.
 *   - **Gain**: the PDM mic is ~-40 dBFS at conversational distance
 *     (audio_clip.h). The session keeps one gain, measured on the pre-roll that
 *     holds the words that opened it, so the level does not pump between
 *     chunks.
 *
 * All times are uint32 milliseconds compared by subtraction, so the day-49
 * wrap is harmless.
 */

#ifndef LIVE_SESSION_CORE_H
#define LIVE_SESSION_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "live_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/** No speech either way for this long closes the session. ADR-032 §4 starts
 *  from the old path's 7 s conversation window. `voice live idle <s>`. */
#define LIVE_SESSION_IDLE_MS_DEFAULT 7000u

/** Session cap, well under the 10 min connection and 15 min audio-session
 *  limits (tools/live-probe/README.md § Facts). `voice live cap <s>`. */
#define LIVE_SESSION_MAX_MS_DEFAULT (5u * 60u * 1000u)

/** Largest server message accepted. Reply audio arrives in chunks of a
 *  fraction of a second (64 kB of base64 per second of audio); anything past
 *  this is dropped whole rather than parsed truncated. */
#define LIVE_RX_MAX_BYTES (256u * 1024u)

/** Room for one turn's reply transcript: the speech queue's line length. */
#define LIVE_SESSION_TURN_TEXT_MAX 320

typedef enum {
    LIVE_END_NONE = 0,
    LIVE_END_IDLE,
    LIVE_END_CAP,
    LIVE_END_GOAWAY,
    LIVE_END_ERROR,
    LIVE_END_STOPPED,
} live_end_t;

/** @brief Stable name for the session log line ("idle", "cap", ...). */
const char *live_end_name(live_end_t end);

typedef struct {
    uint32_t idle_ms;
    uint32_t max_ms;
    uint32_t hangover_ms; /**< Upstream mute after playback ends. */
} live_session_cfg_t;

/** What a server message asks the caller to do. Bit flags. */
enum {
    LIVE_EV_TURN_START = 1u << 0,  /**< First reply audio: open an utterance. */
    LIVE_EV_TURN_END = 1u << 1,    /**< Spoken turn complete: close it, bookkeeping. */
    LIVE_EV_INTERRUPTED = 1u << 2, /**< Drop what is queued (phase 2 barge-in). */
    LIVE_EV_TOOL = 1u << 3,        /**< A function call wants its response sent. */
    LIVE_EV_WAIT = 1u << 4,        /**< The model chose silence. */
};

typedef struct {
    live_session_cfg_t cfg;
    uint32_t opened_ms;
    uint32_t last_activity_ms;
    uint32_t playback_end_ms;
    bool was_playing;
    bool played_once; /**< playback_end_ms is meaningful */

    bool model_speaking; /**< Between a turn's first audio and its end. */
    bool go_away;
    bool error;
    bool stopped;

    /** The reply being spoken, from outputTranscription fragments. Read it on
     *  LIVE_EV_TURN_END, before the next message. */
    char turn_text[LIVE_SESSION_TURN_TEXT_MAX];

    bool tool_pending;
    char tool_id[LIVE_PROTO_TOOL_ID_MAX];
    char tool_name[LIVE_PROTO_TOOL_NAME_MAX];

    uint32_t chunks_sent;
    uint32_t chunks_muted;
    uint32_t turns;
    uint32_t waits;
    uint32_t interrupts;
    size_t audio_bytes;
    uint32_t total_tokens; /**< Last usageMetadata.totalTokenCount. */
} live_session_t;

/** @brief Start a session's state at @p now_ms. */
void live_session_begin(live_session_t *s, const live_session_cfg_t *cfg, uint32_t now_ms);

/**
 * @brief May the mic chunk read now go upstream?
 *
 * Call once per chunk with the player's state; it also tracks the playback
 * edge for the hangover and counts sent and muted chunks.
 */
bool live_session_upstream_allowed(live_session_t *s, bool playing, uint32_t now_ms);

/** @brief A mic chunk held speech: the person is still talking. */
void live_session_note_speech(live_session_t *s, uint32_t now_ms);

/** @brief Apply one parsed server message. @return LIVE_EV_* flags. */
unsigned live_session_note_server(live_session_t *s, const live_server_msg_t *m, uint32_t now_ms);

/** @brief The pending tool response was sent. */
void live_session_tool_sent(live_session_t *s);

/** @brief The transport failed; the session ends at the next check. */
void live_session_fail(live_session_t *s);

/** @brief End the session at the next check (console, shutdown). */
void live_session_stop(live_session_t *s);

/** @brief Should the session close now, and why. */
live_end_t live_session_should_end(live_session_t *s, bool playing, uint32_t now_ms);

/** Server message reassembly from WebSocket frame pieces. */
typedef struct {
    char *buf;
    size_t len;
    size_t cap;
    size_t max;
    bool overflow;
    bool complete; /**< The last feed completed a message; the next starts fresh. */
} live_rx_t;

void live_rx_init(live_rx_t *rx, size_t max_bytes);
void live_rx_free(live_rx_t *rx);

/**
 * @brief Feed one piece of a WebSocket frame.
 *
 * @param data        This piece.
 * @param offset      Its offset within the frame's payload.
 * @param frame_len   The frame's whole payload length.
 * @param fin         The frame is the last of its message.
 * @return 1 when a message is complete (rx->buf / rx->len, terminated, valid
 *         until the next feed); 0 when more is needed; -1 when a message
 *         completed but was dropped (too large, or no memory).
 */
int live_rx_feed(live_rx_t *rx, const char *data, size_t len, size_t offset, size_t frame_len,
                 bool fin);

/**
 * @brief Remove @p dc and apply @p gain_q8 (256 = 1.0x) in place, saturating.
 * @return Samples that hit full scale.
 */
uint32_t live_gain_apply(int16_t *pcm, size_t samples, int32_t dc, uint32_t gain_q8);

#ifdef __cplusplus
}
#endif

#endif  // LIVE_SESSION_CORE_H
