/**
 * @file live_proto.h
 * @brief Gemini Live API (BidiGenerateContent) messages: build what the robot
 *        sends, classify what the server sends back. Pure C, no ESP-IDF.
 *
 * ADR-032 §1: the firmware speaks the Live protocol as JSON over a WebSocket,
 * with no SDK. The shapes here are the ones google-genai 2.34 puts on the wire
 * (captured offline from the SDK's own serialiser, the session the probe in
 * tools/live-probe/ measured), including its mix of camelCase and snake_case
 * field names: protobuf JSON accepts both, and copying the measured session
 * byte for byte leaves nothing to argue about when a request is refused.
 *
 * Client → server:
 *   - `setup` once, first: model, AUDIO modality, the persona's prebuilt voice,
 *     the system instruction, input and output transcription, sliding-window
 *     context compression, and one tool, `wait`.
 *   - `realtime_input.audio`: base64 `audio/pcm;rate=16000`, about 10 a second.
 *   - `tool_response`: the answer to a `wait` call, scheduled SILENT so the
 *     model does not talk about it.
 *
 * Server → client (one JSON object per WebSocket message):
 *   - `setupComplete`, `serverContent` (`modelTurn.parts[].inlineData`,
 *     `turnComplete`, `interrupted`, `generationComplete`,
 *     `inputTranscription`, `outputTranscription`), `toolCall`, `goAway`,
 *     `usageMetadata`. Several can share one message.
 *
 * Why `wait`: not every utterance needs a reaction, and a robot that fills
 * every silence or chimes in on every remark is tiresome. With no trigger word,
 * the model hears everything said in the session, so it needs a way to answer
 * with nothing. It is not addressee detection, which cannot be judged reliably
 * (Lauri, 2026-11-27). A Live session answers in audio, so the old path's
 * `__IGNORE__` text reply does not carry over (ADR-032 §4). The workstation
 * play sessions (tools/live-probe/play.py, 2026-11-16) showed that an
 * instruction to stay silent alone drew a spoken reply 4 times in 4, and a
 * `wait` tool drew `wait` 4 times in 4. It is not a body action, so the model
 * still cannot claim to move (#947).
 */

#ifndef LIVE_PROTO_H
#define LIVE_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "base64.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The Live model (ADR-032 §6). Same rule as every model id here
 *  (.claude/rules/gemini-api.md §1): when it 404s or closes 1008, check
 *  `just robocar::live-probe-models`, never guess a suffix. */
#define LIVE_PROTO_MODEL "gemini-3.8-live"

/** The URI google-genai builds for an API-key session (live.py). The key goes
 *  in the upgrade request's `x-goog-api-key` header, never in the URL, which
 *  reaches the log. */
#define LIVE_PROTO_URI                                       \
    "wss://generativelanguage.googleapis.com/ws/"            \
    "google.ai.generativelanguage.v1beta.GenerativeService." \
    "BidiGenerateContent"

#define LIVE_PROTO_INPUT_RATE_HZ 16000
#define LIVE_PROTO_OUTPUT_RATE_HZ 24000

/** The only output format the firmware plays: the TTS ring's. Measured in
 *  every chunk on 2026-11-07 (tools/live-probe/README.md § Measured). */
#define LIVE_PROTO_OUTPUT_MIME "audio/pcm;rate=24000"

/** The one tool declared (see the file comment). */
#define LIVE_PROTO_WAIT_TOOL "wait"

/** Room for a function call id and name. Ids seen from the SDK are short;
 *  a longer one is truncated and the response will not match, which the
 *  server answers by ignoring it — the `wait` was silent anyway. */
#define LIVE_PROTO_TOOL_ID_MAX 64
#define LIVE_PROTO_TOOL_NAME_MAX 32

/** Transcription text carried out of one message. Fragments are a few words;
 *  a longer one is truncated at a UTF-8 boundary. */
#define LIVE_PROTO_TEXT_MAX 256

/**
 * @brief Build the Live system instruction from a persona.
 *
 * The persona part is the probe's instruction (live_probe.py
 * `extract_persona()`), which kept `gemini-3.8-live` in Teuvo's register and
 * in Finnish on 2026-11-07. Native-audio models take no language code, so
 * the language rides in the instruction: Finnish wording for a persona whose
 * language code starts with "fi", English otherwise. Then one line telling the
 * model when to call `wait`.
 *
 * @return malloc'd string, or NULL on a NULL name or allocation failure.
 *         @p text_brief and @p tts_style may be NULL or empty.
 */
char *live_proto_build_instruction(const char *name, const char *text_brief, const char *tts_style,
                                   const char *language_code);

/**
 * @brief Build the `setup` message.
 *
 * @param model        Bare model id (LIVE_PROTO_MODEL); "models/" is prefixed.
 * @param voice        Prebuilt voice name; NULL or empty leaves the server's.
 * @param instruction  System instruction text.
 * @return malloc'd JSON, or NULL on a NULL model or instruction.
 */
char *live_proto_build_setup(const char *model, const char *voice, const char *instruction);

/**
 * @brief Exact buffer size (terminator included) for an audio message of
 *        @p pcm_bytes bytes of PCM.
 */
size_t live_proto_audio_msg_size(size_t pcm_bytes);

/**
 * @brief Build one `realtime_input.audio` message into @p out.
 *
 * No allocation: this runs ten times a second for the whole session, so the
 * caller keeps one buffer of live_proto_audio_msg_size() bytes.
 *
 * @return Message length (without the terminator), or 0 when @p cap is too
 *         small or an argument is NULL / empty.
 */
size_t live_proto_build_audio(const int16_t *pcm, size_t samples, char *out, size_t cap);

/** @brief The `audioStreamEnd` message, sent when the mic stream pauses. */
const char *live_proto_audio_stream_end(void);

/**
 * @brief Build the `tool_response` for a function call, scheduled SILENT.
 * @return malloc'd JSON, or NULL on a NULL id or name.
 */
char *live_proto_build_tool_response(const char *id, const char *name);

/** What one server message said. Zeroed by the parser before it fills it. */
typedef struct {
    bool setup_complete;
    bool turn_complete;
    bool interrupted;
    bool generation_complete;

    /** inlineData parts with LIVE_PROTO_OUTPUT_MIME, decoded through the sink. */
    uint32_t audio_parts;
    /** PCM bytes handed to the sink. */
    size_t audio_bytes;
    /** inlineData parts in any other format: skipped, never played at a
     *  guessed rate (a WAV at the wrong rate plays pitch-shifted). */
    uint32_t audio_parts_skipped;
    /** The sink returned false (the ring stalled). */
    bool sink_aborted;

    char input_text[LIVE_PROTO_TEXT_MAX];  /**< inputTranscription.text */
    char output_text[LIVE_PROTO_TEXT_MAX]; /**< outputTranscription.text */

    /** The first functionCalls entry, if any. */
    bool tool_call;
    char tool_id[LIVE_PROTO_TOOL_ID_MAX];
    char tool_name[LIVE_PROTO_TOOL_NAME_MAX];

    bool go_away;
    uint32_t go_away_ms; /**< goAway.timeLeft; 0 when absent or unparseable. */

    bool has_usage;
    uint32_t prompt_tokens;
    uint32_t response_tokens;
    uint32_t total_tokens;
} live_server_msg_t;

/**
 * @brief Classify one complete server message, decoding its reply audio.
 *
 * Audio goes straight to @p sink as PCM bytes (base64_stream, the decoder the
 * TTS path uses), in part order. @p sink may be NULL to count without decoding.
 *
 * @return 0 on success (including a message with nothing this firmware uses),
 *         -1 when @p json is not a JSON object.
 */
int live_proto_parse_server(const char *json, size_t len, live_server_msg_t *out,
                            base64_sink_fn sink, void *ctx);

/**
 * @brief Parse a protobuf Duration string ("10s", "9.5s", "0.250s") to ms.
 * @return true on success; false (and *ms = 0) otherwise.
 */
bool live_proto_parse_duration_ms(const char *s, uint32_t *ms);

#ifdef __cplusplus
}
#endif

#endif  // LIVE_PROTO_H
