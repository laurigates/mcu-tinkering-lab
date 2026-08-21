/**
 * @file activity_trace.h
 * @brief What the robot is doing right now, on the LEDs and in one log line each:
 *        every camera capture, every HTTPS call to a Gemini endpoint.
 *
 * ## The problem
 *
 * Between two planner log lines fifteen seconds apart, the robot captures a
 * frame, opens a TLS connection, uploads a JPEG, waits on a model, parses a
 * reply and possibly streams back several seconds of audio. When something in
 * that chain is slow or wedged, the existing logs say almost nothing useful: a
 * capture that never returns and a request that hangs for its full 30 s timeout
 * both look, from the console, like a robot sitting still. And once the robot is
 * off its tether there is no console at all.
 *
 * So this module answers two questions that were previously guesswork:
 *
 *   - **Is it still doing anything?** The LEDs say so continuously, with no
 *     serial cable attached.
 *   - **Where did the time go, and what came back?** One line per event, and a
 *     `trace` command that totals them since boot.
 *
 * ## Two choke points, deliberately
 *
 * Every picture in this firmware comes from camera_capture(), and every endpoint
 * call from gemini_http_post() — the planner, the narrate helper, the TTS
 * renderer and a voice turn all funnel through that one function. Instrumenting
 * those two places covers the whole surface, and (more to the point) keeps
 * covering it when a fifth caller is added later. Instrumenting the call SITES
 * instead would have been the version that silently stops being true.
 *
 * ## What the LEDs mean
 *
 * The left LED is the camera; the right is the network. Colour is state, not
 * decoration:
 *
 * | LED   | Colour  | Meaning                                              |
 * |-------|---------|------------------------------------------------------|
 * | left  | white   | a frame was just captured (brief pulse)              |
 * | left  | red     | the last capture FAILED, and still is the last one   |
 * | right | blue    | a request is in flight — held for as long as it is   |
 * | right | green   | the last request returned 200 (brief pulse)          |
 * | right | yellow  | the last request was rate-limited (429)              |
 * | right | red     | the last request failed — transport error or non-200 |
 *
 * The distinction that earns the module its keep is **blue that stays on**.
 * A held blue is a request that has not come back, which is otherwise invisible
 * until the timeout expires; a blue that blinks briefly every fifteen seconds is
 * a healthy planner. Failure colours HOLD rather than pulse, so a fault that
 * happened while you were not watching is still on the robot when you look.
 *
 * ## Cost, and why instrumentation may never block
 *
 * The hot paths (a capture, an HTTP begin/end) only stamp a few words of state;
 * all I2C traffic to the PCA9685 happens on this module's own low-priority task,
 * which writes an LED only when its colour actually changes. Nothing here waits
 * on the amplifier, the network or a long mutex: an indicator that adds latency
 * to the thing it is measuring is worse than no indicator, because it moves the
 * number you are trying to read. The counter mutex is taken with a short timeout
 * and the update is DROPPED on contention — a lost count is a fair price, a
 * stalled planner is not.
 *
 * LEDs can be turned off (`trace led off`) without losing the counters, for when
 * the robot is being filmed or the LEDs are wanted for something else.
 */

#ifndef ACTIVITY_TRACE_H
#define ACTIVITY_TRACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Which remote endpoint a request is talking to.
 *
 * Kept as an enum rather than a free string so the counters can be a fixed
 * array and the `trace` table has a stable set of rows. ACTIVITY_EP_COUNT is
 * the array bound, never a valid endpoint.
 */
typedef enum {
    ACTIVITY_EP_PLANNER = 0, /**< gemini_backend_plan() — the 15 s driving loop. */
    ACTIVITY_EP_NARRATE,     /**< gemini_backend narrate helper. */
    ACTIVITY_EP_TTS,         /**< gemini_tts — speech synthesis. */
    ACTIVITY_EP_VOICE_TURN,  /**< voice_turn — push-to-talk. */
    ACTIVITY_EP_COUNT,
} activity_endpoint_t;

/**
 * @brief Start the indicator task.
 *
 * Non-fatal by the convention of this firmware: a robot whose indicators failed
 * to start must still drive, talk and answer the console. On failure the
 * recording calls below become cheap no-ops rather than errors, so no caller
 * needs to check whether tracing is up before reporting to it.
 *
 * Call after led_controller_init(). Safe to call twice.
 */
esp_err_t activity_trace_init(void);

/**
 * @brief Record one camera capture.
 *
 * @param ok          Whether a frame came back.
 * @param bytes       Encoded frame size; ignored when @p ok is false.
 * @param elapsed_ms  How long the capture took.
 *
 * Safe from any task, and before activity_trace_init().
 */
void activity_trace_camera(bool ok, size_t bytes, uint32_t elapsed_ms);

/**
 * @brief Record that a request to @p ep has just been sent.
 *
 * Must be paired with activity_trace_http_end() on every path, including error
 * paths — an unpaired begin leaves the endpoint showing as permanently in
 * flight, which is exactly the symptom this module exists to make trustworthy.
 */
void activity_trace_http_begin(activity_endpoint_t ep);

/**
 * @brief Record that a request to @p ep has returned.
 *
 * @param err         Result from the HTTP client.
 * @param status      HTTP status, or 0 when the transport itself failed.
 * @param elapsed_ms  Round-trip time.
 *
 * A 429 is counted separately from other failures rather than lumped in with
 * them: Gemini's free tier is a per-model requests-per-minute cap, and a run of
 * 429s means the loop is outrunning its quota — a pacing problem with a
 * different fix from a run of 500s. See .claude/rules/gemini-api.md §4.
 */
void activity_trace_http_end(activity_endpoint_t ep, esp_err_t err, int status,
                             uint32_t elapsed_ms);

/** @brief Print the `trace` status table to stdout (console command). */
void activity_trace_report(void);

/** @brief Zero every counter. Does not touch the LED setting. */
void activity_trace_reset(void);

/** @brief Enable or disable the LED indicators; counters keep running either
 *         way. Turning them off releases both LEDs immediately. */
void activity_trace_set_leds(bool enabled);

/** @brief Whether the LED indicators are enabled. */
bool activity_trace_leds_enabled(void);

#ifdef __cplusplus
}
#endif

#endif  // ACTIVITY_TRACE_H
