/**
 * @file ambient_listener.h
 * @brief The FreeRTOS task that joins the PDM microphone to the ambient gate.
 *
 * ## The problem
 *
 * `ambient_audio.c` decides whether the soundscape has changed, but it is pure —
 * it never touches hardware and never learns the time. `mic_pdm.c` produces PCM
 * but has no opinion about it. Something has to run continuously, hand one to the
 * other, and know when NOT to. That is this file, and it is the only code in the
 * firmware that includes both.
 *
 * ## Why the planner must not do this inline
 *
 * The obvious cheap design is to capture a frame inside the planner loop, next to
 * the camera frame, and skip the task entirely. That does not work, and the reason
 * is arithmetic rather than taste: the planner runs every PLANNER_LOOP_PERIOD_MS
 * (15 s) and a frame is 64 ms, so it would observe 0.4% of elapsed time. A door
 * slam, or a whole spoken sentence, would land in the 99.6% and simply never be
 * seen. The gate would then look like a threshold that needs tuning, when in fact
 * nothing was ever measured — the worst kind of bug to chase, because every knob
 * you turn appears to do nothing for a reason you cannot observe.
 *
 * So the microphone is sampled continuously at ~15 frames/s and the gate keeps
 * LATCHED PEAKS (see ambient_audio.h); the planner reads the latch when it happens
 * to run. Continuous evidence, sparse consumption.
 *
 * ## Why there is no vTaskDelay in the loop
 *
 * `mic_pdm_read()` blocks on the I2S DMA until a full frame has been captured, so
 * the hardware sets the pace and the block IS the yield. Adding a delay "for
 * safety" would push the task off the DMA cadence and start dropping descriptors.
 * The yield-on-overrun hazard in .claude/rules/freertos-task-gotchas.md does not
 * apply here precisely because this loop never spins — but it is worth saying so
 * out loud, because the shape (a periodic task with no delay) looks exactly like
 * the bug that rule describes.
 *
 * ## Concurrency
 *
 * The mic has one reader at a time, enforced by the lock in mic_pdm.h. This task
 * takes it per frame and releases it immediately, so a voice turn can hold it for
 * an entire recording window without fighting a 15 Hz consumer. That also gives a
 * property worth having deliberately: while someone is talking to the robot, the
 * ambient gate learns nothing from the conversation.
 */

#ifndef AMBIENT_LISTENER_H
#define AMBIENT_LISTENER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the listener task.
 *
 * Requires mic_pdm_init() and ambient_audio_init() to have run. Non-fatal by
 * design: a board with no working microphone must still boot, drive and talk, so
 * a failure here is logged and the gate simply never reports novelty.
 */
esp_err_t ambient_listener_start(void);

/** @brief True once the task is running. False means the robot is deaf. */
bool ambient_listener_is_running(void);

/**
 * @brief Level of the most recent frame, in dBFS (negative; 0 = full scale).
 *
 * The live reading behind the `mic` console command. Paired with
 * ambient_audio_floor_db() it is what distinguishes a quiet room from a dead
 * microphone — a dead mic reads a constant floor with no excursion ever, which is
 * otherwise indistinguishable from a very well-behaved room.
 */
int16_t ambient_listener_level_db(void);

/** @brief Monotonic ms at which a frame was last ACCEPTED into the gate. */
uint32_t ambient_listener_last_accept_ms(void);

/** @brief Frames accepted into the gate since boot. */
uint32_t ambient_listener_frames_accepted(void);

/**
 * @brief Frames captured but discarded because the robot was making noise.
 *
 * Reported separately from accepted frames because the two failure modes read
 * identically from the outside: a gate that never fires because the room is quiet
 * looks like a gate that never fires because the speaker never shuts up.
 */
uint32_t ambient_listener_frames_muted(void);

#ifdef __cplusplus
}
#endif

#endif  // AMBIENT_LISTENER_H
