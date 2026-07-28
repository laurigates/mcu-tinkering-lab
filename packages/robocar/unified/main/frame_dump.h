/**
 * @file frame_dump.h
 * @brief Get the planner's actual camera frames off the board, and measure them.
 *
 * Answers one question that nothing in this firmware could answer before: is
 * Gemini being shown a real, discernible image, or a black frame? Every theory
 * about the robot narrating "it is really dark" was inference until this
 * existed — the sensor registers, the JPEG size and the frame itself had never
 * been read off a device.
 *
 * Two instruments, deliberately separate:
 *
 *  - frame_stats_log() runs on EVERY planner frame and is cheap. It decodes the
 *    JPEG at 1/8 scale and logs mean/p5/p50/p95 luma alongside the sensor's
 *    LIVE gain and exposure registers. That pairing is the diagnosis: high gain
 *    + maxed exposure + low luma means the sensor is starved and the ceiling
 *    needs raising; low gain + low exposure + low luma means the room is dark
 *    and the sensor is not even trying; low mean with a high p95 means backlit.
 *
 *  - frame_dump_maybe() emits the JPEG itself, base64-framed, for the first
 *    FRAME_DUMP_BOOT_FRAMES frames and on demand via the `snap` console
 *    command. This is the ground truth — the exact bytes handed to
 *    gemini_backend_plan(), dumped while the planner still holds the buffer, so
 *    there is no copy, no lifetime question and no chance of showing a
 *    different frame from the one the model saw.
 *
 * WIRE FORMAT (decoded by tools/decode-frame-dump.py):
 *
 *     SNAPBEGIN seq=<n> len=<bytes> crc=<crc32_le_hex> w=320 h=240 gain=<g> exp=<e> ceil=<c>
 *     SNAP <64 base64 chars>            (repeated; 48 raw bytes per line)
 *     SNAPEND seq=<n> lines=<k>
 *
 * Lines are self-delimiting because the console is shared: tasks on both cores
 * log concurrently and WILL interleave. The host filter keeps only "SNAP "
 * lines, and the length+CRC catch the console's silent truncation (see
 * frame_dump.c).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Source frame geometry — must match the camera_config_t in camera.c. */
#define CAMERA_FRAME_WIDTH 320
#define CAMERA_FRAME_HEIGHT 240

/** Frames auto-dumped after boot.
 *
 *  Non-zero by default on purpose: `just robocar-unified::monitor` is
 *  READ-ONLY (tools/esp32s3-monitor.sh never writes to the port) and resets the
 *  board on attach, so there is no way to type `snap` through it. Attaching the
 *  monitor therefore always captures from boot, which is exactly the window the
 *  dark-frame reports point at. */
#define FRAME_DUMP_BOOT_FRAMES 3

/**
 * @brief Arm the dumper for the next @p frames planner frames.
 *
 * Idempotent and cheap; 0 disarms.
 */
void frame_dump_arm(int frames);

/** @brief True while frames remain to be dumped. */
bool frame_dump_pending(void);

/**
 * @brief Emit @p jpeg over the console if armed, otherwise return immediately.
 *
 * Call with the planner's live framebuffer, before the frame is sent to Gemini
 * and before it is returned to the driver.
 *
 * Blocks for roughly one tick per 48 bytes (~250 ms for a 12 kB frame) so the
 * console TX path cannot starve the audio player, which shares core 1 at a
 * higher priority. Never call it from a task with a real-time deadline.
 */
void frame_dump_maybe(const uint8_t *jpeg, size_t len);

/**
 * @brief Log luma statistics and live sensor exposure for @p jpeg.
 *
 * Cheap enough to run on every planner frame. Logs "UNDECODABLE" rather than
 * failing if the JPEG will not decode — which is itself a finding.
 */
void frame_stats_log(const uint8_t *jpeg, size_t len);

#ifdef __cplusplus
}
#endif
