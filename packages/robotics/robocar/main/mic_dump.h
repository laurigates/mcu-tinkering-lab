/**
 * @file mic_dump.h
 * @brief Get the listener's actual microphone frames off the board.
 *
 * The frame_dump.h of the audio path, and for the same non-decorative reason.
 * Every claim about the camera was inference until frame_dump.c existed — and
 * the very first frame it emitted inverted the diagnosis, because the sensor
 * turned out to be an OV3660 and not the OV2640 the header had asserted for
 * years. Slice A is about to make exactly that class of claim about a
 * microphone nobody has listened to: is the amplitude sane, is there a DC term,
 * is the room's noise floor where the gate assumes it is, is the mic connected
 * at all. A loudness threshold over an unheard sensor is guesswork, and it will
 * present itself as a tuning problem.
 *
 * WIRE FORMAT (decoded by tools/decode-mic-dump.py):
 *
 *     MICBEGIN seq=<n> len=<bytes> crc=<crc32_le_hex> rate=16000 bits=16 ch=1
 *     MIC <64 base64 chars>             (repeated; 48 raw bytes per line)
 *     MICEND seq=<n> lines=<k>
 *
 * `len` counts BYTES of little-endian int16 PCM, not samples, so the host can
 * check it against the base64 payload without knowing the sample width.
 *
 * The length and the CRC32 are load-bearing. The USB-Serial-JTAG console
 * silently DROPS transmit bytes once the host stops reading for
 * TX_FLUSH_TIMEOUT_US (50 ms) and reports success anyway, so without a checksum
 * a host hiccup yields audio that is short and torn — which sounds exactly like
 * a broken microphone, the one conclusion this tool exists to avoid
 * fabricating. Lines are self-delimiting because tasks on both cores log
 * concurrently and WILL interleave.
 *
 * Same monitor caveat as `snap`: tools/esp32s3-monitor.sh is READ-ONLY and
 * resets the board on attach, so there is no way to type `mic dump` through it.
 * MIC_DUMP_BOOT_FRAMES is therefore non-zero — attaching the monitor always
 * captures the first few frames of room tone. Use screen/picocom if you want to
 * dump on demand.
 *
 * Cost, stated because the caller has to compensate for it: emitting one 64 ms
 * frame is ~43 console lines and roughly one tick each, so a dump takes far
 * longer than the frame it describes. The RX DMA keeps filling meanwhile (see
 * mic_pdm.h on staleness), so a caller must flush after dumping or the next
 * frames it measures describe a moment that has already passed.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Frames auto-dumped after boot. Non-zero for the reason above: the shipped
 *  monitor cannot type a command, so the only capture anyone gets for free is
 *  the one that starts at reset. Three 64 ms frames is ~200 ms of room tone —
 *  enough to see the noise floor and any DC offset, cheap enough that it does
 *  not delay the listener's first real measurements meaningfully. */
#define MIC_DUMP_BOOT_FRAMES 3

/**
 * @brief Arm the dumper for the next @p frames microphone frames.
 *
 * Idempotent and cheap; 0 or negative disarms.
 */
void mic_dump_arm(int frames);

/** @brief True while frames remain to be dumped.
 *
 *  Callers use this to decide whether to flush the RX backlog afterwards —
 *  ask BEFORE mic_dump_maybe(), since it consumes the arming. */
bool mic_dump_pending(void);

/**
 * @brief Emit @p pcm over the console if armed, otherwise return immediately.
 *
 * @param pcm      Mono little-endian int16 samples at MIC_SAMPLE_RATE_HZ.
 * @param samples  Sample count (not bytes).
 *
 * Blocks for roughly one tick per 48 bytes (~43 ticks for a 1024-sample frame)
 * so the console TX path cannot starve the audio player, which shares Core 1 at
 * a higher priority. Never call it from a task with a real-time deadline, and
 * never while holding the microphone lock — a voice turn waiting behind a dump
 * would lose the start of a sentence.
 */
void mic_dump_maybe(const int16_t *pcm, size_t samples);

#ifdef __cplusplus
}
#endif
