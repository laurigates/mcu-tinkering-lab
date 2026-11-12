/**
 * @file mic_dump.c
 * @brief Microphone frame instrumentation. See mic_dump.h for the design.
 */

#include "mic_dump.h"

#include <stdio.h>

#include "base64.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h" /* MIC_SAMPLE_RATE_HZ — reported so the host need not assume it */

static const char *TAG = "mic_dump";

/** Raw bytes encoded per emitted line. 48 -> exactly 64 base64 chars, matching
 *  frame_dump.c so both host decoders see the same line geometry. */
#define DUMP_BYTES_PER_LINE 48

/** Frames still to dump. Signed reads/writes of an int are atomic on this
 *  target; the console task arms it and the listener task decrements it, and a
 *  race there can only mis-dump one frame. */
static volatile int s_armed_frames = MIC_DUMP_BOOT_FRAMES;
static uint32_t s_seq = 0;

void mic_dump_arm(int frames)
{
    s_armed_frames = (frames < 0) ? 0 : frames;
    ESP_LOGI(TAG, "armed for %d microphone frame dump(s)", s_armed_frames);
}

bool mic_dump_pending(void)
{
    return s_armed_frames > 0;
}

void mic_dump_maybe(const int16_t *pcm, size_t samples)
{
    if (s_armed_frames <= 0 || !pcm || samples == 0) {
        return;
    }
    s_armed_frames--;
    s_seq++;

    const uint8_t *bytes = (const uint8_t *)pcm;
    const size_t len = samples * sizeof(int16_t);

    /* esp_rom_crc32_le(0, ...) is bit-identical to Python's binascii.crc32, so
     * the host check needs no reimplementation of the polynomial. */
    const uint32_t crc = esp_rom_crc32_le(0, bytes, len);

    /* The rate travels with the payload rather than being assumed by the
     * decoder: a wrong assumption there produces a WAV that plays at the wrong
     * speed, which is indistinguishable from a mis-clocked PDM channel. */
    printf("MICBEGIN seq=%u len=%u crc=%08x rate=%u bits=16 ch=1\n", (unsigned)s_seq, (unsigned)len,
           (unsigned)crc, (unsigned)MIC_SAMPLE_RATE_HZ);

    size_t off = 0;
    unsigned lines = 0;
    while (off < len) {
        const size_t n = (len - off < DUMP_BYTES_PER_LINE) ? (len - off) : DUMP_BYTES_PER_LINE;
        char b64[DUMP_BYTES_PER_LINE / 3 * 4 + 1];
        size_t b64_len = sizeof(b64);
        if (base64_encode(bytes + off, n, b64, &b64_len) != 0) {
            ESP_LOGE(TAG, "base64_encode failed at offset %u — dump aborted", (unsigned)off);
            break;
        }

        /* One printf per line: usb_serial_jtag_write() takes the console write
         * lock for the whole buffer, so a single call keeps the line atomic
         * against the other tasks logging concurrently on both cores. The host
         * filter keys on the "MIC " prefix to drop interleaved ESP_LOG lines. */
        printf("MIC %s\n", b64);
        off += n;
        lines++;

        /* Yield every line, exactly as frame_dump does. The console TX path
         * busy-spins while the host is not reading, and this runs on the
         * listener task (Core 1) alongside the audio player at a higher
         * priority — hogging the core here would manufacture audio glitches. */
        vTaskDelay(1);
    }

    printf("MICEND seq=%u lines=%u\n", (unsigned)s_seq, lines);
    ESP_LOGI(TAG, "dumped frame seq=%u (%u samples, %u lines, %d remaining)", (unsigned)s_seq,
             (unsigned)samples, lines, s_armed_frames);
}
