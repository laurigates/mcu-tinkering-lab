/**
 * @file frame_dump.c
 * @brief Frame instrumentation implementation. See frame_dump.h for the design.
 */

#include "frame_dump.h"

#include <stdio.h>
#include <string.h>

#include "base64.h"
#include "camera.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "jpeg_decoder.h"

static const char *TAG = "frame";

/** Raw bytes encoded per emitted line. 48 -> exactly 64 base64 chars, so a line
 *  is 69 characters with its marker and newline: short enough that one console
 *  write is cheap, long enough that a 12 kB JPEG is ~250 lines rather than
 *  thousands. */
#define DUMP_BYTES_PER_LINE 48

/** Thumbnail geometry at JPEG_IMAGE_SCALE_1_8 for a 320x240 source. */
#define THUMB_W (CAMERA_FRAME_WIDTH / 8)
#define THUMB_H (CAMERA_FRAME_HEIGHT / 8)
#define THUMB_PIXELS (THUMB_W * THUMB_H)

/** Frames still to dump. Signed reads/writes of an int are atomic on this
 *  target and only the planner task decrements it, so no lock is needed. */
static volatile int s_armed_frames = FRAME_DUMP_BOOT_FRAMES;
static uint32_t s_seq = 0;

/* Luma histogram, reused across calls — 256 ints on the planner's 8 kB stack
 * would be tight next to the thumbnail. */
static uint16_t s_hist[256];

void frame_dump_arm(int frames)
{
    s_armed_frames = (frames < 0) ? 0 : frames;
    ESP_LOGI(TAG, "armed for %d frame dump(s)", s_armed_frames);
}

bool frame_dump_pending(void)
{
    return s_armed_frames > 0;
}

/* ------------------------------------------------------------------ */
/* Brightness statistics                                               */
/* ------------------------------------------------------------------ */

/** Percentile from the histogram. `rank` is a sample index, not a percentage. */
static uint8_t hist_percentile(uint32_t rank)
{
    uint32_t seen = 0;
    for (int v = 0; v < 256; v++) {
        seen += s_hist[v];
        if (seen > rank) {
            return (uint8_t)v;
        }
    }
    return 255;
}

void frame_stats_log(const uint8_t *jpeg, size_t len, scene_fingerprint_t *out_fp)
{
    /* Static rather than stack: 2400 bytes would be nearly a third of the
     * planner task's stack, next to a TLS request. */
    static uint8_t thumb[THUMB_PIXELS * 2];
    /* The luma plane the statistics and the scene fingerprint are both built
     * from. Static for the same reason, and separate from thumb[] because the
     * fingerprint needs the samples laid out as a plane, not interleaved. */
    static uint8_t luma[THUMB_PIXELS];

    if (out_fp) {
        memset(out_fp, 0, sizeof(*out_fp)); /* valid = false until we decode */
    }

    camera_exposure_t exp = {0};
    camera_read_exposure(&exp);
    char exp_str[64];
    camera_format_exposure(exp_str, sizeof(exp_str), &exp);

    /* Bound the decode against thumb[] BEFORE decoding. jpg2rgb565() passes
     * outbuf_size = UINT32_MAX (its own source calls that "a very bold
     * assumption"), so the library's overflow check is a no-op and the output
     * size comes from the JPEG's own SOF0 header. A corrupted header — the
     * first few hundred bytes of the file, exactly where a torn capture
     * corrupts — would then write past this fixed static buffer. Reading the
     * declared geometry first turns a silent .bss corruption into a log line.
     *
     * esp_jpeg_get_image_info() reports the SOURCE dimensions, not the scaled
     * output — comparing them against the 40x30 thumbnail rejected every frame
     * on the first board this ran on. Compare against the full frame and divide. */
    esp_jpeg_image_cfg_t probe = {
        .indata = (uint8_t *)jpeg,
        .indata_size = len,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_1_8,
    };
    esp_jpeg_image_output_t info = {0};
    if (esp_jpeg_get_image_info(&probe, &info) != ESP_OK || info.width != CAMERA_FRAME_WIDTH ||
        info.height != CAMERA_FRAME_HEIGHT) {
        ESP_LOGW(TAG, "jpeg=%u B luma=UNDECODABLE (source %ux%u, expected %dx%d) | %s",
                 (unsigned)len, info.width, info.height, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
                 exp_str);
        return;
    }

    /* jpg2rgb565 comes from the esp32-camera component's conversions/to_bmp.c,
     * which is compiled unconditionally — no new dependency and no sdkconfig
     * change. At 1/8 scale tjpgd still Huffman-decodes the whole entropy
     * stream, but at the 15 s planner cadence the cost is irrelevant. */
    if (!jpg2rgb565(jpeg, len, thumb, JPEG_IMAGE_SCALE_1_8)) {
        ESP_LOGW(TAG, "jpeg=%u B luma=UNDECODABLE | %s", (unsigned)len, exp_str);
        return;
    }

    memset(s_hist, 0, sizeof(s_hist));
    uint32_t sum = 0;
    for (int i = 0; i < THUMB_PIXELS; i++) {
        /* LITTLE-endian RGB565. jpg2rgb565() hard-codes flags.swap_color_bytes
         * = 0, and jpeg_decoder.c's output callback then writes
         * dst[idx] = LOBYTE(color); dst[idx + 1] = HIBYTE(color). Getting this
         * backwards does not merely skew the numbers — it decodes a different
         * pixel entirely, so the luma statistics would look like real telemetry
         * and be uncorrelated with the scene. */
        const uint16_t px = (uint16_t)(thumb[2 * i] | (thumb[2 * i + 1] << 8));
        const uint8_t r = (uint8_t)(((px >> 11) & 0x1F) << 3);
        const uint8_t g = (uint8_t)(((px >> 5) & 0x3F) << 2);
        const uint8_t b = (uint8_t)((px & 0x1F) << 3);
        const uint8_t y = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
        luma[i] = y;
        s_hist[y]++;
        sum += y;
    }

    if (out_fp) {
        scene_fingerprint_from_luma(luma, THUMB_W, THUMB_H, out_fp);
    }

    /* Percentiles matter as much as the mean: a backlit scene reads dark on
     * average but has a high p95, and that is a completely different problem
     * from an unlit room.
     *
     * The scene-change score is deliberately NOT logged here: at this point the
     * fingerprint has only just been built and not yet recorded, so any distance
     * this function could print would be the previous frame's. The planner logs
     * it against the live reference instead. */
    ESP_LOGI(TAG, "jpeg=%u B luma mean=%u p5=%u p50=%u p95=%u | %s", (unsigned)len,
             (unsigned)(sum / THUMB_PIXELS), hist_percentile((uint32_t)(THUMB_PIXELS * 5 / 100)),
             hist_percentile((uint32_t)(THUMB_PIXELS * 50 / 100)),
             hist_percentile((uint32_t)(THUMB_PIXELS * 95 / 100)), exp_str);
}

/* ------------------------------------------------------------------ */
/* JPEG dump                                                           */
/* ------------------------------------------------------------------ */

void frame_dump_maybe(const uint8_t *jpeg, size_t len)
{
    if (s_armed_frames <= 0 || !jpeg || len == 0) {
        return;
    }
    s_armed_frames--;
    s_seq++;

    camera_exposure_t exp = {0};
    const bool exp_ok = camera_read_exposure(&exp);

    /* Length and CRC are not optional. The USB-Serial-JTAG console DROPS tx
     * bytes silently once the host stops reading for TX_FLUSH_TIMEOUT_US
     * (50 ms in esp_driver_usb_serial_jtag) — usb_serial_jtag_write() returns
     * the full size regardless. Without a checksum a host hiccup produces a
     * short, structurally corrupt JPEG that looks exactly like a broken camera,
     * which is the one conclusion this tool exists to avoid fabricating.
     * esp_rom_crc32_le(0, ...) is bit-identical to Python's binascii.crc32. */
    const uint32_t crc = esp_rom_crc32_le(0, jpeg, len);

    /* The pid/gain/exp/ceil fields are omitted entirely when the sensor could
     * not be read, rather than sent as zeros — the host decoder treats their
     * absence as "unknown" and a zero as a real reading. pid travels with them
     * because the units are sensor-dependent. */
    if (exp_ok) {
        printf("SNAPBEGIN seq=%u len=%u crc=%08x w=%d h=%d pid=%04x gain=%u exp=%u ceil=%u\n",
               (unsigned)s_seq, (unsigned)len, (unsigned)crc, CAMERA_FRAME_WIDTH,
               CAMERA_FRAME_HEIGHT, exp.pid, (unsigned)exp.gain, (unsigned)exp.exposure,
               exp.gainceiling);
    } else {
        printf("SNAPBEGIN seq=%u len=%u crc=%08x w=%d h=%d\n", (unsigned)s_seq, (unsigned)len,
               (unsigned)crc, CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT);
    }

    size_t off = 0;
    unsigned lines = 0;
    while (off < len) {
        const size_t n = (len - off < DUMP_BYTES_PER_LINE) ? (len - off) : DUMP_BYTES_PER_LINE;
        char b64[DUMP_BYTES_PER_LINE / 3 * 4 + 1];
        size_t b64_len = sizeof(b64);
        if (base64_encode(jpeg + off, n, b64, &b64_len) != 0) {
            ESP_LOGE(TAG, "base64_encode failed at offset %u — dump aborted", (unsigned)off);
            break;
        }

        /* One printf per line, deliberately: usb_serial_jtag_write() takes the
         * console write lock for the whole buffer, so a single call keeps the
         * line atomic against the other tasks logging concurrently on both
         * cores. The host filter keys on the "SNAP " prefix to drop interleaved
         * ESP_LOG lines. */
        printf("SNAP %s\n", b64);
        off += n;
        lines++;

        /* Yield every line. The console TX path busy-spins while the host is
         * not reading, and this runs on the planner task (core 1, priority 3)
         * alongside the audio player (core 1, priority 5) — hogging the core
         * here would manufacture the very audio glitches we are also chasing.
         * At ~250 lines this spreads a 12 kB frame over ~250 ms, once. */
        vTaskDelay(1);
    }

    printf("SNAPEND seq=%u lines=%u\n", (unsigned)s_seq, lines);
    ESP_LOGI(TAG, "dumped frame seq=%u (%u bytes, %u lines, %d remaining)", (unsigned)s_seq,
             (unsigned)len, lines, s_armed_frames);
}
