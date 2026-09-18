/**
 * @file speech.c
 * @brief Spoken check names. See speech.h for why this exists and what it is not.
 */

#include "speech.h"

#include <string.h>

#include "audio_player.h"
#include "checks.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "speech";

/** Bytes handed to audio_player_write() per call. Far larger than the 3-byte
 *  granularity that once thrashed the ring in robocar-unified, and small enough
 *  that a wedged ring is noticed within one timeout rather than one clip. */
#define SPEECH_CHUNK_BYTES 4096

#define SPEECH_RING_TIMEOUT_MS 2000

/** Ceiling on the wait for a clip to finish playing, in 100 ms ticks. The
 *  longest clip is ~2.5 s; this is generous on purpose, because the cost of
 *  being wrong is a truncated announcement and the cost of the ceiling is a
 *  bounded stall on an already-broken audio path. */
#define SPEECH_DRAIN_TICKS 60

/* EMBED_FILES exposes a _binary_<mangled path>_start/_end symbol pair per file.
 * Declared as arrays rather than pointers: the linker defines the SYMBOL at the
 * data's address, so `&sym` is the buffer and a pointer declaration would read
 * the first bytes of audio as an address. */
#define SPEECH_CLIP(sym)                                                      \
    extern const uint8_t sym##_start[] asm("_binary_tts_" #sym "_pcm_start"); \
    extern const uint8_t sym##_end[] asm("_binary_tts_" #sym "_pcm_end")

SPEECH_CLIP(psram);
SPEECH_CLIP(flash);
SPEECH_CLIP(buzzer);
SPEECH_CLIP(i2c_mux);
SPEECH_CLIP(i2c_scan);
SPEECH_CLIP(oled);
SPEECH_CLIP(leds);
SPEECH_CLIP(servos);
SPEECH_CLIP(motors);
SPEECH_CLIP(mcp23017);
SPEECH_CLIP(sonar);
SPEECH_CLIP(amp);
SPEECH_CLIP(mic);

/* Matched to g_checks by NAME rather than by index. An index table silently
 * mis-speaks every later check the moment one is inserted; a name lookup that
 * misses is merely silent, and speech_audit() reports it. */
typedef struct {
    const char *check_name;
    const uint8_t *begin;
    const uint8_t *end;
} speech_clip_t;

#define SPEECH_ENTRY(name, sym) {name, sym##_start, sym##_end}

static const speech_clip_t k_clips[] = {
    SPEECH_ENTRY("psram", psram),       SPEECH_ENTRY("flash", flash),
    SPEECH_ENTRY("buzzer", buzzer),     SPEECH_ENTRY("i2c-mux", i2c_mux),
    SPEECH_ENTRY("i2c-scan", i2c_scan), SPEECH_ENTRY("oled", oled),
    SPEECH_ENTRY("leds", leds),         SPEECH_ENTRY("servos", servos),
    SPEECH_ENTRY("motors", motors),     SPEECH_ENTRY("mcp23017", mcp23017),
    SPEECH_ENTRY("sonar", sonar),       SPEECH_ENTRY("amp", amp),
    SPEECH_ENTRY("mic", mic),
};

static const size_t k_clip_count = sizeof(k_clips) / sizeof(k_clips[0]);

static const speech_clip_t *find_clip(const char *check_name)
{
    if (!check_name) {
        return NULL;
    }
    for (size_t i = 0; i < k_clip_count; i++) {
        if (strcmp(k_clips[i].check_name, check_name) == 0) {
            return &k_clips[i];
        }
    }
    return NULL;
}

esp_err_t speech_init(void)
{
    const esp_err_t err = audio_player_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "audio_player_init failed (%s) — the sweep will beep but not speak",
                 esp_err_to_name(err));
    }
    return err;
}

bool speech_is_ready(void)
{
    return audio_player_is_ready();
}

void speech_say(const char *check_name)
{
    if (!audio_player_is_ready()) {
        return;
    }
    const speech_clip_t *clip = find_clip(check_name);
    if (!clip || clip->end <= clip->begin) {
        return;
    }

    const size_t total = (size_t)(clip->end - clip->begin);

    audio_player_begin_utterance();
    for (size_t off = 0; off < total; off += SPEECH_CHUNK_BYTES) {
        const size_t n = (total - off < SPEECH_CHUNK_BYTES) ? (total - off) : SPEECH_CHUNK_BYTES;
        if (audio_player_write(clip->begin + off, n, SPEECH_RING_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGW(TAG, "ring write failed %u bytes into '%s'", (unsigned)off, check_name);
            break;
        }
    }
    audio_player_end_utterance();

    /* Waited out rather than overlapped. The `mic` check sits centimetres from
     * the speaker and would measure this announcement instead of the room, and
     * the `amp` check would play its test tone over its own name. */
    for (int i = 0; i < SPEECH_DRAIN_TICKS && audio_player_is_active(); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void speech_audit(void)
{
    if (!audio_player_is_ready()) {
        ESP_LOGW(TAG, "audio path down — %u check names will not be spoken",
                 (unsigned)g_check_count);
        return;
    }

    size_t covered = 0;
    for (size_t i = 0; i < g_check_count; i++) {
        if (find_clip(g_checks[i].name)) {
            covered++;
        } else {
            ESP_LOGW(TAG, "no clip for check '%s' — it will run unannounced", g_checks[i].name);
        }
    }
    ESP_LOGI(TAG, "%u/%u checks have a spoken name", (unsigned)covered, (unsigned)g_check_count);
}
