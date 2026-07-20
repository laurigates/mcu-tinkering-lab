/**
 * @file audio_player.c
 * @brief I2S playback implementation. See audio_player.h for the design.
 */

#include "audio_player.h"

#include <string.h>

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "audio_player";

/** DMA sizing — 4 descriptors x 256 frames ~= 42 ms of slack at 24 kHz. */
#define I2S_DMA_DESC_NUM 4
#define I2S_DMA_FRAME_NUM 256

/** Mono samples converted per I2S write. Keep the stereo scratch off the
 *  stack budget of a 4 kB task: 128 frames -> 512 bytes. */
#define PLAYER_CHUNK_SAMPLES 128

/** Idle grace before the I2S channel is powered down. The MAX98357A emits a
 *  faint hiss whenever BCLK is running, so it is disabled between utterances
 *  rather than left clocking silence. */
#define PLAYER_IDLE_TIMEOUT_MS 300

static i2s_chan_handle_t s_tx_chan = NULL;
static RingbufHandle_t s_ring = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_channel_active = false;
static volatile bool s_utterance_ended = false;

/* ------------------------------------------------------------------ */
/* I2S setup                                                           */
/* ------------------------------------------------------------------ */

static esp_err_t i2s_setup(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
    chan_cfg.auto_clear = true;  // emit zeros on underrun instead of stale DMA

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        // Stereo slots with a duplicated mono sample: the MAX98357A expects a
        // standard Philips frame and averages L+R when SD_MODE is floating.
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,  // MAX98357A derives its own clock
                .bclk = I2S_BCLK_PIN,
                .ws = I2S_LRCLK_PIN,
                .dout = I2S_DIN_PIN,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {false, false, false},
            },
    };

    err = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
    }
    return err;
}

/* ------------------------------------------------------------------ */
/* Player task                                                         */
/* ------------------------------------------------------------------ */

/** Scale to AUDIO_VOLUME_PCT and duplicate each mono sample into L and R. */
static void mono_to_stereo(const int16_t *mono, size_t count, int16_t *stereo)
{
    for (size_t i = 0; i < count; i++) {
        const int32_t scaled = ((int32_t)mono[i] * AUDIO_VOLUME_PCT) / 100;
        const int16_t s = (int16_t)scaled;  // in-range by construction (pct <= 100)
        stereo[2 * i] = s;
        stereo[2 * i + 1] = s;
    }
}

static void channel_set_active(bool active)
{
    if (active == s_channel_active) {
        return;
    }
    const esp_err_t err = active ? i2s_channel_enable(s_tx_chan) : i2s_channel_disable(s_tx_chan);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_%s failed: %s", active ? "enable" : "disable",
                 esp_err_to_name(err));
        return;
    }
    s_channel_active = active;
}

static void player_task(void *arg)
{
    (void)arg;
    int16_t stereo[PLAYER_CHUNK_SAMPLES * 2];

    for (;;) {
        size_t got = 0;
        void *item = xRingbufferReceiveUpTo(s_ring, &got, pdMS_TO_TICKS(PLAYER_IDLE_TIMEOUT_MS),
                                            PLAYER_CHUNK_SAMPLES * sizeof(int16_t));

        if (!item) {
            // Nothing arrived within the grace window — the utterance is over
            // (or stalled). Power the amp's clock down.
            if (s_channel_active) {
                channel_set_active(false);
                s_utterance_ended = false;
            }
            continue;
        }

        channel_set_active(true);

        const size_t samples = got / sizeof(int16_t);
        mono_to_stereo((const int16_t *)item, samples, stereo);
        vRingbufferReturnItem(s_ring, item);

        size_t written = 0;
        const esp_err_t err = i2s_channel_write(s_tx_chan, stereo, samples * 2 * sizeof(int16_t),
                                                &written, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_write failed: %s", esp_err_to_name(err));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t audio_player_init(void)
{
    if (s_task) {
        return ESP_OK;
    }

    // The ring lives in PSRAM: 96 kB of internal RAM would meaningfully cut
    // into the WiFi/TLS and camera framebuffer budget.
    s_ring = xRingbufferCreateWithCaps(AUDIO_RING_BYTES, RINGBUF_TYPE_BYTEBUF,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring) {
        ESP_LOGE(TAG, "failed to allocate %d-byte PSRAM ring", AUDIO_RING_BYTES);
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t err = i2s_setup();
    if (err != ESP_OK) {
        vRingbufferDeleteWithCaps(s_ring);
        s_ring = NULL;
        return err;
    }

    const BaseType_t ok =
        xTaskCreatePinnedToCore(player_task, "audio_player", AUDIO_PLAYER_TASK_STACK_SIZE, NULL,
                                AUDIO_PLAYER_TASK_PRIORITY, &s_task, AUDIO_PLAYER_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create player task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ready — %d Hz mono, BCLK=%d LRCLK=%d DIN=%d, %d kB PSRAM ring",
             AUDIO_SAMPLE_RATE_HZ, I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DIN_PIN,
             AUDIO_RING_BYTES / 1024);
    return ESP_OK;
}

esp_err_t audio_player_write(const uint8_t *pcm, size_t bytes, uint32_t timeout_ms)
{
    if (!s_ring) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!pcm || bytes == 0) {
        return ESP_OK;
    }

    s_utterance_ended = false;

    // Blocking send is deliberate backpressure: the download must not outrun
    // real-time playback, or a long utterance would need unbounded PSRAM.
    if (xRingbufferSend(s_ring, pcm, bytes, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "ring full for %ums — dropping %u bytes", (unsigned)timeout_ms,
                 (unsigned)bytes);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void audio_player_end_utterance(void)
{
    s_utterance_ended = true;
}

void audio_player_abort(void)
{
    if (!s_ring) {
        return;
    }

    // Drain whatever is queued without playing it.
    for (;;) {
        size_t got = 0;
        void *item = xRingbufferReceiveUpTo(s_ring, &got, 0, AUDIO_RING_BYTES);
        if (!item) {
            break;
        }
        vRingbufferReturnItem(s_ring, item);
    }

    s_utterance_ended = true;
}

bool audio_player_is_busy(void)
{
    if (!s_ring) {
        return false;
    }
    return s_channel_active || xRingbufferGetCurFreeSize(s_ring) < AUDIO_RING_BYTES;
}
